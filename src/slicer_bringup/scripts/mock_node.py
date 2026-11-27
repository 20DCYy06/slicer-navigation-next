#!/usr/bin/env python3
"""Isolated public-interface mock. It publishes no hardware command."""
import math
from collections import deque
from threading import Lock

import actionlib
import rospy
from geometry_msgs.msg import TwistStamped
from slicer_interfaces.msg import (
    NavigateToPointAction, NavigateToPointFeedback as Feedback,
    NavigateToPointGoal as Goal, NavigateToPointResult as Result, UgvStatus,
)
from slicer_interfaces.srv import Control, ControlResponse, Servo, ServoResponse


SCENARIOS = {
    "success", "preparing", "path_failure", "gnss_loss", "manual_override", "estop",
    "stop_unconfirmed",
}
FAULTS = {"none", "hold", "estop", "manual_override", "base_fault",
          "gnss_loss", "path_failure", "stop_unconfirmed"}


def validate(goal):
    if goal.mode == Goal.LOCAL_POSE:
        pose = goal.local_target
        if pose.header.frame_id != "map":
            return "仅支持 map 坐标系"
        age = (rospy.Time.now() - pose.header.stamp).to_sec()
        if not pose.header.stamp.is_zero() and not 0 <= age <= 5.0:
            return "本地目标时间戳已过期"
        if not all(map(math.isfinite, (pose.pose.position.x, pose.pose.position.y))) or pose.pose.position.z != 0:
            return "本地坐标无效"
        if goal.has_yaw:
            q = pose.pose.orientation
            if (not math.isclose(sum(v*v for v in (q.x, q.y, q.z, q.w)), 1.0, abs_tol=1e-3)
                    or abs(q.x) > 1e-3 or abs(q.y) > 1e-3):
                return "姿态四元数无效"
        if goal.require_geographic_verification:
            return "本地目标不能要求地理复核"
    elif goal.mode == Goal.WGS84:
        if not (-90 <= goal.latitude_deg <= 90 and -180 <= goal.longitude_deg <= 180):
            return "经纬度无效"
        if goal.has_altitude and not math.isfinite(goal.altitude_ellipsoid_m):
            return "椭球高无效"
        if goal.has_yaw and not math.isfinite(goal.yaw_enu_rad):
            return "ENU 航向无效"
    else:
        return "目标类型无效"
    limits = (goal.xy_tolerance_m, goal.yaw_tolerance_rad, goal.timeout.to_sec())
    if any(not math.isfinite(value) or value < 0 for value in limits) or goal.yaw_tolerance_rad > math.pi:
        return "容差或期限无效"
    return ""


class Mock:
    def __init__(self):
        self.events = deque()
        self.lock = Lock()
        self.active = None
        self.pending_stop = None
        self.grasp = None
        self.needs_release = False
        self.started = rospy.Time()
        self.last_fix = rospy.Time()
        self.last_result = None
        self.hold = True
        self.estop = False
        self.manual = False
        self.control_epoch = 0
        self.mission = None
        self.motion_since = rospy.Time.now()
        self.last_mission_stamp = rospy.Time()
        self.fault = "none"
        self.scenario = rospy.get_param("~scenario", "success")
        if self.scenario not in SCENARIOS:
            raise ValueError("Unknown mock scenario: " + self.scenario)
        self.status_pub = rospy.Publisher("/ugv/status", UgvStatus, queue_size=1)
        self.server = actionlib.ActionServer(
            "/ugv/navigate", NavigateToPointAction,
            lambda handle: self.enqueue("goal", handle),
            lambda handle: self.enqueue("cancel", handle),
            auto_start=False,
        )
        if rospy.get_param("~vision", False):
            try:
                from slicer_visual_grasp.msg import GraspAction, GraspResult
            except ImportError as error:
                raise RuntimeError("vision:=true requires the slicer_visual_grasp ROS overlay") from error
            self.grasp_result = GraspResult
            self.grasp_server = actionlib.ActionServer(
                "/ugv/grasp", GraspAction,
                lambda handle: self.enqueue("grasp_goal", handle),
                lambda handle: self.enqueue("grasp_cancel", handle),
                auto_start=False,
            )
            self.grasp_server.start()
        self.server.start()
        rospy.Service("/ugv/control/hold", Control, self.on_hold)
        rospy.Service("/ugv/control/release", Control, self.on_release)
        rospy.Service("/ugv/actuators/servo", Servo, self.on_servo)
        rospy.Subscriber("/ugv/cmd_vel/mission", TwistStamped, self.on_mission, queue_size=1)

    def enqueue(self, event, handle):
        with self.lock:
            self.events.append((event, handle))

    def on_hold(self, _request):
        with self.lock:
            if not self.hold:
                self.hold = True
                self.invalidate(rospy.Time.now())
                self.events.append(("hold", None))
        return ControlResponse(True, "Mock 已抑制自主和任务输出")

    def on_release(self, _request):
        with self.lock:
            if self.estop_active or self.fault in ("hold", "manual_override", "base_fault"):
                return ControlResponse(False, "Mock 控制锁或底盘故障仍有效")
            if self.hold or self.manual:
                self.hold = self.manual = False
                self.invalidate(rospy.Time.now())
        return ControlResponse(True, "Mock 已解除 hold")

    @staticmethod
    def on_servo(request):
        if request.channel != 1 or not 500 <= request.pulse_us <= 2500:
            return ServoResponse(False, False, "通道或脉宽无效")
        return ServoResponse(True, True, "Mock 模拟收到协议应答")

    def on_mission(self, msg):
        latched = (getattr(msg, "_connection_header", {}) or {}).get("latching") == "1"
        if (not latched and msg.header.frame_id in ("", "base_link") and
                all(map(math.isfinite, (msg.twist.linear.x, msg.twist.angular.z)))):
            self.enqueue("mission", msg)

    def invalidate(self, now):
        self.mission = None
        self.motion_since = now
        self.control_epoch += 1

    @property
    def estop_active(self):
        return self.estop or self.fault == "estop"

    @property
    def stop_feedback(self):
        return self.fault not in ("base_fault", "stop_unconfirmed") and self.scenario != "stop_unconfirmed"

    def mission_at(self, now):
        return (self.mission if self.mission is not None and
                0 <= (now - self.mission.header.stamp).to_sec() < .25 else None)

    def control_failure(self, now):
        if self.estop_active:
            return Result.ESTOP, "Mock 急停"
        if self.fault == "base_fault":
            return Result.BASE_FAULT, "Mock 底盘反馈不可用"
        if self.manual:
            return Result.MANUAL_OVERRIDE, "Mock 人工接管"
        if self.hold:
            return Result.HELD, "Mock 底盘已驻车"
        if self.grasp or self.mission_at(now) is not None:
            return Result.HELD, "Mock 任务控制权已被占用"
        return None

    def update_fault(self, now):
        fault = rospy.get_param_cached("~fault", "none")
        if not isinstance(fault, str) or fault not in FAULTS:
            rospy.logwarn_throttle(5., "Unknown Mock fault: %r", fault)
            return
        if fault != self.fault:
            with self.lock:
                previous, self.fault = self.fault, fault
                if fault in ("hold", "estop"):
                    self.hold = True
                elif fault == "manual_override":
                    self.manual = True
                if fault in ("hold", "estop", "manual_override", "base_fault") or previous == "base_fault":
                    self.invalidate(now)

    def accept_mission(self, msg, now):
        stamp = msg.header.stamp
        with self.lock:
            if (self.hold or self.estop_active or self.manual or self.grasp or self.fault == "base_fault"
                    or stamp <= max(self.motion_since, self.last_mission_stamp)
                    or not 0 <= (now - stamp).to_sec() < .25):
                return
            self.last_mission_stamp = stamp
            if self.active is not None and self.mission_at(now) is None:
                self.invalidate(now)
                self.begin_stop(now, Result.HELD, "Mock 任务流接管", "set_aborted", stop_commanded=False)
            self.mission = msg

    def finish_grasp(self, handle, terminal, reason, success=False):
        result = self.grasp_result(success=success, needs_release=self.needs_release, message=reason)
        getattr(handle, terminal)(result, reason)

    def start_grasp(self, handle, now):
        goal = handle.get_goal()
        handle.set_accepted("Mock 已接受视觉目标")
        with self.lock:
            busy = (self.mission is not None and
                    0 <= (now - self.mission.header.stamp).to_sec() <= .25)
            if goal.command not in (goal.PICK, goal.RELEASE):
                reason = "Mock 未知抓取指令"
            elif self.hold or self.estop_active or self.manual or not self.stop_feedback:
                reason = "Mock 底盘未就绪"
            elif self.active or self.pending_stop or self.grasp or busy:
                reason = "Mock 底盘控制权已被占用"
            elif goal.command == goal.PICK and self.needs_release:
                reason = "Mock 需要先释放"
            else:
                self.grasp = (handle, now)
                return
        self.finish_grasp(handle, "set_aborted", reason)

    def finish(self, handle, code, reason, now, terminal, stop_commanded=False,
               stop_valid=True, stopped=True):
        goal = handle.get_goal()
        result = Result(
            code=code, reason=reason, stop_commanded=stop_commanded,
            stop_valid=stop_valid, stopped=stopped,
        )
        if goal.mode == Goal.WGS84:
            result.latitude_deg = goal.latitude_deg
            result.longitude_deg = goal.longitude_deg
            result.has_altitude = goal.has_altitude
            result.altitude_ellipsoid_m = goal.altitude_ellipsoid_m
        if code == Result.SUCCESS:
            result.position_error_valid = True
            result.error_source = "mock"
        getattr(handle, terminal)(result, reason)
        self.last_result = (handle.get_goal_id().id, code, now)

    def begin_stop(self, now, code, reason, terminal="set_aborted", stop_commanded=True):
        self.pending_stop = (self.active, now, code, reason, terminal, stop_commanded)
        self.active = None

    def finish_stop(self, now, force=False):
        if self.pending_stop is None:
            return
        handle, since, code, reason, terminal, commanded = self.pending_stop
        if not force and now <= since:
            return
        mission = self.mission_at(now)
        moving = mission is not None and bool(mission.twist.linear.x or mission.twist.angular.z)
        valid = self.stop_feedback and now > since
        stopped = valid and not moving and self.grasp is None
        if code == Result.SUCCESS:
            issue = self.control_failure(now)
            if issue and issue[0] in (Result.ESTOP, Result.MANUAL_OVERRIDE, Result.HELD):
                code, reason = issue
                terminal, commanded = "set_aborted", False
        if code in (Result.SUCCESS, Result.CANCELED) and not stopped:
            code, reason, terminal = Result.STOP_NOT_CONFIRMED, "Mock 无有效停车观测", "set_aborted"
        self.finish(handle, code, reason, now, terminal, commanded, valid, stopped)
        self.pending_stop = None

    def tick(self, now):
        self.update_fault(now)
        self.finish_stop(now)
        with self.lock:
            events = list(self.events)
            self.events.clear()
        for event, handle in events:
            if event == "hold":
                if self.active is not None:
                    self.begin_stop(now, Result.HELD, "Mock 底盘已驻车", stop_commanded=False)
                if self.grasp is not None:
                    self.finish_grasp(self.grasp[0], "set_aborted", "Mock 底盘已驻车")
                    self.grasp = None
                continue
            if event == "mission":
                self.accept_mission(handle, now)
                continue
            if event == "grasp_goal":
                self.start_grasp(handle, now)
                continue
            if event == "grasp_cancel":
                if self.grasp and handle.get_goal_id().id == self.grasp[0].get_goal_id().id:
                    self.finish_grasp(self.grasp[0], "set_canceled", "Mock 抓取任务已取消")
                    self.grasp = None
                continue
            if event == "cancel":
                if self.active and handle.get_goal_id().id == self.active.get_goal_id().id:
                    self.begin_stop(now, Result.CANCELED, "Mock 已取消", "set_canceled")
                continue

            reason = validate(handle.get_goal())
            if reason:
                self.finish(handle, Result.INVALID_GOAL, reason, now, "set_rejected",
                            stop_valid=False, stopped=False)
                continue
            self.finish_stop(now, force=True)
            if self.active:
                self.finish(self.active, Result.REPLACED, "Mock 已替换", now,
                            "set_canceled", stop_valid=False, stopped=False)
            handle.set_accepted("Mock 已接受")
            self.active = handle
            self.started = now
            issue = self.control_failure(now)
            if issue:
                self.begin_stop(now, *issue, stop_commanded=False)

        phase = Feedback.IDLE
        issue = self.control_failure(now)
        if self.grasp is not None and (self.hold or self.manual or self.estop_active or self.fault == "base_fault"):
            self.finish_grasp(self.grasp[0], "set_aborted", issue[1])
            self.grasp = None
        if issue and self.active:
            self.begin_stop(now, *issue, stop_commanded=False)
        if self.active:
            goal = self.active.get_goal()
            age = (now - self.started).to_sec()
            preparing = goal.mode == Goal.WGS84 and age < 1.5
            phase = Feedback.PREPARING if preparing else Feedback.NAVIGATING
            if self.scenario == "preparing" and goal.mode == Goal.WGS84:
                phase = Feedback.PREPARING
            elif 1.5 <= age < 1.7 or goal.mode == Goal.LOCAL_POSE and age < 0.2:
                phase = Feedback.PLANNING
            elif goal.require_geographic_verification and age > 2.5:
                phase = Feedback.VERIFYING
            timeout = goal.timeout.to_sec() or 120.0
            ending = None
            if age >= timeout:
                ending = (Result.TIMEOUT, "Mock 任务超时", "set_aborted")
            elif age >= 0.8 and self.scenario == "manual_override":
                with self.lock:
                    self.manual = True
                    self.invalidate(now)
                ending = (Result.MANUAL_OVERRIDE, "Mock 人工接管", "set_aborted")
            elif age >= 0.8 and self.scenario == "estop":
                with self.lock:
                    self.estop = self.hold = True
                    self.invalidate(now)
                ending = (Result.ESTOP, "Mock 急停", "set_aborted")
            elif self.fault == "path_failure" or age >= 0.8 and self.scenario == "path_failure":
                ending = (Result.NO_PATH, "Mock 无可行路径", "set_aborted")
            elif goal.mode == Goal.WGS84 and (self.fault == "gnss_loss" or
                                               age >= 0.8 and self.scenario == "gnss_loss"):
                ending = (Result.LOCALIZATION_UNAVAILABLE, "Mock GNSS 失效", "set_aborted")
            elif age >= 3.0 and phase != Feedback.PREPARING:
                ending = (Result.SUCCESS, "Mock 已到点并停车", "set_succeeded")
            if ending:
                code, reason, terminal = ending
                handoff = code in (Result.ESTOP, Result.MANUAL_OVERRIDE)
                self.begin_stop(now, code, reason, terminal, stop_commanded=not handoff)
                phase = Feedback.IDLE
            else:
                feedback = Feedback(
                    phase=phase, goal_id=self.active.get_goal_id().id, reason="Mock 模拟",
                )
                if goal.mode == Goal.LOCAL_POSE:
                    feedback.projected_target = goal.local_target
                    feedback.projected_target_valid = True
                self.active.publish_feedback(feedback)
        if self.grasp and (now - self.grasp[1]).to_sec() >= 1.0:
            handle, _ = self.grasp
            goal = handle.get_goal()
            self.needs_release = goal.command == goal.PICK
            reason = "Mock 抓取完成" if self.needs_release else "Mock 释放完成"
            self.finish_grasp(handle, "set_succeeded", reason, success=True)
            self.grasp = None
        if self.pending_stop is not None:
            handle, _, _, reason, _, _ = self.pending_stop
            handle.publish_feedback(Feedback(phase=Feedback.STOPPING, goal_id=handle.get_goal_id().id, reason=reason))
        self.publish_status(now, phase)

    def publish_status(self, now, phase):
        status = UgvStatus(base_online=self.fault != "base_fault", stop_valid=self.stop_feedback, reason="Mock 模拟")
        status.header.stamp = now
        status.pose.header.stamp = now
        status.pose.header.frame_id = "map"
        status.pose.pose.orientation.w = 1.0
        status.pose_valid = True
        with self.lock:
            mission = self.mission_at(now)
            status.hold = self.hold
            status.manual_override = self.manual
            status.estop = self.estop_active
            status.control_epoch = self.control_epoch
        moving = mission is not None and bool(mission.twist.linear.x or mission.twist.angular.z)
        status.stopped = status.stop_valid and not moving and self.active is None and self.grasp is None
        status.velocity_valid = status.stop_valid
        status.velocity.header.stamp = now
        status.velocity.header.frame_id = "base_link"
        status.active_velocity_source = "zero"
        if moving:
            status.active_velocity_source = "mission"
            status.velocity = mission
        elif self.active is not None:
            status.goal_id = self.active.get_goal_id().id
            status.nav_goal_id = status.goal_id
            status.phase = phase
            status.active_velocity_source = "nav"
        elif self.grasp is not None:
            status.active_velocity_source = "mission"
        if self.pending_stop is not None:
            status.goal_id = self.pending_stop[0].get_goal_id().id
            status.phase = UgvStatus.STOPPING
        if self.fault != "gnss_loss" and not (self.scenario == "gnss_loss" and
                not self.started.is_zero() and (now - self.started).to_sec() >= 0.8):
            status.gnss_valid = True
            self.last_fix = now
        if not self.last_fix.is_zero():
            status.gnss_fix_type = UgvStatus.GNSS_FIXED
            status.gnss_stamp = self.last_fix
            status.gnss_received_stamp = self.last_fix
        if self.last_result:
            status.last_goal_id, status.last_result_code, status.last_result_stamp = self.last_result
            status.last_result_valid = True
        self.status_pub.publish(status)


if __name__ == "__main__":
    rospy.init_node("ugv_mock")
    mock = Mock()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        mock.tick(rospy.Time.now())
        rate.sleep()
