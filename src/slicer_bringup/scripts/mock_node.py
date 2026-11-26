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


def validate(goal):
    if goal.mode == Goal.LOCAL_POSE:
        pose = goal.local_target
        if pose.header.frame_id != "map":
            return "仅支持 map 坐标系"
        age = (rospy.Time.now() - pose.header.stamp).to_sec()
        if not pose.header.stamp.is_zero() and not 0 <= age <= 0.5:
            return "本地目标时间戳已过期"
        if not all(map(math.isfinite, (pose.pose.position.x, pose.pose.position.y))):
            return "本地坐标无效"
        if goal.has_yaw:
            q = pose.pose.orientation
            if not math.isclose(sum(v*v for v in (q.x, q.y, q.z, q.w)), 1.0, abs_tol=1e-3):
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
    if any(not math.isfinite(value) or value < 0 for value in limits):
        return "容差或期限无效"
    return ""


class Mock:
    def __init__(self):
        self.events = deque()
        self.lock = Lock()
        self.active = None
        self.started = rospy.Time()
        self.last_result = None
        self.hold = False
        self.estop = False
        self.manual = False
        self.control_epoch = 0
        self.mission = None
        self.stop_confirmed = True
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
            self.hold = True
            self.mission = None
            self.control_epoch += 1
            self.events.append(("hold", None))
        return ControlResponse(True, "Mock 已抑制自主和任务输出")

    def on_release(self, _request):
        with self.lock:
            if self.estop or self.manual:
                return ControlResponse(False, "Mock 急停或人工接管仍有效")
            self.mission = None
            self.hold = False
            self.control_epoch += 1
        return ControlResponse(True, "Mock 已解除 hold")

    @staticmethod
    def on_servo(request):
        if request.channel != 1 or not 500 <= request.pulse_us <= 2500:
            return ServoResponse(False, False, "通道或脉宽无效")
        return ServoResponse(True, True, "Mock 模拟收到协议应答")

    def on_mission(self, msg):
        with self.lock:
            if not (self.hold or self.estop or self.manual or self.active or self.events):
                age = (rospy.Time.now() - msg.header.stamp).to_sec()
                if 0.0 <= age <= 0.25:
                    self.mission = msg

    def finish(self, handle, code, reason, now, terminal, stop_commanded=False,
               stop_valid=True, stopped=True, update_last=True):
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
        if update_last:
            self.last_result = (handle.get_goal_id().id, code, now)

    def tick(self, now):
        with self.lock:
            events = list(self.events)
            self.events.clear()
        for event, handle in events:
            if event == "hold":
                if self.active is not None:
                    self.finish(self.active, Result.HELD, "Mock hold", now, "set_aborted", True)
                    self.active = None
                continue
            if event == "cancel":
                if self.active and handle.get_goal_id().id == self.active.get_goal_id().id:
                    confirmed = self.scenario != "stop_unconfirmed"
                    self.finish(
                        self.active, Result.CANCELED if confirmed else Result.STOP_NOT_CONFIRMED,
                        "Mock 已取消" if confirmed else "Mock 无有效停车观测", now,
                        "set_canceled" if confirmed else "set_aborted", True, confirmed, confirmed,
                    )
                    self.stop_confirmed = confirmed
                    self.active = None
                continue

            reason = validate(handle.get_goal())
            with self.lock:
                blocked = self.estop or self.manual or self.hold
            if reason or blocked:
                code = Result.INVALID_GOAL if reason else (
                    Result.ESTOP if self.estop else Result.MANUAL_OVERRIDE if self.manual else Result.HELD
                )
                self.finish(handle, code, reason or "Mock 控制已锁定", now, "set_rejected",
                            stop_valid=False, stopped=False, update_last=False)
                continue
            if self.active:
                self.finish(self.active, Result.REPLACED, "Mock 已替换", now,
                            "set_canceled", stop_valid=False, stopped=False)
            with self.lock:
                self.mission = None
            handle.set_accepted("Mock 已接受")
            self.active = handle
            self.started = now

        phase = Feedback.IDLE
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
            timeout = goal.timeout.to_sec() or 60.0
            ending = None
            if age >= timeout:
                ending = (Result.TIMEOUT, "Mock 任务超时", "set_aborted")
            elif age >= 0.8 and self.scenario == "manual_override":
                with self.lock:
                    self.manual = True
                    self.control_epoch += 1
                ending = (Result.MANUAL_OVERRIDE, "Mock 人工接管", "set_aborted")
            elif age >= 0.8 and self.scenario == "estop":
                with self.lock:
                    self.estop = True
                    self.control_epoch += 1
                ending = (Result.ESTOP, "Mock 急停", "set_aborted")
            elif age >= 0.8 and self.scenario == "path_failure":
                ending = (Result.NO_PATH, "Mock 无可行路径", "set_aborted")
            elif age >= 0.8 and self.scenario == "gnss_loss" and goal.mode == Goal.WGS84:
                ending = (Result.LOCALIZATION_UNAVAILABLE, "Mock GNSS 失效", "set_aborted")
            elif age >= 3.0 and phase != Feedback.PREPARING:
                ending = (Result.SUCCESS, "Mock 已到点并停车", "set_succeeded")
            if ending:
                code, reason, terminal = ending
                handoff = code in (Result.ESTOP, Result.MANUAL_OVERRIDE)
                self.finish(self.active, code, reason, now, terminal, not handoff, not handoff, not handoff)
                self.active = None
                phase = Feedback.IDLE
            else:
                feedback = Feedback(
                    phase=phase, goal_id=self.active.get_goal_id().id, reason="Mock 模拟",
                )
                if goal.mode == Goal.LOCAL_POSE:
                    feedback.projected_target = goal.local_target
                    feedback.projected_target_valid = True
                self.active.publish_feedback(feedback)
        self.publish_status(now, phase)

    def publish_status(self, now, phase):
        status = UgvStatus(base_online=True, stop_valid=self.stop_confirmed, reason="Mock 模拟")
        status.header.stamp = now
        status.pose.header.stamp = now
        status.pose.header.frame_id = "map"
        status.pose.pose.orientation.w = 1.0
        status.pose_valid = True
        with self.lock:
            mission = self.mission
            status.hold = self.hold
            status.manual_override = self.manual
            status.estop = self.estop
            status.control_epoch = self.control_epoch
        moving = mission is not None and 0 <= (now - mission.header.stamp).to_sec() <= 0.25
        status.stopped = status.stop_valid and not moving and self.active is None
        status.velocity_valid = status.stop_valid
        status.velocity.header.stamp = now
        status.velocity.header.frame_id = "base_link"
        if moving:
            status.active_velocity_source = "mission"
            status.velocity = mission
        elif self.active is not None:
            status.goal_id = self.active.get_goal_id().id
            status.nav_goal_id = status.goal_id
            status.phase = phase
            status.active_velocity_source = "navigation"
        if not (self.scenario == "gnss_loss" and
                not self.started.is_zero() and (now - self.started).to_sec() >= 0.8):
            status.gnss_fix_type = UgvStatus.GNSS_FIXED
            status.gnss_valid = True
            status.gnss_stamp = now
            status.gnss_received_stamp = now
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
