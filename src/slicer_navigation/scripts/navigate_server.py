#!/usr/bin/env python3
"""The single public navigation action and owner of private goal execution."""
from copy import deepcopy
from dataclasses import dataclass
import math
from queue import Empty, SimpleQueue
from time import monotonic, sleep

import actionlib
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose

from slicer_interfaces.msg import NavigateToPointAction, NavigateToPointFeedback as Feedback
from slicer_interfaces.msg import NavigateToPointGoal as Goal, NavigateToPointResult as Result
from slicer_interfaces.msg import NavigationVelocity, UgvStatus
from slicer_interfaces.srv import NavigationAuthority
from slicer_localization.geographic_node import RosGeographicSession
from slicer_localization.geographic_session import GeographicUnavailable

from slicer_navigation.arrival_check import pose_error
from slicer_navigation.goal_ownership import GoalOwnership
from slicer_navigation.move_base_adapter import MOVE_BASE, MoveBaseAdapter, SerialServiceCalls
from slicer_navigation.msg import PlannerVelocity
from slicer_navigation.observed_space import grid_cell, preparation_candidate


@dataclass
class Task:
    handle: object
    accepted: float
    deadline: float
    goal_id: str
    goal: Goal
    phase: int = Feedback.PLANNING
    reason: str = ""
    target: PoseStamped = None
    projected: object = None
    drive_target: PoseStamped = None
    preparation: bool = False
    grant: object = None
    control_epoch: int = None
    config: object = None
    move_sent: bool = False
    velocity_armed: bool = False
    stop_code: int = None
    stop_stamp: object = None
    stop_deadline: float = 0.0
    stop_call: object = None
    external_stop: bool = False
    verify_deadline: float = 0.0
    arrival: object = None


def fresh(stamp, limit, now=None):
    if stamp.is_zero():
        return False
    if now is None:
        now = rospy.Time.now()
    return 0 <= (now - stamp).to_sec() <= limit


def map_pose(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x, pose.pose.position.y = x, y
    q = pose.pose.orientation
    q.x, q.y, q.z, q.w = quaternion_from_euler(0, 0, yaw)
    return pose


class NavigationServer:
    def __init__(self):
        self.base = None
        self.pose = None
        self.lio_stamp = rospy.Time()
        self.grid = None
        self.local_grid = None
        self.scan_stamp = rospy.Time()
        self.last_result = None
        self.last_status_at = 0.0
        self.last_velocity_goal = ""
        self.task = None
        self.ownership = GoalOwnership()
        self.inbox = SimpleQueue()
        self.services = SerialServiceCalls()
        self.last_config = None
        self.last_disable = None
        self.cache_refresh = None
        self.grid_after = rospy.Time()
        self.footprint = rospy.get_param("/ugv/motion/footprint")
        self.stop_timeout = rospy.get_param("/ugv/motion/stop_timeout_s")
        self.stop_freshness = rospy.get_param("/ugv/motion/stop_freshness_s")
        self.verified_stop_reserve_m = (
            rospy.get_param("/ugv/motion/stop_linear_mps") * rospy.get_param("/ugv/motion/stop_window_s")
        )
        self.default_timeout = rospy.get_param("~default_timeout_s", 120.0)
        self.startup_timeout = rospy.get_param("~startup_timeout_s", 3.0)
        self.preparation_timeout = rospy.get_param("~preparation_timeout_s", 60.0)
        self.geo = RosGeographicSession(preparation_timeout_s=self.preparation_timeout)
        self.move_base = MoveBaseAdapter()
        self.transforms = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.transforms)
        self.claim = rospy.ServiceProxy("/ugv/internal/nav_authority", NavigationAuthority)
        self.velocity = rospy.Publisher("/ugv/internal/nav_velocity", NavigationVelocity, queue_size=1)
        self.status = rospy.Publisher("/ugv/internal/navigation_status", UgvStatus, queue_size=1)
        self.subscribers = [
            rospy.Subscriber("/ugv/internal/base_status", UgvStatus, self.on_base, queue_size=1),
            rospy.Subscriber("/odometry/global", Odometry, self.on_pose, queue_size=1),
            rospy.Subscriber("/lio/odom", Odometry, self.on_lio, queue_size=1),
            rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1),
            rospy.Subscriber(MOVE_BASE + "/global_costmap/costmap", OccupancyGrid, self.on_grid, queue_size=1),
            rospy.Subscriber(MOVE_BASE + "/local_costmap/costmap", OccupancyGrid, self.on_local_grid, queue_size=1),
            rospy.Subscriber(MOVE_BASE + "/DWAPlannerROS/command", PlannerVelocity, self.on_velocity,
                             queue_size=1, tcp_nodelay=True),
        ]
        self.server = actionlib.ActionServer("/ugv/navigate", NavigateToPointAction,
                                             self.on_goal, self.on_cancel, auto_start=False)
        self.server.start()

    def on_base(self, msg):
        self.base = msg
        task = self.task
        if task is not None and task.phase != Feedback.STOPPING:
            issue = self.control_failure(msg, task)
            if issue is not None and (issue[0] != Result.BASE_FAULT or task.grant is not None):
                if self.ownership.cancel(task.goal_id):
                    self.inbox.put(("interrupted", (task.goal_id, issue)))

    def on_pose(self, msg):
        if msg.header.frame_id == "map" and msg.child_frame_id == "base_link":
            position = msg.pose.pose.position
            q = msg.pose.pose.orientation
            if (math.isfinite(position.x) and math.isfinite(position.y)
                    and abs(math.hypot(q.x, q.y, q.z, q.w) - 1.0) <= .02):
                self.pose = msg

    def on_lio(self, msg):
        if msg.header.frame_id == "odom" and msg.child_frame_id == "base_link":
            self.lio_stamp = msg.header.stamp

    def on_grid(self, msg):
        if msg.header.frame_id == "map":
            self.grid = msg

    def on_local_grid(self, msg):
        if msg.header.frame_id == "odom":
            self.local_grid = msg

    def on_scan(self, msg):
        if msg.header.frame_id == "base_link":
            self.scan_stamp = msg.header.stamp

    def on_velocity(self, msg):
        if msg.header.frame_id != "base_link" or not fresh(msg.header.stamp, .25):
            return
        if not fresh(self.lio_stamp, .5):
            return
        if not all(math.isfinite(v) for v in (msg.twist.linear.x, msg.twist.angular.z)):
            return

        def publish(goal_id):
            self.velocity.publish(NavigationVelocity(header=msg.header, goal_id=goal_id, twist=msg.twist))
            self.last_velocity_goal = goal_id

        self.ownership.route(msg.header.stamp, publish, msg.target_stamp)

    def validate(self, goal):
        if (not math.isfinite(goal.xy_tolerance_m) or goal.xy_tolerance_m < 0 or
                not math.isfinite(goal.yaw_tolerance_rad) or not 0 <= goal.yaw_tolerance_rad <= math.pi or
                goal.timeout.to_sec() < 0):
            return Result.INVALID_GOAL, "Invalid tolerance or deadline"
        if goal.mode == Goal.WGS84:
            if (not -90 <= goal.latitude_deg <= 90 or not -180 <= goal.longitude_deg <= 180 or
                    (goal.has_altitude and not math.isfinite(goal.altitude_ellipsoid_m)) or
                    (goal.has_yaw and not math.isfinite(goal.yaw_enu_rad))):
                return Result.INVALID_GOAL, "Invalid WGS84 coordinate or ENU yaw"
            altitude = goal.altitude_ellipsoid_m if goal.has_altitude else None
            yaw = goal.yaw_enu_rad if goal.has_yaw else None
            try:
                self.geo.validate_site(goal.latitude_deg, goal.longitude_deg, altitude)
                grid = self.current_grid()
                if grid is None or not self.geo.ready:
                    return None
                target = self.geo.project(goal.latitude_deg, goal.longitude_deg, altitude, yaw)
            except GeographicUnavailable:
                return None
            except ValueError as exc:
                return Result.OUTSIDE_MAP, str(exc)
            if grid_cell(grid, target.x, target.y) is None:
                return Result.OUTSIDE_MAP, "The WGS84 target is outside the current global costmap"
            return None
        if goal.mode != Goal.LOCAL_POSE or goal.require_geographic_verification:
            return Result.INVALID_GOAL, "Select LOCAL_POSE without geographic verification or WGS84"
        pose = goal.local_target
        if pose.header.frame_id != "map":
            return Result.INVALID_GOAL, "Local targets require the map frame"
        if not pose.header.stamp.is_zero() and not fresh(pose.header.stamp, 5.0):
            return Result.INVALID_GOAL, "Local target timestamp is stale or in the future"
        xyz = pose.pose.position
        q = pose.pose.orientation
        if not all(math.isfinite(v) for v in (xyz.x, xyz.y, xyz.z, q.x, q.y, q.z, q.w)) or xyz.z != 0:
            return Result.INVALID_GOAL, "Local target requires finite planar coordinates"
        norm = math.hypot(q.x, q.y, q.z, q.w)
        if goal.has_yaw and (abs(norm - 1.0) > .002 or abs(q.x) > .001 or abs(q.y) > .001):
            return Result.INVALID_GOAL, "Local yaw requires a unit planar quaternion"
        grid = self.current_grid()
        if grid is not None and grid_cell(grid, xyz.x, xyz.y) is None:
            return Result.OUTSIDE_MAP, "Local target is outside the current global costmap"
        return None

    def on_goal(self, handle):
        goal_id, goal = handle.get_goal_id().id, handle.get_goal()
        issue = self.validate(goal)
        if issue:
            result = self.result_for(goal, *issue)
            handle.set_rejected(result, result.reason)
            self.inbox.put(("rejected", (goal_id, result.code, rospy.Time.now())))
            return
        accepted = monotonic()
        deadline = accepted + (goal.timeout.to_sec() or self.default_timeout)
        self.ownership.accept(goal_id, valid=True)
        handle.set_accepted()
        self.inbox.put(("goal", Task(handle, accepted, deadline, goal_id, goal)))

    def on_cancel(self, handle):
        goal_id = handle.get_goal_id().id
        if self.ownership.cancel(goal_id):
            self.inbox.put(("cancel", goal_id))

    def receive_events(self):
        while True:
            try:
                kind, value = self.inbox.get_nowait()
            except Empty:
                return
            if kind == "rejected":
                self.last_result = value
            elif kind == "cancel":
                if self.task is not None and self.task.goal_id == value:
                    self.begin_stop(self.task, Result.CANCELED, "Canceled by the client")
            elif kind == "interrupted":
                goal_id, issue = value
                if self.task is not None and self.task.goal_id == goal_id:
                    self.begin_interruption(self.task, *issue)
            elif kind == "goal":
                if self.task:
                    old = self.task
                    preserve_preparation = old.preparation and value.goal.mode == Goal.WGS84
                    self.release(old)
                    if old.external_stop:
                        self.finish(old, old.stop_code, old.reason, preserve_preparation)
                    elif old.stop_code == Result.CANCELED:
                        code = Result.CANCELED if self.stop_confirmed(old) else Result.STOP_NOT_CONFIRMED
                        self.finish(old, code, old.reason, preserve_preparation)
                    else:
                        self.finish(old, Result.REPLACED, "Replaced by a new goal", preserve_preparation)
                self.task = value

    def control_failure(self, base, task=None):
        if base is None or not fresh(base.header.stamp, .5):
            return Result.BASE_FAULT, "Base feedback unavailable"
        if base.estop:
            return Result.ESTOP, "Emergency stop is active"
        if not base.base_online:
            return Result.BASE_FAULT, "Base feedback unavailable"
        if base.manual_override:
            return Result.MANUAL_OVERRIDE, "Manual control is active"
        if base.hold:
            return Result.HELD, "Autonomous motion is held"
        if base.active_velocity_source == "mission":
            return Result.HELD, "Mission control is active"
        if task and task.control_epoch is not None and task.control_epoch != base.control_epoch:
            return Result.HELD, "Base control authority changed"
        return None

    def current_pose(self):
        pose = self.pose
        if pose is not None and fresh(pose.header.stamp, .5) and fresh(self.lio_stamp, .5):
            return pose
        return None

    def current_grid(self):
        grid = self.grid
        if grid is not None and fresh(grid.header.stamp, 1.5) and grid.header.stamp > self.grid_after:
            return grid
        return None

    def move(self, task, now):
        geo_goal = task.goal.mode == Goal.WGS84
        base = self.base
        if task.phase == Feedback.STOPPING:
            issue = self.control_failure(base, task)
            if task.stop_code == Result.SUCCESS and issue and issue[0] in (
                    Result.ESTOP, Result.MANUAL_OVERRIDE, Result.HELD):
                self.begin_interruption(task, *issue)
                return
            self.confirm_stop(task, now)
            return
        issue = self.control_failure(base, task)
        if issue:
            if issue[0] == Result.BASE_FAULT and now - task.accepted <= self.startup_timeout and task.grant is None:
                return
            self.begin_interruption(task, *issue)
            return
        if now >= task.deadline:
            self.begin_stop(task, Result.TIMEOUT, "Navigation deadline exceeded")
            return
        if task.phase == Feedback.VERIFYING:
            self.verify(task, now)
            return

        if self.cache_refresh is not None:
            if not self.cache_refresh.done:
                return
            refresh, self.cache_refresh = self.cache_refresh, None
            if refresh.error:
                self.begin_stop(task, Result.NO_PATH, "Could not refresh costmaps after alignment")
                return
            self.grid_after = refresh.value

        pose, grid = self.current_pose(), self.current_grid()
        if not pose or grid is None or self.scan_stamp <= self.grid_after or not fresh(self.scan_stamp, .5):
            if task.move_sent or now - task.accepted > self.startup_timeout:
                self.begin_stop(task, Result.LOCALIZATION_UNAVAILABLE, "Fresh localization, lidar scan and costmap are required")
            return
        if geo_goal and not self.geographic_target(task, now):
            return
        if task.target is None and not geo_goal:
            task.target = deepcopy(task.goal.local_target)
            if not task.goal.has_yaw:
                task.target.pose.orientation = pose.pose.pose.orientation

        if task.drive_target is None:
            task.drive_target = task.target
        point = task.drive_target.pose.position
        if grid_cell(grid, point.x, point.y) is None:
            self.begin_stop(task, Result.OUTSIDE_MAP, "Navigation target is outside the current global costmap")
            return
        self.drive(task, now, base)

    def geographic_target(self, task, now):
        if not self.geo.operational:
            self.begin_stop(task, Result.LOCALIZATION_UNAVAILABLE, "Geographic navigation is disabled")
            return False
        if task.projected is not None:
            if self.geo.ready and self.geo.version == task.projected.transform_version:
                return True
            self.begin_stop(task, Result.LOCALIZATION_UNAVAILABLE, "Fresh RTK or the accepted alignment was lost")
            return False
        if self.geo.version and not self.geo.ready:
            self.begin_stop(task, Result.LOCALIZATION_UNAVAILABLE, "The current alignment needs fresh RTK observations")
            return False
        if self.geo.ready:
            goal = task.goal
            try:
                task.projected = self.geo.project(goal.latitude_deg, goal.longitude_deg,
                    goal.altitude_ellipsoid_m if goal.has_altitude else None,
                    goal.yaw_enu_rad if goal.has_yaw else None)
            except GeographicUnavailable as exc:
                self.begin_stop(task, Result.LOCALIZATION_UNAVAILABLE, str(exc))
                return False
            except ValueError as exc:
                self.begin_stop(task, Result.OUTSIDE_MAP, str(exc))
                return False
            target = task.projected
            task.target = map_pose(target.x, target.y, target.yaw if target.yaw is not None else 0.0)
            task.phase, task.reason = Feedback.PLANNING, ""
            return True
        task.phase, task.reason = Feedback.PREPARING, "Collecting aligned RTK and LIO motion"
        if now - task.accepted > self.preparation_timeout:
            self.begin_stop(task, Result.LOCALIZATION_UNAVAILABLE, "Geographic preparation exceeded its time limit")
            return False
        if not self.geo.can_prepare:
            self.begin_stop(task, Result.LOCALIZATION_UNAVAILABLE, "RTK or synchronized preparation poses unavailable")
            return False
        self.geo.begin_preparation()
        task.preparation = True
        if self.geo.candidate_ready():
            self.begin_stop(task, Result.SUCCESS, "Alignment candidate ready")
            return False
        if task.drive_target is None:
            return self.choose_preparation_target(task, now)
        return True

    def choose_preparation_target(self, task, now):
        grid = self.local_grid
        try:
            if grid is None or not fresh(grid.header.stamp, 1.5) or grid.header.stamp <= self.grid_after:
                raise ValueError("Fresh local costmap unavailable")
            # Query the latest common epoch of map and odom, then use it for
            # both the local footprint check and the target transformation.
            at = self.transforms.lookup_transform("map", "base_link", rospy.Time()).header.stamp
            if not fresh(at, .5):
                raise ValueError("Fresh chassis pose in map and odom unavailable")
            base = self.transforms.lookup_transform("odom", "base_link", at)
            to_map = self.transforms.lookup_transform("map", "odom", at)
        except (tf2_ros.TransformException, ValueError) as exc:
            task.reason = str(exc)
            if now - task.accepted > self.startup_timeout:
                self.begin_stop(task, Result.LOCALIZATION_UNAVAILABLE,
                                "Fresh local costmap and synchronized odom/map TF required")
            return False

        p = base.transform.translation
        pose_in_odom = Pose(position=Point(p.x, p.y, 0.0), orientation=base.transform.rotation)
        # The local footprint can clear the corridor end while the global
        # planner's circular clearance still rejects a stop there.
        candidate = preparation_candidate(grid, pose_in_odom, self.footprint, 2.0, stop_reserve=.25)
        if candidate is None:
            self.begin_stop(task, Result.ALIGNMENT_SPACE_INSUFFICIENT, "No observed 2 m preparation corridor")
            return False
        target_in_odom = map_pose(*candidate)
        target_in_odom.header = base.header
        task.drive_target = do_transform_pose(target_in_odom, to_map)
        return True

    def drive(self, task, now, base):
        if task.move_sent:
            state, reason = self.move_base.state()
            if state == GoalStatus.ACTIVE:
                if not task.velocity_armed:
                    task.velocity_armed = self.ownership.arm(
                        task.goal_id, self.move_base.sent_at, task.drive_target.header.stamp)
                if not task.preparation and self.last_velocity_goal == task.goal_id:
                    task.phase = Feedback.NAVIGATING
            elif state == GoalStatus.SUCCEEDED:
                code = Result.SUCCESS
                if task.preparation and not self.geo.candidate_ready():
                    code, reason = Result.LOCALIZATION_UNAVAILABLE, "Preparation movement yielded insufficient RTK observations"
                self.begin_stop(task, code, reason or "Planner reached the target")
            elif state is not None:
                self.begin_stop(task, Result.NO_PATH, reason or "The private navigation goal failed")
            return

        if not self.move_base.ready():
            if now - task.accepted > self.startup_timeout and not task.preparation:
                self.begin_stop(task, Result.NO_PATH, "move_base is unavailable or still canceling the old goal")
            return
        if task.config is None:
            if self.last_config is not None and not self.last_config.done:
                return
            xy, yaw = self.planner_tolerances(task)
            task.config = self.last_config = self.services.submit(self.move_base.configure, xy, yaw, task.preparation)
        if not task.config.done:
            return
        if task.config.error:
            self.begin_stop(task, Result.NO_PATH, "Could not apply planner tolerances: " + str(task.config.error))
            return
        if task.grant is None:
            if self.last_disable is not None and not self.last_disable.done:
                return
            task.control_epoch = base.control_epoch
            task.grant = self.services.submit(self.claim, task.goal_id, True, task.control_epoch)
            return
        if not task.grant.done:
            return
        if task.grant.error or not task.grant.value.accepted:
            reason = str(task.grant.error) if task.grant.error else task.grant.value.reason
            if not task.grant.error and reason == "mission command stream owns motion":
                self.begin_interruption(task, Result.HELD, reason)
            else:
                self.begin_stop(task, Result.BASE_FAULT, "Base rejected navigation authority: " + reason)
            return
        task.drive_target.header.stamp = rospy.Time.now()
        self.move_base.start(task.drive_target)
        task.move_sent = True

    @staticmethod
    def tolerances(task):
        if task.preparation:
            return .20, math.pi + .001
        xy = task.goal.xy_tolerance_m or .20
        yaw = (task.goal.yaw_tolerance_rad or .30) if task.goal.has_yaw else math.pi + .001
        return xy, yaw

    def planner_tolerances(self, task):
        xy, yaw = self.tolerances(task)
        if not task.preparation and task.goal.mode == Goal.WGS84 and task.goal.require_geographic_verification:
            # Leave room for motion at the stop threshold during the measurement window.
            xy -= min(self.verified_stop_reserve_m, xy * .5)
        return xy, yaw

    def release(self, task):
        self.ownership.disarm(task.goal_id)
        self.move_base.cancel()
        if task.grant is not None and task.stop_call is None:
            task.stop_call = self.last_disable = self.services.submit(self.claim, task.goal_id, False, task.control_epoch)

    def begin_stop(self, task, code, reason):
        if task.external_stop:
            return
        if task.stop_code == code and task.phase == Feedback.STOPPING:
            return
        self.release(task)
        task.stop_code, task.reason, task.phase = code, reason, Feedback.STOPPING
        task.stop_stamp = rospy.Time.now()
        task.stop_deadline = monotonic() + self.stop_timeout

    def begin_interruption(self, task, code, reason):
        if not task.external_stop:
            self.begin_stop(task, code, reason)
            task.external_stop = True

    def observed_stop(self, task):
        base = self.base
        valid = (base is not None and fresh(base.header.stamp, .5) and base.stop_valid
                 and base.velocity_valid and fresh(base.velocity.header.stamp, self.stop_freshness)
                 and task.stop_stamp is not None and base.velocity.header.stamp > task.stop_stamp)
        return valid, bool(valid and base.stopped)

    def stop_confirmed(self, task):
        if not self.observed_stop(task)[1]:
            return False
        if task.stop_call is None:
            return True
        return task.stop_call.done and self.base is not None and self.base.nav_goal_id != task.goal_id

    def confirm_stop(self, task, now):
        if task.external_stop:
            valid, stopped = self.observed_stop(task)
            other_owner = self.base is not None and (
                self.base.manual_override or self.base.active_velocity_source == "mission")
            if (valid and (stopped or other_owner)) or now >= task.stop_deadline:
                self.finish(task, task.stop_code, task.reason)
            return
        if not self.stop_confirmed(task):
            if now < task.stop_deadline:
                return
            code = Result.STOP_NOT_CONFIRMED if task.stop_code in (Result.CANCELED, Result.SUCCESS) else task.stop_code
            self.finish(task, code, task.reason + "; stop could not be confirmed")
            return
        if task.stop_code != Result.SUCCESS:
            self.finish(task, task.stop_code, task.reason)
            return
        if task.preparation:
            self.commit_alignment(task)
            return
        pose = self.current_pose()
        if pose is None:
            self.finish(task, Result.LOCALIZATION_UNAVAILABLE, "Fresh final pose unavailable")
            return
        position_error, yaw_error = pose_error(pose.pose.pose, task.target.pose, task.goal.has_yaw)
        xy, yaw = self.tolerances(task)
        if position_error > xy or (yaw_error is not None and yaw_error > yaw):
            self.finish(task, Result.NO_PATH, "Final stopped pose is outside the requested tolerance")
        elif task.goal.require_geographic_verification:
            task.phase, task.reason = Feedback.VERIFYING, "Waiting for a fresh stopped RTK observation"
            task.verify_deadline = min(task.deadline, now + 5.0)
        else:
            self.finish(task, Result.SUCCESS, "Target reached and measured stop confirmed")

    def commit_alignment(self, task):
        if not self.geo.commit(True):
            self.finish(task, Result.LOCALIZATION_UNAVAILABLE, "No qualified alignment at measured stop")
            return
        self.cache_refresh = self.services.submit(self.move_base.refresh_costmaps, self.geo.version)
        self.grid_after = rospy.Time.now()
        task.accepted = monotonic()
        task.preparation = False
        task.phase, task.reason = Feedback.PLANNING, "Refreshing costmaps after alignment"
        task.drive_target = task.grant = task.config = task.control_epoch = None
        task.stop_call = task.stop_stamp = task.stop_code = None
        task.move_sent = task.velocity_armed = False

    def verify(self, task, now):
        if not self.observed_stop(task)[1]:
            self.begin_stop(task, Result.STOP_NOT_CONFIRMED, "Measured stop became unavailable during verification")
            return
        check = self.geo.verify(task.projected, task.stop_stamp.to_nsec(), self.tolerances(task)[0])
        if check.valid or now >= task.verify_deadline:
            task.arrival = check
            code = Result.SUCCESS if check.arrived else Result.GEOGRAPHIC_VERIFICATION_FAILED
            self.finish(task, code, check.reason)

    @staticmethod
    def result_for(goal, code, reason):
        result = Result(code=code, reason=reason)
        if goal.mode == Goal.WGS84:
            result.latitude_deg = goal.latitude_deg
            result.longitude_deg = goal.longitude_deg
            result.has_altitude = goal.has_altitude
            result.altitude_ellipsoid_m = goal.altitude_ellipsoid_m
        return result

    def finish(self, task, code, reason, preserve_preparation=False):
        self.ownership.cancel(task.goal_id)
        if task.preparation and not preserve_preparation:
            self.geo.cancel_preparation()
        result = self.result_for(task.goal, code, reason)
        if code != Result.REPLACED:
            result.stop_commanded = bool(not task.external_stop and task.stop_call is not None and task.stop_call.done
                                         and task.stop_call.error is None and task.stop_call.value.accepted)
            result.stop_valid, result.stopped = self.observed_stop(task)
        pose = self.current_pose()
        if task.target is not None and pose is not None:
            error, yaw_error = pose_error(pose.pose.pose, task.target.pose, task.goal.has_yaw)
            result.position_error_valid, result.position_error_m = True, error
            result.error_source = "map"
            if yaw_error is not None:
                result.yaw_error_valid, result.yaw_error_rad = True, yaw_error
        if task.arrival and task.arrival.valid:
            result.position_error_valid, result.position_error_m = True, task.arrival.error_m
            result.error_source = "gnss_xy+map_yaw" if result.yaw_error_valid else "gnss_xy"
        if code == Result.SUCCESS:
            task.handle.set_succeeded(result, reason)
        elif code in (Result.CANCELED, Result.REPLACED):
            task.handle.set_canceled(result, reason)
        else:
            task.handle.set_aborted(result, reason)
        self.last_result = (task.goal_id, code, rospy.Time.now())
        self.task = None

    def publish_status(self):
        status = UgvStatus()
        status.header.stamp = rospy.Time.now()
        if self.last_result is not None:
            status.last_goal_id, status.last_result_code, status.last_result_stamp = self.last_result
            status.last_result_valid = True
        task = self.task
        if task:
            status.goal_id, status.phase, status.navigation_reason = task.goal_id, task.phase, task.reason
            feedback = Feedback(phase=task.phase, goal_id=task.goal_id, reason=task.reason)
            base, pose = self.base, self.current_pose()
            if base is not None and base.velocity_valid and fresh(base.velocity.header.stamp, self.stop_freshness):
                feedback.velocity, feedback.velocity_valid = base.velocity, True
            if task.target is not None:
                feedback.projected_target = task.target
                feedback.projected_target_valid = True
                feedback.transform_version = task.projected.transform_version if task.projected else 0
                if pose is not None:
                    feedback.remaining_distance_m = pose_error(pose.pose.pose, task.target.pose, False)[0]
                    feedback.remaining_distance_valid = True
            task.handle.publish_feedback(feedback)
        self.status.publish(status)

    def run(self):
        while not rospy.is_shutdown():
            self.receive_events()
            now = monotonic()
            task = self.task
            if task is not None:
                try:
                    self.move(task, now)
                except (ValueError, RuntimeError) as exc:
                    rospy.logwarn("Navigation goal failed: %s", exc)
                    self.begin_stop(task, Result.LOCALIZATION_UNAVAILABLE, str(exc))
            if now - self.last_status_at >= .1:
                self.publish_status()
                self.last_status_at = now
            sleep(.02)


if __name__ == "__main__":
    rospy.init_node("slicer_navigate")
    NavigationServer().run()
