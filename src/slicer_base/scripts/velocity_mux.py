#!/usr/bin/env python3
"""Resident ROS command arbiter with a fail-closed costmap and odometry Guard."""

from math import isfinite
from threading import Lock
import time

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rospy
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
import tf2_ros
import yaml

from slicer_interfaces.msg import NavigationVelocity, UgvStatus
from slicer_interfaces.srv import Control, ControlResponse, NavigationAuthority, NavigationAuthorityResponse
from slicer_base.motion_guard import MotionLimits, body_twist_between, swept_clear
from slicer_base.velocity_mux import VelocityMux, ZERO


def yaw_of(quaternion):
    values = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    if not all(isfinite(value) for value in values) or not any(values):
        raise ValueError("invalid quaternion")
    return euler_from_quaternion(values)[2]


def age_valid(received, stamp, now, ros_now, limit):
    return not stamp.is_zero() and 0 <= now - received <= limit and 0 <= (ros_now - stamp).to_sec() <= limit


def is_latched(message):
    return (getattr(message, "_connection_header", {}) or {}).get("latching", "0") != "0"


class Guard:
    def __init__(self, limits):
        self.limits = limits
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.lock = Lock()
        self.grid = None
        self.lio = None
        self.grid_sub = rospy.Subscriber(rospy.get_param("~costmap_topic"), OccupancyGrid, self.costmap, queue_size=1)
        self.lio_sub = rospy.Subscriber(rospy.get_param("~lio_topic"), Odometry, self.odometry, queue_size=1)

    def costmap(self, message):
        info = message.info
        if (is_latched(message) or message.header.stamp.is_zero() or not message.header.frame_id
                or not .001 <= info.resolution <= 1
                or not info.width or not info.height or len(message.data) != info.width * info.height):
            return
        position = info.origin.position
        try:
            origin = position.x, position.y, yaw_of(info.origin.orientation)
            if not all(isfinite(value) for value in origin):
                return
        except ValueError:
            return
        grid = dict(origin=origin, width=info.width, height=info.height, resolution=info.resolution, data=message.data)
        with self.lock:
            if self.grid is None or message.header.stamp > self.grid[1]:
                self.grid = time.monotonic(), message.header.stamp, message.header.frame_id, grid

    def odometry(self, message):
        if is_latched(message) or message.header.stamp.is_zero() or not message.header.frame_id:
            return
        received = time.monotonic()
        stamp = message.header.stamp
        pose = message.pose.pose
        try:
            xy_yaw = pose.position.x, pose.position.y, yaw_of(pose.orientation)
            if not all(isfinite(value) for value in xy_yaw):
                return
        except ValueError:
            return
        with self.lock:
            observed = None
            if self.lio is not None:
                before, previous, frame, previous_pose, _ = self.lio
                dt = (stamp - previous).to_sec()
                if dt <= 0:
                    return
                if 0 < received - before <= .5 and dt <= .5 and frame == message.header.frame_id:
                    observed = body_twist_between(previous_pose, xy_yaw, dt)
                    if (not all(isfinite(value) for value in observed)
                            or abs(observed[0]) > 3 or abs(observed[1]) > 3 or abs(observed[2]) > 6):
                        observed = None
            self.lio = received, stamp, message.header.frame_id, xy_yaw, observed

    def check(self, requested, output, previous, now, ros_now):
        with self.lock:
            grid_sample, lio_sample = self.grid, self.lio
        if (lio_sample is None or lio_sample[-1] is None
                or not age_valid(*lio_sample[:2], now, ros_now, .5)):
            return "Guard requires fresh advancing LIO odometry"
        if grid_sample is None or not age_valid(*grid_sample[:2], now, ros_now, 1.5):
            return "Guard requires a fresh local costmap"
        _, _, frame, grid = grid_sample
        try:
            transform = self.buffer.lookup_transform(frame, "base_link", rospy.Time())
            if not age_valid(now, transform.header.stamp, now, ros_now, .5):
                return "Guard requires fresh base TF"
            translation = transform.transform.translation
            pose = translation.x, translation.y, yaw_of(transform.transform.rotation)
            if not all(isfinite(value) for value in pose):
                raise ValueError("invalid TF")
        except (tf2_ros.TransformException, ValueError):
            return "Guard requires base TF in the local costmap frame"
        candidates = [(linear, 0., angular) for linear, angular in (requested, output, previous)]
        candidates.append(lio_sample[-1])
        for linear, lateral, angular in set(candidates):
            horizon = self.limits.horizon((linear, lateral, angular), .5)
            if not swept_clear(pose, linear, angular, self.limits.footprint, grid, horizon, lateral=lateral):
                return "Guard blocked the swept braking footprint"
        return ""


class MuxNode:
    def __init__(self, config, motion):
        self.period = 1 / config["publish_hz"]
        self.timeout = config["command_timeout_s"]
        self.status_timeout = motion["stop_freshness_s"]
        self.policy = VelocityMux(self.timeout)
        self.limits = MotionLimits(motion)
        self.guard = Guard(self.limits)
        self.lock = Lock()
        self.driver = None
        self.output = ZERO
        self.output_time = time.monotonic()
        self.command_pub = rospy.Publisher("/ugv/internal/body_velocity", TwistStamped, queue_size=1, tcp_nodelay=True)
        self.status_pub = rospy.Publisher("/ugv/internal/base_status", UgvStatus, queue_size=1)
        self.subscriptions = [
            rospy.Subscriber("/ugv/internal/driver_status", UgvStatus, self.driver_status, queue_size=1),
            rospy.Subscriber("/ugv/internal/nav_velocity", NavigationVelocity, self.command, "nav", queue_size=1),
            rospy.Subscriber("/ugv/cmd_vel/mission", TwistStamped, self.command, "mission", queue_size=1),
            rospy.Subscriber("/teleop/cmd_vel", Twist, self.command, "teleop", queue_size=1),
            rospy.Subscriber("/ugv/internal/manual_override", Bool, self.takeover, queue_size=1),
            rospy.Subscriber("/cup_nav/emergency_stop", Bool, self.emergency, queue_size=1),
        ]
        self.services = [
            rospy.Service("/ugv/control/hold", Control, self.hold),
            rospy.Service("/ugv/control/release", Control, self.release),
            rospy.Service("/ugv/internal/nav_authority", NavigationAuthority, self.authority),
        ]

    def set_output(self, velocity, now, stamp):
        self.output, self.output_time = velocity, now
        message = TwistStamped()
        message.header.stamp, message.header.frame_id = stamp, "base_link"
        message.twist.linear.x, message.twist.angular.z = velocity
        self.command_pub.publish(message)

    def driver_status(self, status):
        source = (getattr(status, "_connection_header", {}) or {}).get("callerid")
        if source != "/slicer_base_driver" or is_latched(status) or status.header.stamp.is_zero():
            return
        now, stamp = time.monotonic(), rospy.Time.now()
        with self.lock:
            if self.driver is not None and status.header.stamp <= self.driver[1].header.stamp:
                return
            age = (stamp - status.header.stamp).to_sec()
            if not 0 <= age <= self.status_timeout:
                return
            self.driver = now, status
            previous = self.policy.online
            self.policy.set_online(status.base_online, now, stamp.to_sec())
            if previous and not status.base_online:
                self.set_output(ZERO, now, stamp)

    def command(self, message, source):
        if is_latched(message):
            return
        received, now_ros = time.monotonic(), rospy.Time.now()
        if source == "teleop":
            twist, stamp = message, now_ros
        else:
            if message.header.frame_id not in ("", "base_link"):
                return
            twist, stamp = message.twist, message.header.stamp
        with self.lock:
            old_goal = self.policy.goal_id
            self.policy.command(source, (twist.linear.x, twist.angular.z), received, time.monotonic(),
                                stamp.to_sec(), now_ros.to_sec(), getattr(message, "goal_id", ""))
            if old_goal and not self.policy.goal_id:
                self.set_output(ZERO, received, now_ros)

    def takeover(self, message):
        if not message.data or is_latched(message):
            return
        now, stamp = time.monotonic(), rospy.Time.now()
        with self.lock:
            previous = self.policy.epoch
            self.policy.takeover(now, stamp.to_sec())
            if self.policy.epoch != previous:
                self.set_output(ZERO, now, stamp)

    def emergency(self, message):
        if not message.data:
            return
        now, stamp = time.monotonic(), rospy.Time.now()
        with self.lock:
            self.policy.emergency(True, now, stamp.to_sec())
            self.set_output(ZERO, now, stamp)

    def hold(self, _request):
        with self.lock:
            now, stamp = time.monotonic(), rospy.Time.now()
            response = self.policy.hold(now, stamp.to_sec())
            if not self.policy.manual:
                self.set_output(ZERO, now, stamp)
            return ControlResponse(*response)

    def release(self, _request):
        with self.lock:
            now, stamp = time.monotonic(), rospy.Time.now()
            self.refresh_driver(now, stamp)
            return ControlResponse(*self.policy.release(now, stamp.to_sec()))

    def authority(self, request):
        with self.lock:
            now, stamp = time.monotonic(), rospy.Time.now()
            self.refresh_driver(now, stamp)
            previous = self.policy.goal_id
            result = self.policy.authorize(request.goal_id, request.enable, request.control_epoch, now, stamp.to_sec())
            if result[0] and previous != self.policy.goal_id and previous:
                self.set_output(ZERO, now, stamp)
            return NavigationAuthorityResponse(*result)

    def refresh_driver(self, now, stamp):
        fresh = self.driver is not None and age_valid(
            self.driver[0], self.driver[1].header.stamp, now, stamp, self.status_timeout)
        if not fresh:
            self.policy.set_online(False, now, stamp.to_sec())
        return fresh

    def tick(self):
        now, stamp = time.monotonic(), rospy.Time.now()
        with self.lock:
            self.refresh_driver(now, stamp)
            selection = self.policy.select(now)
            source, requested = selection
            epoch, goal_id = self.policy.epoch, self.policy.goal_id
            previous = self.output
            dt = min(self.period, max(0., now - self.output_time))
        reason = ""
        if requested == ZERO:
            output = ZERO
        else:
            requested = self.limits.clip(requested)
            output = self.limits.output(requested, previous, dt)
            if source != "teleop":
                reason = self.guard.check(requested, output, previous, now, stamp)
                if reason:
                    output = ZERO
        with self.lock:
            now, stamp = time.monotonic(), rospy.Time.now()
            fresh = self.refresh_driver(now, stamp)
            if ((epoch, goal_id) != (self.policy.epoch, self.policy.goal_id)
                    or selection != self.policy.select(now)):
                source, output = "zero", ZERO
            self.set_output(output, now, stamp)
            status = UgvStatus()
            status.header.stamp = stamp
            status.base_online = self.policy.online
            status.estop = self.policy.estop
            status.manual_override = self.policy.manual
            status.hold = self.policy.held
            status.active_velocity_source = source if output != ZERO else "zero"
            status.control_epoch = self.policy.epoch
            status.nav_goal_id = self.policy.goal_id
            if self.driver is not None:
                data = self.driver[1]
                status.velocity = data.velocity
                measurement_fresh = (fresh and not data.velocity.header.stamp.is_zero()
                                     and 0 <= (stamp - data.velocity.header.stamp).to_sec() <= self.status_timeout)
                status.velocity_valid = measurement_fresh and data.velocity_valid
                status.stop_valid = measurement_fresh and data.stop_valid
                status.stopped = status.stop_valid and data.stopped
            if self.policy.estop:
                status.reason = "emergency stop is latched"
            elif not self.policy.online:
                status.reason = self.driver[1].reason if fresh else "base driver status unavailable"
            elif self.policy.manual:
                status.reason = "manual takeover held"
            elif self.policy.held:
                status.reason = "autonomous and mission motion held"
            else:
                status.reason = reason or ("awaiting a fresh command" if source == "zero" else "")
            self.status_pub.publish(status)

    def run(self):
        try:
            while not rospy.is_shutdown():
                started = time.monotonic()
                self.tick()
                time.sleep(max(0., self.period - (time.monotonic() - started)))
        finally:
            self.set_output(ZERO, time.monotonic(), rospy.Time.now())


def main():
    rospy.init_node("slicer_velocity_mux")
    with open(rospy.get_param("~hardware_config"), encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    MuxNode(config, rospy.get_param("/ugv/motion")).run()


if __name__ == "__main__":
    main()
