#!/usr/bin/env python3
"""Join owner snapshots while retaining the original observation timestamps."""
from threading import Lock

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from slicer_interfaces.msg import UgvStatus


BASE_FIELDS = (
    "velocity", "velocity_valid", "base_online", "estop", "manual_override",
    "hold", "stop_valid", "stopped", "active_velocity_source", "control_epoch",
    "nav_goal_id", "reason",
)
NAVIGATION_FIELDS = (
    "goal_id", "phase", "last_goal_id", "last_result_code",
    "last_result_valid", "last_result_stamp", "navigation_reason",
)
GEO_FIELDS = (
    "gnss_fix_type", "gnss_valid", "gnss_stamp", "gnss_received_stamp",
    "alignment_state", "transform_version", "geographic_reason",
)


def fresh(stamp, now, limit):
    return not stamp.is_zero() and 0.0 <= (now - stamp).to_sec() <= limit


class StatusNode:
    def __init__(self):
        self._lock = Lock()
        self._sources = {}
        self._publisher = rospy.Publisher("/ugv/status", UgvStatus, queue_size=1)
        for key, topic, msg_type in (
            ("base", "/ugv/internal/base_status", UgvStatus),
            ("navigation", "/ugv/internal/navigation_status", UgvStatus),
            ("geographic", "/ugv/internal/geographic_status", UgvStatus),
            ("odometry", "/odometry/global", Odometry),
        ):
            rospy.Subscriber(topic, msg_type, self._update, key, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(0.1), self._publish)

    def _update(self, msg, key):
        with self._lock:
            self._sources[key] = msg

    def _publish(self, _event):
        with self._lock:
            sources = self._sources.copy()
        now = rospy.Time.now()
        status = UgvStatus()
        status.header.stamp = now
        for key, fields in (
            ("base", BASE_FIELDS), ("navigation", NAVIGATION_FIELDS), ("geographic", GEO_FIELDS),
        ):
            source = sources.get(key)
            if source is not None:
                for field in fields:
                    setattr(status, field, getattr(source, field))

        odometry = sources.get("odometry")
        if odometry is not None:
            status.pose = PoseStamped(header=odometry.header, pose=odometry.pose.pose)
            status.pose_valid = odometry.header.frame_id == "map" and fresh(
                odometry.header.stamp, now, 0.5
            )
        if "base" not in sources or not fresh(sources["base"].header.stamp, now, 0.5):
            status.velocity_valid = status.base_online = status.stop_valid = status.stopped = False
            status.reason = "底盘状态不可用"
        else:
            status.velocity_valid &= fresh(status.velocity.header.stamp, now, 0.5)
        if "navigation" in sources and not fresh(sources["navigation"].header.stamp, now, 0.5):
            status.goal_id = ""
            status.phase = UgvStatus.IDLE
            status.navigation_reason = "导航状态已过期"
        if "geographic" not in sources or not fresh(sources["geographic"].header.stamp, now, 1.5):
            status.gnss_valid = False
            status.alignment_state = UgvStatus.ALIGNMENT_UNAVAILABLE
            status.geographic_reason = "地理状态不可用"
        self._publisher.publish(status)


if __name__ == "__main__":
    rospy.init_node("ugv_status")
    StatusNode()
    rospy.spin()
