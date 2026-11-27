#!/usr/bin/env python3
"""Exercise installed ROS entry points with simulated inputs and no device access."""

import time
import unittest

from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rospy
import rostest
from std_msgs.msg import Bool
import tf2_ros

from slicer_interfaces.msg import NavigationVelocity, UgvStatus
from slicer_interfaces.srv import Control, NavigationAuthority, Servo


class InstalledBaseOwnership(unittest.TestCase):
    def setUp(self):
        self.status = None
        self.status_sub = rospy.Subscriber("/ugv/internal/base_status", UgvStatus, self.update, queue_size=1)
        self.lio = rospy.Publisher("/lio/odom", Odometry, queue_size=1)
        self.grid = rospy.Publisher("/ugv/internal/move_base/local_costmap/costmap", OccupancyGrid, queue_size=1)
        self.nav = rospy.Publisher("/ugv/internal/nav_velocity", NavigationVelocity, queue_size=1)
        self.mission = rospy.Publisher("/ugv/cmd_vel/mission", TwistStamped, queue_size=1)
        self.takeover = rospy.Publisher("/ugv/internal/manual_override", Bool, queue_size=1)
        self.teleop = rospy.Publisher("/teleop/cmd_vel", Twist, queue_size=1)
        self.emergency = rospy.Publisher("/cup_nav/emergency_stop", Bool, queue_size=1, latch=True)
        self.tf = tf2_ros.TransformBroadcaster()

    def update(self, status):
        self.status = status

    def publish_context(self):
        stamp = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, "lio", "base_link"
        odom.pose.pose.orientation.w = 1
        self.lio.publish(odom)
        grid = OccupancyGrid()
        grid.header.stamp, grid.header.frame_id = stamp, "odom"
        grid.info.resolution = .05
        grid.info.width = grid.info.height = 80
        grid.info.origin.position.x = grid.info.origin.position.y = -2
        grid.info.origin.orientation.w = 1
        grid.data = [0] * 6400
        self.grid.publish(grid)
        transform = TransformStamped()
        transform.header = grid.header
        transform.child_frame_id = "base_link"
        transform.transform.rotation.w = 1
        self.tf.sendTransform(transform)

    def until(self, condition, pump=None):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.publish_context()
            if pump:
                pump()
            if self.status and condition(self.status):
                return self.status
            time.sleep(.04)
        self.fail(f"condition timed out with {self.status}")

    def publish_nav(self, goal):
        message = NavigationVelocity(goal_id=goal)
        message.header.stamp, message.header.frame_id = rospy.Time.now(), "base_link"
        message.twist.linear.x = .12
        self.nav.publish(message)

    def publish_mission(self):
        message = TwistStamped()
        message.header.stamp, message.header.frame_id = rospy.Time.now(), "base_link"
        message.twist.linear.x = -.12
        self.mission.publish(message)

    def publish_manual(self):
        self.takeover.publish(Bool(data=True))
        command = Twist()
        command.angular.z = .12
        self.teleop.publish(command)

    def test_online_handoff_hold_watchdog_and_servo(self):
        started = self.until(lambda status: status.base_online and status.stopped)
        self.assertFalse(started.hold)
        self.assertTrue(started.stop_valid)
        self.assertEqual(started.active_velocity_source, "zero")

        release = rospy.ServiceProxy("/ugv/control/release", Control)
        hold = rospy.ServiceProxy("/ugv/control/hold", Control)
        authority = rospy.ServiceProxy("/ugv/internal/nav_authority", NavigationAuthority)
        servo = rospy.ServiceProxy("/ugv/actuators/servo", Servo)
        self.assertTrue(authority("a", True, started.control_epoch).accepted)
        self.until(lambda status: status.active_velocity_source == "nav" and status.velocity.twist.linear.x > .02,
                   lambda: self.publish_nav("a"))

        self.assertTrue(authority("b", True, started.control_epoch).accepted)
        self.assertFalse(authority("a", False, started.control_epoch).accepted)
        self.until(lambda status: status.active_velocity_source == "nav" and status.nav_goal_id == "b",
                   lambda: self.publish_nav("b"))

        handed = self.until(lambda status: status.active_velocity_source == "mission", self.publish_mission)
        self.assertFalse(handed.nav_goal_id)
        self.assertNotEqual(handed.control_epoch, started.control_epoch)
        stopped = self.until(lambda status: status.stopped and status.active_velocity_source == "zero")
        self.assertGreater(stopped.velocity.header.stamp, handed.velocity.header.stamp)

        self.assertTrue(hold().accepted)
        self.assertTrue(release().accepted)
        self.assertFalse(authority("b", True, handed.control_epoch).accepted)
        self.assertEqual(self.until(lambda status: not status.hold).active_velocity_source, "zero")

        self.until(lambda status: status.active_velocity_source == "teleop", self.publish_manual)
        self.assertFalse(release().accepted)
        quiet_manual = self.until(lambda status: status.stopped and status.active_velocity_source == "zero")
        self.assertTrue(quiet_manual.manual_override)
        self.assertTrue(release().accepted)

        forwarded = servo(1, 1800)
        self.assertTrue(forwarded.accepted)
        self.assertFalse(forwarded.acknowledged)
        self.assertFalse(servo(2, 1800).accepted)

        self.emergency.publish(Bool(data=True))
        self.until(lambda status: status.estop)
        self.assertFalse(release().accepted)


if __name__ == "__main__":
    rospy.init_node("installed_base_ownership")
    rostest.rosrun("slicer_base", "installed_base_ownership", InstalledBaseOwnership)
