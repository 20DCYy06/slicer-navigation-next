#!/usr/bin/env python3
"""Project FAST-LIO into a fixed planar chassis reference without publishing TF."""
import math

import numpy as np
import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_matrix, quaternion_from_euler, quaternion_matrix

from slicer_localization.sensor_frames import mounting_transforms


class LioProjector:
    def __init__(self):
        _, base_to_imu = mounting_transforms()
        self.imu_to_base = np.linalg.inv(base_to_imu)
        self.variances = rospy.get_param("~lio_pose_variance")
        self.origin = None
        self.last_stamp = rospy.Time()
        self.last_position = None
        self.local = rospy.Publisher("/lio/odom", Odometry, queue_size=5)
        self.map_reference = rospy.Publisher("/lio/map_reference", Odometry, queue_size=5)
        self.motion = rospy.Publisher("/lio/body_motion", TwistWithCovarianceStamped, queue_size=5)
        self.input = rospy.Subscriber("/Odometry", Odometry, self.project, queue_size=5)

    def project(self, msg):
        if (msg.header.frame_id != "camera_init" or msg.child_frame_id != "body"
                or msg.header.stamp <= self.last_stamp):
            return
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        xyz = [position.x, position.y, position.z]
        rotation = [q.x, q.y, q.z, q.w]
        if not np.all(np.isfinite(xyz + rotation)) or not .99 < np.linalg.norm(rotation) < 1.01:
            return
        world_to_imu = quaternion_matrix(rotation)
        world_to_imu[:3, 3] = xyz
        world_to_base = world_to_imu @ self.imu_to_base
        if self.origin is None:
            initial_yaw = euler_from_matrix(world_to_base)[2]
            world_to_origin = quaternion_matrix(quaternion_from_euler(0, 0, initial_yaw))
            world_to_origin[:3, 3] = world_to_base[:3, 3]
            self.origin = np.linalg.inv(world_to_origin)

        dt = (msg.header.stamp - self.last_stamp).to_sec()
        self.last_stamp = msg.header.stamp
        origin_to_base = self.origin @ world_to_base
        yaw = euler_from_matrix(origin_to_base)[2]
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "odom"
        out.child_frame_id = "base_link"
        out.pose.pose.position.x, out.pose.pose.position.y = map(float, origin_to_base[:2, 3])
        orientation = out.pose.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = quaternion_from_euler(0, 0, yaw)
        for index in range(6):
            out.pose.covariance[index * 7] = 1e6
        for index, variance in zip((0, 7, 35), self.variances):
            out.pose.covariance[index] = variance
        self.local.publish(out)
        out.header.frame_id = "map"
        self.map_reference.publish(out)
        position = origin_to_base[:2, 3].copy()
        if self.last_position is not None and .02 <= dt <= .25:
            self.publish_motion(position - self.last_position, yaw, dt, out.header.stamp)
        self.last_position = position

    def publish_motion(self, increment, yaw, dt, stamp):
        # Ending-at-stamp average velocity, resolved in the chassis frame
        # at the end of the interval. Pose variances are propagated as
        # independent endpoint errors before dividing by the squared interval.
        cosine, sine = math.cos(yaw), math.sin(yaw)
        map_to_body = np.array([[cosine, sine], [-sine, cosine]])
        velocity = map_to_body @ increment / dt
        covariance = map_to_body @ np.diag(self.variances[:2]) @ map_to_body.T
        covariance *= 2.0 / dt ** 2
        out = TwistWithCovarianceStamped()
        out.header.stamp = stamp
        out.header.frame_id = "base_link"
        out.twist.twist.linear.x, out.twist.twist.linear.y = map(float, velocity)
        for index in range(6):
            out.twist.covariance[index * 7] = 1e6
        for index, value in zip((0, 1, 6, 7), covariance.flat):
            out.twist.covariance[index] = value
        self.motion.publish(out)


if __name__ == "__main__":
    rospy.init_node("slicer_lio_projector")
    projector = LioProjector()
    rospy.spin()
