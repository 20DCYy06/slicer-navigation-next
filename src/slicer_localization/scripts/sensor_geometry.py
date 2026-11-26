#!/usr/bin/env python3
"""Publish the calibrated static sensor frames once."""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_matrix

from slicer_localization.sensor_frames import mounting_transforms


def transform(parent, child, translation, rotation):
    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    xyz = msg.transform.translation
    xyz.x, xyz.y, xyz.z = map(float, translation)
    quat = msg.transform.rotation
    quat.x, quat.y, quat.z, quat.w = map(float, rotation)
    return msg


if __name__ == "__main__":
    rospy.init_node("slicer_sensor_geometry")
    laser, imu = mounting_transforms()
    arm = rospy.get_param("~laser_to_arm")
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform([
        transform("base_link", "laser_link", laser[:3, 3], quaternion_from_matrix(laser)),
        transform("base_link", "body", imu[:3, 3], quaternion_from_matrix(imu)),
        transform("laser_link", "arm_base_link", arm["translation"], arm["rotation"]),
    ])
    rospy.spin()
