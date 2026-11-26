import numpy as np
import rospy
from tf.transformations import quaternion_matrix


def mounting_transforms():
    laser = rospy.get_param("~base_to_laser")
    base_to_laser = quaternion_matrix(laser["rotation"])
    base_to_laser[:3, 3] = laser["translation"]
    imu_to_laser = np.eye(4)
    imu_to_laser[:3, :3] = np.reshape(rospy.get_param("~fast_lio/mapping/extrinsic_R"), (3, 3))
    imu_to_laser[:3, 3] = rospy.get_param("~fast_lio/mapping/extrinsic_T")
    return base_to_laser, base_to_laser @ np.linalg.inv(imu_to_laser)
