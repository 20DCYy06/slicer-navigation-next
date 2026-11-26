import math

from tf.transformations import euler_from_quaternion


def yaw_of(orientation):
    return euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]


def pose_error(actual, target, has_yaw):
    x = actual.position.x - target.position.x
    y = actual.position.y - target.position.y
    yaw = None
    if has_yaw:
        yaw = abs(math.remainder(yaw_of(actual.orientation) - yaw_of(target.orientation), math.tau))
    return math.hypot(x, y), yaw
