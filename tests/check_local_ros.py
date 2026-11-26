#!/usr/bin/env python3
"""No-hardware closed loop through both EKFs, move_base, the Guard and the Mux.

Run ugv.launch with driver_mock:=true and start_lio:=false on a dedicated master.
The plant integrates accepted body commands and supplies synthetic LIO and scan observations.
"""
import math
from threading import Lock
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
import rosnode
import rospy
from sensor_msgs.msg import LaserScan
from slicer_interfaces.msg import NavigateToPointAction, NavigateToPointGoal, NavigateToPointResult, UgvStatus
from slicer_interfaces.srv import Control
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, quaternion_matrix


def wait_for(predicate, timeout=6.0):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(0.025)
    raise AssertionError("Timed out waiting for local navigation condition")


class Plant:
    def __init__(self):
        self.lock = Lock()
        self.x = self.y = self.yaw = 0.0
        self.last_command = None
        self.last_tick = time.monotonic()
        self.motion_count = 0
        self.publish_scan = True
        base_laser = rospy.get_param("/slicer_lio_projector/base_to_laser")
        base_to_laser = quaternion_matrix(base_laser["rotation"])
        base_to_laser[:3, 3] = base_laser["translation"]
        mapping = rospy.get_param("/slicer_lio_projector/fast_lio/mapping")
        imu_to_laser = np.eye(4)
        imu_to_laser[:3, :3] = np.reshape(mapping["extrinsic_R"], (3, 3))
        imu_to_laser[:3, 3] = mapping["extrinsic_T"]
        self.base_to_imu = base_to_laser @ np.linalg.inv(imu_to_laser)

        self.lio = rospy.Publisher("/Odometry", Odometry, queue_size=1)
        self.scan = rospy.Publisher("/scan", LaserScan, queue_size=1)
        rospy.Subscriber("/ugv/internal/body_velocity", TwistStamped, self.on_velocity, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)

    def on_velocity(self, msg):
        with self.lock:
            self.last_command = msg
            if abs(msg.twist.linear.x) + abs(msg.twist.angular.z) > 1e-5:
                self.motion_count += 1

    def tick(self, _event):
        stamp = rospy.Time.now()
        now = time.monotonic()
        with self.lock:
            interval = min(0.10, now - self.last_tick)
            self.last_tick = now
            cmd = self.last_command
            if cmd is not None and 0.0 <= (stamp - cmd.header.stamp).to_sec() < 0.25:
                self.x += math.cos(self.yaw) * cmd.twist.linear.x * interval
                self.y += math.sin(self.yaw) * cmd.twist.linear.x * interval
                self.yaw += cmd.twist.angular.z * interval
            transform = quaternion_matrix(quaternion_from_euler(0, 0, self.yaw))
            transform[:3, 3] = [self.x, self.y, 0.0]
            publish_scan = self.publish_scan

        imu = transform @ self.base_to_imu
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "camera_init"
        odometry.child_frame_id = "body"
        position = odometry.pose.pose.position
        position.x, position.y, position.z = imu[:3, 3]
        rotation = odometry.pose.pose.orientation
        rotation.x, rotation.y, rotation.z, rotation.w = quaternion_from_matrix(imu)
        self.lio.publish(odometry)

        if publish_scan:
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = "base_link"
            scan.angle_min = -math.pi
            scan.angle_increment = 2 * math.pi / 1440
            scan.angle_max = scan.angle_min + scan.angle_increment * 1439
            scan.range_min = 0.03
            scan.range_max = 12.0
            scan.ranges = [float("inf")] * 1440
            self.scan.publish(scan)


def goal(x, y=0.0, frame="map"):
    value = NavigateToPointGoal(mode=NavigateToPointGoal.LOCAL_POSE, timeout=rospy.Duration(18))
    value.local_target.header.frame_id = frame
    value.local_target.pose.position.x = x
    value.local_target.pose.position.y = y
    value.local_target.pose.orientation.w = 1.0
    return value


def main():
    rospy.init_node("slicer_local_loop_check")
    wait_for(lambda: rospy.has_param("/slicer_lio_projector/fast_lio/mapping"))
    if rospy.get_param("/slicer_base_driver/mock", False) is not True:
        raise RuntimeError("The test requires the simulated driver on an isolated ROS Master")
    simulated = Plant()
    client = actionlib.ActionClient("/ugv/navigate", NavigateToPointAction)
    assert client.wait_for_server(rospy.Duration(12)), "public action not online"
    status = []
    rospy.Subscriber("/ugv/status", UgvStatus, lambda data: status.append(data), queue_size=1)
    wait_for(lambda: status and status[-1].pose_valid and status[-1].base_online and status[-1].stopped)
    time.sleep(1.0)
    assert simulated.motion_count == 0, "startup moved before a goal"
    release = rospy.ServiceProxy("/ugv/control/release", Control)
    assert release().accepted, "simulated release rejected"
    wait_for(lambda: not status[-1].hold)

    initial = client.send_goal(goal(1.0))
    wait_for(lambda: initial.get_goal_status() == GoalStatus.ACTIVE or initial.get_result())
    assert initial.get_result() is None, initial.get_result()
    malformed = client.send_goal(goal(1.0, frame="old_map"))
    wait_for(malformed.get_result)
    assert malformed.get_result().code == NavigateToPointResult.INVALID_GOAL
    replacement = client.send_goal(goal(1.2))
    wait_for(initial.get_result)
    assert initial.get_result().code == NavigateToPointResult.REPLACED, initial.get_result()
    initial.cancel()
    wait_for(replacement.get_result, 23)
    result = replacement.get_result()
    assert result.code == NavigateToPointResult.SUCCESS, result
    assert result.stop_valid and result.stopped, result
    assert simulated.motion_count > 0, "only zero commands"
    assert math.hypot(simulated.x - 1.2, simulated.y) < 0.23, (simulated.x, simulated.y)

    count = simulated.motion_count
    canceled = client.send_goal(goal(2.0))
    wait_for(lambda: simulated.motion_count > count or canceled.get_result())
    assert canceled.get_result() is None, canceled.get_result()
    canceled.cancel()
    wait_for(canceled.get_result)
    assert canceled.get_result().code == NavigateToPointResult.CANCELED, canceled.get_result()
    assert canceled.get_result().stop_valid and canceled.get_result().stopped
    count = simulated.motion_count
    interrupted = client.send_goal(goal(2.0))
    wait_for(lambda: simulated.motion_count > count or interrupted.get_result())
    assert interrupted.get_result() is None, interrupted.get_result()
    killed, failures = rosnode.kill_nodes(["/ugv/internal/slicer_navigate"])
    assert not failures and killed
    wait_for(lambda: status[-1].active_velocity_source == "zero", timeout=1.0)
    wait_for(lambda: status[-1].stop_valid and status[-1].stopped, timeout=4.0)
    print(f"Simulated fixed-map local loop, replacement, cancel and navigation-exit stop passed ({simulated.x:.3f} m final)")


if __name__ == "__main__":
    main()
