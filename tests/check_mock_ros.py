#!/usr/bin/env python3
"""End-to-end public API test on a dedicated mock ROS master."""
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy
from slicer_interfaces.msg import NavigateToPointAction, NavigateToPointGoal, NavigateToPointResult
from slicer_interfaces.msg import UgvStatus
from slicer_interfaces.srv import Control, Servo


def wait_for(predicate, timeout=5.0):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("Timed out waiting for ROS action or status")


def new_goal(frame="map"):
    goal = NavigateToPointGoal(mode=NavigateToPointGoal.LOCAL_POSE)
    goal.local_target.header.frame_id = frame
    goal.local_target.pose.position.x = 1.0
    return goal


def main():
    rospy.init_node("check_mock_ros")
    client = actionlib.ActionClient("/ugv/navigate", NavigateToPointAction)
    assert client.wait_for_server(rospy.Duration(8))
    status = []
    rospy.Subscriber("/ugv/status", UgvStatus, lambda msg: status.append(msg), queue_size=1)
    wait_for(lambda: status)
    first = client.send_goal(new_goal())
    wait_for(lambda: first.get_goal_status() == GoalStatus.ACTIVE)
    bad = client.send_goal(new_goal("old_frame"))
    wait_for(lambda: bad.get_result())
    assert bad.get_result().code == NavigateToPointResult.INVALID_GOAL
    assert first.get_goal_status() == GoalStatus.ACTIVE
    new = client.send_goal(new_goal())
    wait_for(lambda: first.get_result())
    assert first.get_result().code == NavigateToPointResult.REPLACED
    first.cancel()
    wait_for(lambda: new.get_goal_status() == GoalStatus.ACTIVE)
    new.cancel()
    wait_for(lambda: new.get_result())
    result = new.get_result()
    assert result.code == NavigateToPointResult.CANCELED and result.stop_valid and result.stopped
    wait_for(lambda: status[-1].last_result_valid and status[-1].last_result_code == result.code)
    end_stamp = status[-1].last_result_stamp
    assert end_stamp != rospy.Time()
    wait_for(lambda: status[-1].header.stamp > end_stamp)
    assert status[-1].last_result_stamp == end_stamp
    hold = rospy.ServiceProxy("/ugv/control/hold", Control)
    release = rospy.ServiceProxy("/ugv/control/release", Control)
    assert hold().accepted
    wait_for(lambda: status[-1].hold)
    assert release().accepted
    wait_for(lambda: not status[-1].hold and not status[-1].goal_id)
    assert not rospy.ServiceProxy("/ugv/actuators/servo", Servo)(99, 1000).accepted
    print("Mock Action replacement, late cancel, source/terminal stamps, hold/release and typed servo passed")


if __name__ == "__main__":
    main()
