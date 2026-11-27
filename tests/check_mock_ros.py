#!/usr/bin/env python3
"""End-to-end public API test on a dedicated mock ROS master."""
from pathlib import Path
import subprocess
import sys
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped
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
    assert status[-1].base_online and not status[-1].hold
    assert status[-1].active_velocity_source == "zero"
    hold = rospy.ServiceProxy("/ugv/control/hold", Control)
    release = rospy.ServiceProxy("/ugv/control/release", Control)
    first = client.send_goal(new_goal())
    wait_for(lambda: first.get_goal_status() == GoalStatus.ACTIVE)
    bad = client.send_goal(new_goal("old_frame"))
    wait_for(lambda: bad.get_result())
    assert bad.get_result().code == NavigateToPointResult.INVALID_GOAL
    assert first.get_goal_status() == GoalStatus.ACTIVE
    new = client.send_goal(new_goal())
    wait_for(lambda: first.get_result())
    replaced = first.get_result()
    assert replaced.code == NavigateToPointResult.REPLACED
    assert not (replaced.stop_commanded or replaced.stop_valid or replaced.stopped)
    first.cancel()
    wait_for(lambda: new.get_goal_status() == GoalStatus.ACTIVE)
    new.cancel()
    wait_for(lambda: new.get_result())
    result = new.get_result()
    assert result.code == NavigateToPointResult.CANCELED and result.stop_commanded and result.stop_valid and result.stopped
    wait_for(lambda: status[-1].last_result_valid and status[-1].last_result_code == result.code)
    end_stamp = status[-1].last_result_stamp
    assert end_stamp != rospy.Time()
    wait_for(lambda: status[-1].header.stamp > end_stamp)
    assert status[-1].last_result_stamp == end_stamp
    quiescing = client.send_goal(new_goal())
    wait_for(lambda: status[-1].active_velocity_source == "nav")
    quiesce_script = Path(__file__).resolve().parents[1] / "src/slicer_bringup/scripts/quiesce_node.py"
    subprocess.run([sys.executable, str(quiesce_script), "--no-hold"], check=True, timeout=12)
    wait_for(quiescing.get_result)
    assert quiescing.get_goal_status() == GoalStatus.PREEMPTED
    result = quiescing.get_result()
    assert result.code == NavigateToPointResult.CANCELED and result.stop_valid and result.stopped
    wait_for(lambda: status[-1].active_velocity_source == "zero" and not status[-1].goal_id)
    assert not status[-1].hold

    holding = client.send_goal(new_goal())
    wait_for(lambda: status[-1].active_velocity_source == "nav")
    assert hold().accepted
    wait_for(holding.get_result)
    held_result = holding.get_result()
    assert holding.get_goal_status() == GoalStatus.ABORTED
    assert held_result.code == NavigateToPointResult.HELD and not held_result.stop_commanded
    wait_for(lambda: status[-1].hold and status[-1].stop_valid and status[-1].stopped)
    locked = client.send_goal(new_goal())
    wait_for(locked.get_result)
    locked_result = locked.get_result()
    assert locked.get_goal_status() == GoalStatus.ABORTED
    assert locked_result.code == NavigateToPointResult.HELD and not locked_result.stop_commanded
    assert locked_result.stop_valid and locked_result.stopped
    assert release().accepted
    wait_for(lambda: not status[-1].hold and not status[-1].goal_id)

    mission = rospy.Publisher("/ugv/cmd_vel/mission", TwistStamped, queue_size=1)
    wait_for(lambda: mission.get_num_connections())
    active = client.send_goal(new_goal())
    wait_for(lambda: status[-1].active_velocity_source == "nav")
    command = TwistStamped()
    command.header.frame_id = "base_link"
    command.twist.linear.x = .1

    def publish_until_preempted():
        command.header.stamp = rospy.Time.now()
        mission.publish(command)
        return active.get_result()

    wait_for(publish_until_preempted)
    result = active.get_result()
    assert active.get_goal_status() == GoalStatus.ABORTED
    assert result.code == NavigateToPointResult.HELD and not result.stop_commanded
    assert result.stop_valid and not result.stopped
    wait_for(lambda: status[-1].active_velocity_source == "zero" and not status[-1].goal_id)
    mission.unregister()

    active = client.send_goal(new_goal())
    wait_for(lambda: status[-1].active_velocity_source == "nav")
    try:
        rospy.set_param("/ugv_mock/fault", "estop")
        wait_for(active.get_result)
        result = active.get_result()
        assert active.get_goal_status() == GoalStatus.ABORTED
        assert result.code == NavigateToPointResult.ESTOP and not result.stop_commanded
        assert not release().accepted
    finally:
        rospy.set_param("/ugv_mock/fault", "none")
    wait_for(lambda: not status[-1].estop and status[-1].hold)
    assert release().accepted
    wait_for(lambda: not status[-1].hold)
    assert not rospy.ServiceProxy("/ugv/actuators/servo", Servo)(99, 1000).accepted
    print("Mock Action Hold, mission handoff, injected E-stop, replacement, cancel and typed servo passed")


if __name__ == "__main__":
    main()
