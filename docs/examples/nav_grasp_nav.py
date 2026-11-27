#!/usr/bin/env python3
"""Navigate to a local pose, pick and release there, then navigate onward."""

import argparse
import json
import time

import actionlib
from actionlib.action_client import CommState
from actionlib_msgs.msg import GoalStatus
import rospy
from slicer_interfaces.msg import (
    NavigateToPointAction, NavigateToPointGoal, NavigateToPointResult, UgvStatus,
)

from navigate import finite


def wait_until(is_ready, timeout):
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if is_ready():
            return True
        time.sleep(.05)
    return False


def wait_for_stopped(latest, after):
    def ready():
        status = latest[0]
        if status is None or status.header.stamp <= after:
            return False
        now = rospy.Time.now()
        return (0 <= (now - status.header.stamp).to_sec() < .45
                and status.velocity_valid and 0 <= (now - status.velocity.header.stamp).to_sec() < .45
                and status.base_online and status.stop_valid and status.stopped
                and not (status.hold or status.estop or status.manual_override)
                and not (status.goal_id or status.nav_goal_id)
                and status.active_velocity_source == "zero")

    if not wait_until(ready, 5.):
        raise RuntimeError("Fresh released, stopped and idle base status is unavailable")


def run_goal(client, goal, label, timeout):
    handle = client.send_goal(goal)

    def finished():
        return handle.get_comm_state() == CommState.DONE

    interrupted = False
    try:
        completed = wait_until(finished, timeout)
    except KeyboardInterrupt:
        interrupted, completed = True, False
    if not completed:
        handle.cancel()
        wait_until(finished, 5.)

    result = handle.get_result() if finished() else None
    status = handle.get_goal_status()
    fields = {"step": label, "action_status": status}
    if isinstance(result, NavigateToPointResult):
        fields.update(code=result.code, reason=result.reason,
                      stop_commanded=result.stop_commanded, stop_valid=result.stop_valid,
                      stopped=result.stopped)
    elif result is not None:
        fields.update(success=result.success, needs_release=result.needs_release, message=result.message)
    else:
        fields["message"] = "Terminal result unavailable. Check /ugv/status and the active task."
    print(json.dumps(fields, ensure_ascii=False), flush=True)

    if interrupted:
        raise KeyboardInterrupt
    if not completed or result is None or status != GoalStatus.SUCCEEDED:
        raise RuntimeError(f"{label} did not succeed")
    return result


def navigate(client, coordinates, label, timeout):
    goal = NavigateToPointGoal(mode=NavigateToPointGoal.LOCAL_POSE,
                               timeout=rospy.Duration.from_sec(timeout))
    goal.local_target.header.frame_id = "map"
    goal.local_target.pose.position.x, goal.local_target.pose.position.y = coordinates
    goal.local_target.pose.orientation.w = 1.
    result = run_goal(client, goal, label, timeout + 5.)
    if result.code != NavigateToPointResult.SUCCESS or not (result.stop_valid and result.stopped):
        raise RuntimeError(f"{label} lacks successful arrival with measured stop")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("arrival_x", type=finite, help="pick-site x in map, metres")
    parser.add_argument("arrival_y", type=finite, help="pick-site y in map, metres")
    parser.add_argument("next_x", type=finite, help="next x in map, metres")
    parser.add_argument("next_y", type=finite, help="next y in map, metres")
    parser.add_argument("--timeout", type=finite, default=90., help="per-action deadline in seconds")
    args = parser.parse_args(rospy.myargv()[1:])
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    try:
        from slicer_visual_grasp.msg import GraspAction, GraspGoal
    except ImportError:
        parser.exit(1, "Load the slicer_visual_grasp install overlay before running this example.\n")

    rospy.init_node("slicer_nav_grasp_example", anonymous=True, disable_signals=True)
    latest = [None]

    def remember(status):
        latest[0] = status

    subscriber = rospy.Subscriber("/ugv/status", UgvStatus, remember, queue_size=1)
    nav = actionlib.ActionClient("/ugv/navigate", NavigateToPointAction)
    grasp = actionlib.ActionClient("/ugv/grasp", GraspAction)
    try:
        for client, name in ((nav, "/ugv/navigate"), (grasp, "/ugv/grasp")):
            if not client.wait_for_server(rospy.Duration(5.)):
                raise RuntimeError(f"{name} unavailable")
        wait_for_stopped(latest, rospy.Time.now())
        navigate(nav, (args.arrival_x, args.arrival_y), "navigate_to_pick", args.timeout)
        wait_for_stopped(latest, rospy.Time.now())
        picked = run_goal(grasp, GraspGoal(command=GraspGoal.PICK), "pick", args.timeout)
        if not picked.success:
            raise RuntimeError("PICK did not succeed")
        wait_for_stopped(latest, rospy.Time.now())
        released = run_goal(grasp, GraspGoal(command=GraspGoal.RELEASE), "release", args.timeout)
        if not released.success or released.needs_release:
            raise RuntimeError("RELEASE did not resolve the grasp state")
        wait_for_stopped(latest, rospy.Time.now())
        navigate(nav, (args.next_x, args.next_y), "continue_navigation", args.timeout)
        return 0
    except KeyboardInterrupt:
        return 130
    except RuntimeError as exc:
        rospy.logerr("%s", exc)
        return 1
    finally:
        subscriber.unregister()


if __name__ == "__main__":
    raise SystemExit(main())
