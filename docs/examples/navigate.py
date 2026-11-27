#!/usr/bin/env python3
"""Send one public ROS goal and cancel only that goal on Ctrl+C."""

import argparse
import json
import math
import time

import actionlib
from actionlib.action_client import CommState
import rospy
from slicer_interfaces.msg import NavigateToPointAction, NavigateToPointGoal, NavigateToPointResult


def finite(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("expected a finite number")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=finite, default=0., help="server deadline in seconds; zero uses the default")
    modes = parser.add_subparsers(dest="mode", required=True)
    local = modes.add_parser("local", help="absolute x and y in the session map, in metres")
    local.add_argument("x_m", type=finite)
    local.add_argument("y_m", type=finite)
    geographic = modes.add_parser("wgs84", help="decimal degrees with stopped RTK arrival verification")
    geographic.add_argument("latitude_deg", type=finite)
    geographic.add_argument("longitude_deg", type=finite)
    args = parser.parse_args(rospy.myargv()[1:])
    if args.timeout < 0:
        parser.error("--timeout must be non-negative")
    goal = NavigateToPointGoal(timeout=rospy.Duration.from_sec(args.timeout))
    if args.mode == "local":
        goal.mode = goal.LOCAL_POSE
        goal.local_target.header.frame_id = "map"
        goal.local_target.pose.position.x = args.x_m
        goal.local_target.pose.position.y = args.y_m
        goal.local_target.pose.orientation.w = 1.
    else:
        if not (-90 <= args.latitude_deg <= 90 and -180 <= args.longitude_deg <= 180):
            parser.error("latitude or longitude out of range")
        goal.mode = goal.WGS84
        goal.latitude_deg, goal.longitude_deg = args.latitude_deg, args.longitude_deg
        goal.require_geographic_verification = True

    rospy.init_node("slicer_bt_example", anonymous=True, disable_signals=True)
    action = actionlib.ActionClient("/ugv/navigate", NavigateToPointAction)
    if not action.wait_for_server(rospy.Duration(5)):
        rospy.logerr("/ugv/navigate unavailable")
        return 1

    previous_phase = None

    def feedback(_handle, status):
        nonlocal previous_phase
        if status.phase != previous_phase:
            rospy.loginfo("phase=%d version=%d %s", status.phase, status.transform_version, status.reason)
            previous_phase = status.phase

    handle = action.send_goal(goal, feedback_cb=feedback)
    canceled = False
    try:
        while not rospy.is_shutdown() and handle.get_comm_state() != CommState.DONE:
            time.sleep(.05)
    except KeyboardInterrupt:
        canceled = True
        handle.cancel()
        until = time.monotonic() + 5.
        while not rospy.is_shutdown() and handle.get_comm_state() != CommState.DONE and time.monotonic() < until:
            time.sleep(.05)

    result = handle.get_result()
    if result is None:
        rospy.logerr("No terminal result received. Verify the vehicle state through /ugv/status.")
        return 1
    print(json.dumps({
        "code": result.code,
        "reason": result.reason,
        "stop_commanded": result.stop_commanded,
        "stop_valid": result.stop_valid,
        "stopped": result.stopped,
        "position_error_m": result.position_error_m if result.position_error_valid else None,
        "error_source": result.error_source if result.position_error_valid else None,
    }, ensure_ascii=False))
    if canceled and result.code == NavigateToPointResult.CANCELED and result.stop_valid and result.stopped:
        return 130
    return 0 if result.code == NavigateToPointResult.SUCCESS and result.stop_valid and result.stopped else 1


if __name__ == "__main__":
    raise SystemExit(main())
