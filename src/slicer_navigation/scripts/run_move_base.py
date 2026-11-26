#!/usr/bin/env python3
"""Populate planner startup parameters from the single motion configuration."""
import math
import os
import sys

import rosgraph
from rosgraph.names import get_ros_namespace, load_mappings, ns_join
from roslib.packages import find_node


def shared_limits(master):
    limits = master.getParam("/ugv/motion")
    names = {
        "max_vel_x": "max_linear_mps",
        "max_vel_trans": "max_linear_mps",
        "max_vel_theta": "max_angular_rps",
        "acc_lim_x": "linear_acceleration_mps2",
        "acc_lim_trans": "linear_acceleration_mps2",
        "acc_lim_theta": "angular_acceleration_rps2",
        "trans_stopped_vel": "stop_linear_mps",
        "theta_stopped_vel": "stop_angular_rps",
    }
    params = {key: float(limits[value]) for key, value in names.items()}
    for key, value in params.items():
        if not math.isfinite(value) or value <= 0:
            raise ValueError("invalid shared motion limit {}: {}".format(names[key], value))

    footprint = limits["footprint"]
    if (len(footprint) < 3 or any(len(point) != 2 for point in footprint)
            or not all(math.isfinite(value) for point in footprint for value in point)):
        raise ValueError("invalid shared motion footprint")
    return params, footprint


def main():
    prefix = ns_join(get_ros_namespace(), load_mappings(sys.argv).get("__name", "move_base"))
    master = rosgraph.Master(prefix)
    planner, footprint = shared_limits(master)
    planner_prefix = prefix + "/DWAPlannerROS/"
    min_linear_param = planner_prefix + "min_vel_trans"
    if master.hasParam("/ugv/motion/min_navigation_linear_mps"):
        min_linear = float(master.getParam("/ugv/motion/min_navigation_linear_mps"))
        if not math.isfinite(min_linear) or not 0 < min_linear <= planner["max_vel_x"]:
            raise ValueError("invalid shared min_navigation_linear_mps")
        master.setParam(min_linear_param, min_linear)
    if master.getParam(planner_prefix + "min_vel_x") != 0:
        raise ValueError("DWAPlannerROS/min_vel_x must be zero for forward-only navigation")
    if master.getParam(planner_prefix + "use_dwa"):
        frequency_param = planner_prefix + "controller_frequency"
        if not master.hasParam(frequency_param):
            frequency_param = prefix + "/controller_frequency"
        frequency = float(master.getParam(frequency_param))
        if not math.isfinite(frequency) or frequency <= 0:
            raise ValueError("DWA controller_frequency must be positive and finite")
        linear = min(planner["max_vel_x"], planner["acc_lim_x"] / frequency)
        angular = min(planner["max_vel_theta"], planner["acc_lim_theta"] / frequency)
        min_linear = master.getParam(min_linear_param)
        min_angular = master.getParam(planner_prefix + "min_vel_theta")
        if min_linear > linear + 1e-4 and min_angular > angular + 1e-4:
            raise ValueError(
                "DWA cannot sample from rest at {} Hz: min_vel_trans {} > {:.5f} m/s "
                "and min_vel_theta {} > {:.5f} rad/s".format(frequency, min_linear, linear, min_angular, angular)
            )
    limits = {
        **{planner_prefix + key: value for key, value in planner.items()},
        prefix + "/local_costmap/footprint": footprint,
        prefix + "/global_costmap/footprint": footprint,
    }
    for param, value in limits.items():
        if master.hasParam(param) and master.getParam(param) != value:
            raise ValueError("{} conflicts with /ugv/motion; expected {}".format(param, value))
        master.setParam(param, value)
    executable = find_node("move_base", "move_base")[0]
    os.execv(executable, [executable, *sys.argv[1:]])


if __name__ == "__main__":
    main()
