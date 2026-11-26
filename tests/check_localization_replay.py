#!/usr/bin/env python3
"""Compare a recorded no-GNSS candidate replay with the original field bag.

Replay only /Odometry and /wheel/odom (remap wheel input to
/ugv/internal/wheel_odom) on a dedicated master with /use_sim_time:=true
and slicer_localization/localization.launch start_lio:=false. Record
/lio/odom, /lio/map_reference, /odom, /odometry/global, /tf, /diagnostics.
Run this tool with original.bag and replay.bag after recording.
"""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import math

import numpy as np
import rosbag


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def poses(path, topics):
    by_topic = defaultdict(list)
    authorities = Counter()
    diagnostics = Counter()
    with rosbag.Bag(path) as bag:
        for topic, message, _, connection in bag.read_messages(topics=topics, return_connection_header=True):
            if topic == "/tf":
                for item in message.transforms:
                    frame = (item.header.frame_id, item.child_frame_id)
                    authorities[(frame, connection["callerid"])] += 1
                    t = item.transform
                    by_topic[" -> ".join(frame)].append((item.header.stamp.to_sec(), t.translation.x,
                                                           t.translation.y, yaw(t.rotation), 0., 0.))
            elif topic == "/diagnostics":
                for status in message.status:
                    if status.level != 0:
                        diagnostics[(status.name, status.message)] += 1
            else:
                p, v = message.pose.pose, message.twist.twist
                by_topic[topic].append((message.header.stamp.to_sec(), p.position.x, p.position.y,
                                       yaw(p.orientation), v.linear.x, v.angular.z))
    return {key: np.array(sorted(value)) for key, value in by_topic.items()}, authorities, diagnostics


def wrap(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


def pairs(target, reference, gap=.10):
    times = reference[:, 0]
    a, b = [], []
    for sample in target:
        right = bisect_left(times, sample[0])
        near = [i for i in (right - 1, right) if 0 <= i < len(times)]
        index = min(near, key=lambda i: abs(times[i] - sample[0]))
        if abs(times[index] - sample[0]) <= gap:
            a.append(sample)
            b.append(reference[index])
    return np.array(a), np.array(b)


def summary(label, values):
    print(f"  {label:35s} p50/p95/max {np.percentile(values, [50, 95, 100]).round(5)}")


def compare(label, target, reference):
    a, b = pairs(target, reference)
    print(label, "matched", len(a), "of", len(target))
    stationary = np.abs(b[:, 4]) < .025
    offset = wrap(b[:, 3] - a[:, 3])
    # The old global map uses a separately aligned ENU frame. Fit one constant
    # rigid coordinate change to compare relative path and heading.
    angle = math.atan2(np.mean(np.sin(offset)), np.mean(np.cos(offset)))
    c, s = math.cos(angle), math.sin(angle)
    rotate = np.array([[c, s], [-s, c]])
    translation = np.median(b[:, 1:3] - a[:, 1:3] @ rotate, axis=0)
    print("  rigid offset m/rad:", translation.round(5), round(angle, 5))
    summary("raw x/y difference m", np.linalg.norm(a[:, 1:3] - b[:, 1:3], axis=1))
    summary("raw yaw difference rad", np.abs(offset))
    summary("aligned x/y residual m", np.linalg.norm(a[:, 1:3] @ rotate + translation - b[:, 1:3], axis=1))
    summary("aligned yaw residual rad", np.abs(wrap(offset - angle)))
    summary("absolute vx difference m/s", np.abs(a[:, 4] - b[:, 4]))
    summary("absolute yaw rate difference rad/s", np.abs(a[:, 5] - b[:, 5]))
    if np.any(stationary):
        summary("new abs vx at reference stop", np.abs(a[stationary, 4]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("original", help="field bag with archived local/global odometry")
    parser.add_argument("replay", help="candidate replay bag containing both estimators and TF")
    args = parser.parse_args()
    old, _, _ = poses(args.original, ["/Odometry", "/lio/odom", "/odom", "/global/odom", "/wheel/odom"])
    new, authorities, diagnostics = poses(args.replay, ["/lio/odom", "/lio/map_reference", "/odom",
                                                        "/odometry/global", "/tf", "/diagnostics"])
    for topic, data in new.items():
        print(topic, len(data), "samples", "seconds", round(float(data[-1, 0] - data[0, 0]), 3))
    print("New TF authority:", authorities)
    print("New diagnostics:", diagnostics)
    compare("LIO chassis projection", new["/lio/odom"], old["/lio/odom"])
    compare("new local vs prior local", new["/odom"], old["/odom"])
    compare("new global vs prior GPS-corrected global", new["/odometry/global"], old["/global/odom"])
    compare("new global vs new local", new["/odometry/global"], new["/odom"])
    for name in ("map -> odom", "odom -> base_link"):
        tf = new[name]
        print(name, "stamp gaps and changes", "non-increasing stamps", int(np.sum(np.diff(tf[:, 0]) <= 0)))
        summary("positive dt s", np.diff(tf[:, 0])[np.diff(tf[:, 0]) > 0])
        summary("x/y translation jump m", np.linalg.norm(np.diff(tf[:, 1:3], axis=0), axis=1))
        summary("yaw jump rad", np.abs(wrap(np.diff(tf[:, 3]))))
        if name == "map -> odom":
            summary("distance from initial offset m", np.linalg.norm(tf[:, 1:3] - tf[0, 1:3], axis=1))
            summary("change in yaw offset rad", np.abs(wrap(tf[:, 3] - tf[0, 3])))

    expected = {
        (("odom", "base_link"), b"/slicer_local_ekf"),
        (("map", "odom"), b"/slicer_global_ekf"),
    }
    dynamic_edges = {edge for edge in authorities if edge[0] in {item[0] for item in expected}}
    if dynamic_edges != expected:
        raise SystemExit("Unexpected local/global TF authority")


if __name__ == "__main__":
    main()
