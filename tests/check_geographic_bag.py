#!/usr/bin/env python3
"""Audit GNSS pairing and geographic fit against archived ROS1 observations."""

import argparse
from collections import Counter
import json
import math

import rosbag

from slicer_localization.alignment import PlanarPose
from slicer_localization.geographic_session import GeographicSession
from slicer_localization.geographic_transform import GeoPoint
from slicer_localization.gnss_adapter import Fix, FixAdapter, FixQuality


def pose(message):
    value = message.pose.pose
    q = value.orientation
    sine = 2.0 * (q.w * q.z + q.x * q.y)
    cosine = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
    return PlanarPose(message.header.stamp.to_nsec(), value.position.x, value.position.y,
                      math.atan2(sine, cosine))


def audit(path, map_topic, lio_topic):
    adapter = FixAdapter()
    session = GeographicSession((0.065, 0.0))
    counters = Counter()
    first_ns = None
    fit_ns = None
    peak_pair_count = 0
    max_receipt_delay_ns = 0
    topics = ["/gnss/fix", "/gnss/fix_quality", "/gnss/nmea", map_topic, lio_topic]
    with rosbag.Bag(path) as bag:
        for topic, message, recorded in bag.read_messages(topics=topics):
            now_ns = recorded.to_nsec()
            if first_ns is None:
                first_ns = now_ns
            if topic == "/gnss/nmea":
                counters["gst_sentences"] += int(message.data[3:6] == "GST")
            elif topic == "/gnss/fix":
                source_ns = message.header.stamp.to_nsec()
                max_receipt_delay_ns = max(max_receipt_delay_ns, now_ns - source_ns)
                try:
                    altitude = message.altitude if math.isfinite(message.altitude) else None
                    point = GeoPoint(message.latitude, message.longitude, altitude)
                except ValueError:
                    point = None
                adapter.add_fix(Fix(source_ns, now_ns, point, message.status.status,
                                    message.header.frame_id, tuple(message.position_covariance),
                                    message.position_covariance_type))
                counters["fix_messages"] += 1
            elif topic == "/gnss/fix_quality":
                for status in message.status:
                    if status.name == "cup_gnss/receiver":
                        values = {pair.key: pair.value for pair in status.values}
                        adapter.add_quality(FixQuality(message.header.stamp.to_nsec(), now_ns, values))
                        counters["receiver_quality_messages"] += 1
                        break
            else:
                try:
                    sample = pose(message)
                except ValueError:
                    counters["bad_pose"] += 1
                    continue
                if topic == map_topic:
                    session.add_map_pose(sample)
                else:
                    session.add_lio_pose(sample)
                if not session.preparing:
                    session.begin_preparation()
            for observation in adapter.poll(now_ns):
                association = "exact_fix_quality_pairs" if observation.quality_matched else "unpaired_fixes"
                counters[association] += 1
                if observation.usable:
                    counters["usable_" + observation.covariance_source] += 1
                else:
                    counters[observation.reason] += 1
                session.observe(observation, now_ns)
            session.process_pending(now_ns)
            peak_pair_count = max(peak_pair_count, len(session.pairs))
            if fit_ns is None and session.candidate_ready(now_ns):
                fit_ns = now_ns
    return {
        "bag": path,
        "archived_map_topic": map_topic,
        "archived_lio_topic": lio_topic,
        "counts": dict(counters),
        "candidate_after_s": None if fit_ns is None else (fit_ns - first_ns) / 1e9,
        "peak_pair_count": peak_pair_count,
        "maximum_bag_receipt_delay_s": max_receipt_delay_ns / 1e9,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--map-topic", default="/global/odom")
    parser.add_argument("--lio-topic", default="/lio/odom")
    args = parser.parse_args()
    for path in args.bags:
        print(json.dumps(audit(path, args.map_topic, args.lio_topic), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
