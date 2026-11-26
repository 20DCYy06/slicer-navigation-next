#!/usr/bin/env python3
"""Record a field run and preserve its candidate identity and effective ROS parameters."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


TOPICS = (
    "/ugv/navigate/goal", "/ugv/navigate/cancel", "/ugv/navigate/feedback",
    "/ugv/navigate/result", "/ugv/navigate/status", "/ugv/status",
    "/ugv/internal/base_status", "/ugv/internal/navigation_status",
    "/ugv/internal/geographic_status", "/ugv/internal/wheel_odom",
    "/odom", "/odometry/global", "/lio/odom", "/tf", "/tf_static",
    "/gnss/fix", "/gnss/fix_quality", "/gnss/nmea", "/ugv/localization/gnss_xy",
    "/ugv/internal/move_base/DWAPlannerROS/command",
    "/ugv/cmd_vel/mission", "/ugv/internal/nav_velocity", "/ugv/internal/move_base/unrouted_cmd_vel",
    "/ugv/internal/body_velocity", "/ugv/internal/mcu_feedback", "/scan",
    "/ugv/internal/move_base/global_costmap/costmap",
    "/ugv/internal/move_base/local_costmap/costmap",
    "/ugv/internal/move_base/plan", "/map",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=os.environ.get("SLICER_RUN_DIR", "field-data"))
    parser.add_argument("--manifest", default=os.environ.get("SLICER_RELEASE_MANIFEST"))
    args = parser.parse_args()
    started = datetime.now(timezone.utc)
    run = Path(args.output).expanduser().resolve() / started.strftime("%Y%m%dT%H%M%S.%fZ")
    run.mkdir(parents=True)
    identity = json.loads(Path(args.manifest).read_text()) if args.manifest else {
        "verification": "development run, no release manifest"
    }
    # Keep this scoped to noncredential motion parameters.
    config = subprocess.run(
        ["rosparam", "get", "/ugv/motion"], capture_output=True, timeout=5, check=False,
    )
    if config.returncode == 0:
        (run / "motion.yaml").write_bytes(config.stdout)
        identity["motion_sha256"] = hashlib.sha256(config.stdout).hexdigest()
    identity["record_started_utc"] = started.isoformat()
    (run / "run.json").write_text(json.dumps(identity, indent=2) + "\n")
    topics = list(TOPICS)
    discovered = subprocess.run(["rostopic", "list"], capture_output=True, text=True, timeout=5)
    topics.extend(topic for topic in discovered.stdout.splitlines()
                  if topic.startswith("/ugv/internal/move_base/") and topic.endswith("/plan"))
    print(run, flush=True)
    os.execvp("rosbag", ["rosbag", "record", "-O", str(run / "navigation.bag"),
                         *dict.fromkeys(topics)])


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, subprocess.TimeoutExpired) as error:
        sys.exit(str(error))
