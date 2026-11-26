#!/usr/bin/env bash
set -eo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
cd "$root"
unset CMAKE_PREFIX_PATH PKG_CONFIG_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/noetic/setup.bash

if [[ ${1:-} == --mock-only ]]; then
    shift
    exec catkin_make install -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        '-DCATKIN_WHITELIST_PACKAGES=slicer_interfaces;slicer_bringup' "$@"
fi

/usr/bin/python3 - "$root" <<'PY'
import pathlib
import subprocess
import sys

import yaml

root = pathlib.Path(sys.argv[1])
with (root / "dependencies.repos").open() as manifest:
    repos = yaml.safe_load(manifest)["repositories"]
for path, spec in repos.items():
    location = root / path
    if not (location / ".git").exists():
        sys.exit(f"Missing dependency {path}. Run vcs import . < dependencies.repos")
    revision = subprocess.check_output(["git", "-C", str(location), "rev-parse", "HEAD"], text=True).strip()
    if revision != spec["version"]:
        sys.exit(f"Dependency {path} has {revision}, expected {spec['version']}")
    changes = subprocess.check_output(["git", "-C", str(location), "status", "--porcelain"], text=True)
    if changes:
        sys.exit(f"Dependency {path} has local changes")
PY

sdk="$root/sdk/install"
cmake -S deps/Livox-SDK2 -B sdk/build -DCMAKE_BUILD_TYPE=Release "-DCMAKE_INSTALL_PREFIX:PATH=$sdk"
cmake --build sdk/build --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
cmake --install sdk/build
exec catkin_make install -DCATKIN_WHITELIST_PACKAGES= -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1 "-DLIVOX_SDK2_ROOT:PATH=$sdk" \
    "-DSLICER_C30D_HARDWARE_ROOT:PATH=$root/deps/slicer_ugv_hardware" "$@"
