#!/usr/bin/env bash
set -eo pipefail

release=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
source "$release/install/setup.bash"
export SLICER_RELEASE_MANIFEST="$release/release.json"
export SLICER_RUN_DIR="${SLICER_RUN_DIR:-$HOME/.local/share/slicer/field-data}"

if [[ ${1:-} == ui ]]; then
    shift
    exec rosrun slicer_bringup operator_ui.py "$@"
fi
exec roslaunch slicer_bringup ugv_field.launch "$@"
