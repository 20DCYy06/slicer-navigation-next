#!/usr/bin/env bash
set -eo pipefail

release=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
if [[ ${1:-} == ui && -n ${SLICER_GRASP_INSTALL:-} ]]; then
    source "${SLICER_GRASP_INSTALL%/}/setup.bash"
    source "$release/install/setup.bash" --extend
else
    source "$release/install/setup.bash"
fi
export SLICER_RELEASE_MANIFEST="$release/release.json"
export SLICER_RUN_DIR="${SLICER_RUN_DIR:-$HOME/.local/share/slicer/field-data}"

if [[ ${1:-} == ui ]]; then
    shift
    export SLICER_RECORD_RUN="$release/install/lib/slicer_bringup/record_run.py"
    exec rosrun slicer_operator_ui operator_ui.py "$@"
fi
exec roslaunch slicer_bringup ugv_field.launch "$@"
