#!/usr/bin/env bash
set -eo pipefail

release=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
control="$release/service/ctl"
if [[ ${1:-} == service ]]; then
    shift
    exec "$control" "$@"
fi
if [[ -x "$control" ]] && "$control" installed; then
    if [[ ${1:-} != ui ]]; then
        if (( $# )); then
            printf '%s\n' 'Managed services use ~/.config/slicer/services/field.env for launch settings.' >&2
            exit 2
        fi
        "$control" start localization
        exec "$control" start navigation
    fi
    "$control" verify-ui
    settings="$HOME/.config/slicer/services/field.env"
    if [[ -f "$settings" ]]; then
        source "$settings"
    fi
    unset ROS_HOSTNAME ROS_IP
    export ROS_MASTER_URI=http://localhost:11311
    if [[ -n ${SLICER_ROS_IP:-} ]]; then
        export ROS_IP="$SLICER_ROS_IP"
    fi
    grasp=${SLICER_GRASP_INSTALL:-$HOME/workspace/slicer-visual-grasp/install}
    if [[ -f "$grasp/setup.bash" ]]; then
        export SLICER_GRASP_INSTALL="$grasp"
    fi
fi
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
    if [[ -x "$control" ]]; then
        export SLICER_SERVICE_RUN="$release/run"
    fi
    exec rosrun slicer_operator_ui operator_ui.py "$@"
fi
exec roslaunch slicer_bringup ugv_field.launch "$@"
