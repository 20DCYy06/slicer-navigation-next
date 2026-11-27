#!/usr/bin/env bash
set -eo pipefail

fail() {
    printf '%s\n' "$1" >&2
    exit 1
}

usage() {
    printf '%s\n' '在要测试的 Git 工作区运行 tools/test_branch.sh {core|mock|sim|ui|exec COMMAND...}' >&2
    exit 2
}

mode=${1:-}
case "$mode" in
    core|mock|sim|ui) [[ $# == 1 ]] || usage ;;
    exec) [[ $# -gt 1 ]] || usage ;;
    *) usage ;;
esac
shift
workspace=$(git rev-parse --show-toplevel 2>/dev/null) || fail '请先进入要测试的导航 Git 工作区。'
cd "$workspace"
[[ -d src/slicer_bringup ]] || fail '当前目录不属于 Slicer 导航工作区。'

port=${SLICER_TEST_ROS_PORT:-15131}
[[ "$port" =~ ^[0-9]{1,5}$ ]] || fail 'SLICER_TEST_ROS_PORT 必须是有效端口。'
port=$((10#$port))
((port >= 1024 && port <= 65535 && port != 11311)) ||
    fail '测试端口请用 1024–65535 范围内的空闲端口，11311 保留给车端。'

unset CMAKE_PREFIX_PATH PKG_CONFIG_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
unset ROS_HOSTNAME ROS_IPV6 ROS_NAMESPACE ROS_LOG_DIR SLICER_SERVICE_RUN SLICER_RELEASE_MANIFEST SLICER_RECORD_RUN
if [[ "$mode" == core ]]; then
    source /opt/ros/noetic/setup.bash
else
    space=${SLICER_TEST_SPACE:-}
    if [[ -z "$space" ]]; then
        space=install
        if [[ -f devel/setup.bash ]]; then
            space=devel
        fi
    fi
    case "$space" in
        install|devel) ;;
        *) fail 'SLICER_TEST_SPACE 只接受 install 或 devel。' ;;
    esac
    prefix="$workspace/$space"
    [[ -f "$prefix/setup.bash" ]] || fail "当前工作区缺少 $space/setup.bash，请先构建要测试的分支。"
    source "$prefix/setup.bash"
fi

require_node() {
    [[ -x "$prefix/lib/$1" ]] || fail "当前 $space 未构建 $1。试界面或 Mock 可用 tools/build_candidate.sh --mock-only -j2，测试完整导航栈需要完整构建。"
}

case "$mode" in
    mock) require_node slicer_bringup/mock_node.py ;;
    sim)
        require_node slicer_base/driver_node.py
        require_node slicer_base/velocity_mux.py
        require_node slicer_localization/lio_projector.py
        require_node slicer_navigation/navigate_server.py
        require_node slicer_bringup/status_node.py
        ;;
    ui) require_node slicer_operator_ui/operator_ui.py ;;
esac

test_data="$workspace/field-data/test-$port"
export ROS_MASTER_URI="http://127.0.0.1:$port" ROS_IP=127.0.0.1
export ROS_HOME="$test_data/ros" SLICER_RUN_DIR="$test_data"
mkdir -p "$ROS_HOME"

case "$mode" in
    core|mock|sim)
        /usr/bin/python3 - "$port" <<'PY'
import socket
import sys

port = int(sys.argv[1])
with socket.socket() as listener:
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        listener.bind(("127.0.0.1", port))
    except OSError as error:
        raise SystemExit(f"测试端口 {port} 不可用。请先结束原测试，或设置 SLICER_TEST_ROS_PORT。{error}")
PY
        ;;
esac

printf '测试 Master %s · 工作区 %s\n' "$ROS_MASTER_URI" "$workspace" >&2
case "$mode" in
    core) exec roscore -p "$port" ;;
    mock) exec roslaunch -p "$port" slicer_bringup ugv_mock.launch ;;
    sim)
        exec roslaunch -p "$port" slicer_bringup ugv.launch hardware_enabled:=false driver_mock:=true \
            gamepad_enabled:=false start_lio:=false start_perception:=false enable_geographic:=true \
            enable_gnss_receiver:=false enable_gnss_fusion:=true ntrip:=false
        ;;
    ui)
        export SLICER_RECORD_RUN="$prefix/lib/slicer_bringup/record_run.py"
        exec "$prefix/lib/slicer_operator_ui/operator_ui.py"
        ;;
    exec) exec "$@" ;;
esac
