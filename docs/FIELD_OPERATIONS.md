# Slicer 现场操作

## 独立 mock 与现场客户端

从新工程根目录运行

```bash
tools/build_candidate.sh --mock-only -j2
source install/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:14673 ROS_IP=127.0.0.1
roslaunch slicer_bringup ugv_mock.launch
```

在同一桌面的另一个同配置终端运行 `rosrun slicer_bringup operator_ui.py`。
已验证输入预览、发送本地目标、取消、状态刷新及开始与结束录包。Mock 可用
`scenario:=preparing`、`path_failure`、`gnss_loss`、`manual_override`、
`estop` 和 `stop_unconfirmed` 启动相应场景。Mock 与候选使用各自的 ROS Master。

## 候选构建和部署

`dependencies.repos` 声明四个固定源码版本。在独立工程根目录运行
`vcs import . < dependencies.repos` 获取缺少的依赖，然后运行
`tools/build_candidate.sh -j2`。Noetic 基础环境位于 `/opt/ros/noetic`。
完整构建成功后，可用 `tools/stage_candidate.sh` 复制 install 到新工程
`releases/<完整源码提交>/`。它打印目录，生成 `release.json`，并记录配置摘要。

从待测候选目录运行

```bash
source install/setup.bash
export SLICER_RELEASE_MANIFEST="$PWD/release.json"
export SLICER_RUN_DIR="$PWD/../../field-data"
roslaunch slicer_bringup ugv.launch
```

默认启动用于静态检查，C30D、GNSS 接收机和 MID360 默认不开启。软件回放可用
`driver_mock:=true start_lio:=false start_perception:=false`，通过合成传感器输入闭环。

## 单人监督阶段

1. 现场负责人确认原栈已退出，并在车旁保留可立即关闭驱动的手段。唯一执行流启动候选。
   实机参数入口为 `roslaunch slicer_bringup ugv.launch hardware_enabled:=true`。
   先维持初始 Hold，核对手柄与停稳状态、雷达、局部与全局里程计，以及 `map → odom → base_link`。
2. 得到现场确认后开始录包，再解除 Hold，等状态显示底盘在线且 Hold 已解除，选择 1–2 m 可见空地中的 `map` 目标。
   UI 预览原始坐标后发送。观察车的方向与停止，保留停车观测和独立的地面测量。
   受控试运行可通过 `motion_config:=<绝对路径>` 选择经记录的低速配置，规划器与 Guard 共用该配置。
3. 下一轮录包覆盖行进中替换和取消。取消后按反馈确认停车，驻车用 Hold。
   游戏手柄 B 键会锁存软件急停，硬件 Motor OFF 单独切断驱动能力。两者需用现车逐项核对。
4. 地理试验使用 `enable_geographic:=true enable_gnss_receiver:=true` 接入有效接收机。
   本车接收机通过 `gnss_config:=/home/ugv/.config/slicer/gnss.yaml` 和
   `gnss_serial_device:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5185028488-if00` 指定。
   `enable_gnss_fusion` 独立控制 GNSS xy 纠偏。已配置的 NTRIP 通过 `ntrip:=true` 与
   `ntrip_credentials_file:=/home/ugv/.config/slicer/ntrip_credentials.json` 选择受保护凭据。
   UI 显示定位状态与服务端投影。要求地理复核时，需要停稳后取得同历元不确定度可用的新 RTK Fixed。

UI 普通关闭会尝试取消其目标。先使用取消并观察反馈，再结束现场会话。录包也可通过
`rosrun slicer_bringup record_run.py --output <目录> --manifest <候选的 release.json>` 运行，
Ctrl+C 收尾。录包持续到停车观测完成后，记录现场工况和独立位置测量。

## 回退

取消、Hold，确认输出抑制与实际停稳后，退出候选及其 ROS Master。底盘只允许一个串口持有者。
已有回退入口现场指向 `/home/ugv/workspace/slicer/releases/20261125-c30d-front-pwm-v19`。
使用 `/home/ugv/workspace/slicer/run status` 校验后，按现场原有启动方式恢复该发布。
原发布文件和选择路径留在原位置。软件、回放与实车证据见 [迁移与验证记录](MIGRATION.md)。
