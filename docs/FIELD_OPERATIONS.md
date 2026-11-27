# Slicer 现场操作

## 启动车端与界面

在 Jetson 上进入交付的 `releases/<源码提交>/` 目录。先退出当前运行的车端栈，确保底盘和
GNSS 串口只有一个持有者。确认现场有人可随时关闭实体驱动，再执行

```bash
./run
```

启动后驻车 Hold。默认同时启用本地导航、WGS84、NTRIP 及配准后的实时 RTK 平移融合。
线速度上限为 0.10 m/s，已有完整箱体绕行记录。只用本地坐标时执行 `./run outdoor:=false`。
开阔场地可选以下 0.20 m/s 配置。各档的实测工况见[迁移与验证记录](MIGRATION.md)。

```bash
./run motion_config:="$PWD/install/share/slicer_bringup/config/motion_c30d_v110_speed_020.yaml"
```

户外模式默认使用本车的 UM982 USB 设备和 `~/.config/slicer/{gnss.yaml,ntrip_credentials.json}`。
换车时用 `gnss_serial_device:=<设备路径>`、`gnss_config:=<配置文件>` 和
`ntrip_credentials_file:=<凭据文件>` 指定。串口节点启动时请求 1 Hz GST，供停稳后地理复核，
设置在接收机上随重启清除。接收机已提供 GST 或使用其他型号时可传 `um982_gst:=false`。

在同一桌面另开终端，进入同一发布目录后运行 `./run ui`。车端和 UI 使用同一个
`ROS_MASTER_URI`，未设置时使用本机 ROS Master。BT 从同一安装环境调用
[公开 Action、服务与状态](INTERFACE.md)。

## 发目标、停车与录包

1. 在界面确认底盘在线、定位有效、车已停稳。WGS84 任务再确认 RTK Fixed，并保留「地理到点复核」。
2. 点击「开始录包」和「解除 Hold」，等状态更新。选择「本地 map」输入米制 x、y，或选择
   「WGS84」输入十进制度的纬度与经度。核对「预览目标」后点击「启动导航」。
   首个 WGS84 目标可触发短距自动配准，服务端在反馈里给出投影及变换版本。
3. 停止任务时点击「取消本目标」，观察停车反馈，再点击「驻车 Hold」。手柄 B 键是软件急停。
   需要立刻切断驱动时使用车上实体 Motor 开关。
4. 停稳后点击「结束录包」，等界面显示保存成功。默认目录为 `~/.local/share/slicer/field-data/`，
   每次运行包含 bag、有效运动配置和发布标识。退出界面后，在车端终端用 Ctrl+C 停栈。

## 配套与回退

本发布配对 C30D V1.10。封存目录中的 `firmware/c30d-v1.10/` 放置经现场验收的镜像和摘要，
来自硬件仓库标签 `c30d-fw-v1.10`（`6a09d77`），主机库固定为 `eeb56f7`。
启动时主机通过 `0x84` 校验车身能力版本 2，车速使用 V1.10 物理单位。

回退前取消、驻车 Hold、确认实际停稳并退出新栈。V1.9 工程保留在
`/home/ugv/workspace/slicer/releases/20261125-c30d-front-pwm-v19`。
回到该发布时配对留存的 V1.9 固件镜像，由底盘负责人执行。用
`/home/ugv/workspace/slicer/run status` 核对发布，然后沿原现场入口启动。

## 开发与排查

- 从新工程根目录运行 `vcs import . < dependencies.repos` 安装固定版本依赖，
  再运行 `tools/build_candidate.sh -j2` 和 `tools/stage_candidate.sh`。封存步骤会打印发布目录。
- Mock 可用 `tools/build_candidate.sh --mock-only -j2` 构建。在独立 ROS Master 下从新
  `install/setup.bash` 加载环境，再运行 `roslaunch slicer_bringup ugv_mock.launch`。
  同一环境可运行 `rosrun slicer_bringup operator_ui.py`。用 `scenario:=` 选择
  `preparing`、`path_failure`、`gnss_loss`、`manual_override`、`estop` 或 `stop_unconfirmed`。
- 静态雷达排查时先断开实体驱动。在独立 ROS Master 启动
  `roslaunch slicer_bringup ugv.launch start_lio:=true start_perception:=true`。
  输出稳定后用 `rosrun slicer_bringup record_run.py --perception-snapshot` 录制约 8 秒。
  对比两次录包时固定车位与朝向。
