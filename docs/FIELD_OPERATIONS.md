# Slicer 现场操作

## 启动车端与界面

日常从新项目工作区启动。`current` 指向选定的稳定发布。先退出当前运行的车端栈，确保底盘和 GNSS 串口只有一个持有者。确认现场有人可随时关闭实体驱动，再执行以下命令。

```bash
cd /home/ugv/workspace/slicer-navigation-next
./current/run
```

启动后驻车 Hold。默认同时启用本地导航、WGS84、NTRIP 及配准后的实时 RTK 平移融合。
线速度上限为 0.10 m/s，已有完整箱体绕行记录。只用本地坐标时执行 `./current/run outdoor:=false`。
开阔场地可选以下 0.20 m/s 配置。各档的实测工况见[迁移与验证记录](MIGRATION.md)。

```bash
./current/run motion_config:="$PWD/current/install/share/slicer_bringup/config/motion_c30d_v110_speed_020.yaml"
```

户外模式默认使用本车的 UM982 USB 设备和 `~/.config/slicer/{gnss.yaml,ntrip_credentials.json}`。
换车时用 `gnss_serial_device:=<设备路径>`、`gnss_config:=<配置文件>` 和
`ntrip_credentials_file:=<凭据文件>` 指定。串口节点启动时请求 1 Hz GST，供停稳后地理复核，
设置在接收机上随重启清除。接收机已提供 GST 或使用其他型号时可传 `um982_gst:=false`。

在同一桌面另开终端，进入同一工作区后运行 `./current/run ui`。车端和 UI 使用同一个 `ROS_MASTER_URI`，未设置时使用本机 ROS Master。BT 从同一安装环境调用[公开 Action、服务与状态](INTERFACE.md)。

接口版 `interfaces-v1.0.0` 使用 `slicer_interfaces` 1.0.0。切换到这版发布时，从选定发布的同一个安装环境重新构建视觉与 BT，再启动车端和各客户端。用 `rosmsg md5 slicer_interfaces/UgvStatus` 核对各环境的消息指纹。

监督测试其他封存候选时，在其 `releases/<源码提交>/` 目录直接使用 `./run` 和 `./run ui`。

## 发目标、停车与录包

1. 在界面确认底盘在线、定位有效、车已停稳。WGS84 任务再确认 RTK Fixed，并保留「地理到点复核」。
2. 点击「开始录包」和「解除 Hold」，等状态更新。选择「本地 map」输入米制 x、y，或选择
   「WGS84」输入十进制度的纬度与经度。核对「预览目标」后点击「启动导航」。
   首个 WGS84 目标可触发短距自动配准，服务端在反馈里给出投影及变换版本。
3. 停止任务时点击「取消本目标」，观察停车反馈，再点击「驻车 Hold」。手柄 B 键是软件急停。
   需要立刻切断驱动时使用车上实体 Motor 开关。
4. 停稳后点击「结束录包」，等界面显示保存成功。默认目录为 `~/.local/share/slicer/field-data/`，
   每次运行包含 bag、有效运动配置和发布标识。退出界面后，在车端终端用 Ctrl+C 停栈。

## 视觉伺服

采用[视觉仓库](https://github.com/20DCYy06/slicer-visual-grasp)提供的独立安装。导航和视觉可各自启动，在车上联动时使用同一 ROS Master。先运行导航车端，再在另一终端加载导航与视觉安装并启动视觉节点。

```bash
NAV="$HOME/workspace/slicer-navigation-next"
GRASP="$HOME/workspace/slicer-visual-grasp/install"
source "$GRASP/setup.bash"
source "$NAV/current/install/setup.bash" --extend
roslaunch slicer_visual_grasp visual_grasp.launch
```

在桌面终端用 `SLICER_GRASP_INSTALL="$HOME/workspace/slicer-visual-grasp/install" ./current/run ui` 启动界面。导航到点或取消后，等停稳并解除 Hold，再用「启动抓取」、「释放」或「取消本动作」。视觉终态的持物恢复提示由现场人员处理。等底盘停稳、速度源归零后可以继续导航。接口和顺序调用见 [BT 接入说明](INTERFACE.md)。

## 配套与回退

| 组件 | V1.10 配套版本 |
| --- | --- |
| 固件 | 硬件标签 `c30d-fw-v1.10`（`6a09d77`），封存于 `firmware/c30d-v1.10/` |
| 主机驱动 | 硬件库提交 `eeb56f78843655e505bbee9baa4e287b4d2051df` |
| 车端配置 | `install/share/slicer_base/config/c30d_v110.yaml`，车身能力版本 2，SLC2 HELLO 为 60000 CPR，主机里程计为 120000 计数/圈 |

发布目录内的 `release.json` 记录源码提交、硬件库提交和有效配置摘要。可用 `sha256sum firmware/c30d-v1.10/firmware.bin` 对照同目录 `manifest.json`。已配套验收的镜像 SHA-256 为 `f959b58d712e77aa8b617d0d9b56006ac926aac3e8b2c7571b8eba160abdfed0`。

V1.9 回退同时恢复固件和软件发布。

1. 取消目标、驻车 Hold，确认实车停稳并退出新车端栈。
2. 用 `/home/ugv/workspace/slicer/run status` 核对旧入口选中的发布是 `20261125-c30d-front-pwm-v19`。旧发布根目录是 `/home/ugv/workspace/slicer/releases/20261125-c30d-front-pwm-v19`。
3. 由底盘负责人断开驱动动力，使用该旧发布内 `deps/slicer_ugv_hardware/Firmware/v1.9/bin/firmware.bin` 恢复 V1.9，SHA-256 应为 `f152dbdfafbb3bcab569edd77cad48ef010cb832b0bc560f201b4be6c68e82e5`。确认刷写和读回后，通过 `/home/ugv/workspace/slicer/run rtk` 启动旧 RTK 界面，或用 `/home/ugv/workspace/slicer/run no-rtk nav` 启动旧本地导航。

## 开发与排查

- 依赖导入、构建与封存命令见仓库根目录 `README.md`。
- Mock 可用 `tools/build_candidate.sh --mock-only -j2` 构建。在独立 ROS Master 下从新
  `install/setup.bash` 加载环境，再运行 `roslaunch slicer_bringup ugv_mock.launch`。
  同一环境可运行 `rosrun slicer_operator_ui operator_ui.py`。Mock 默认 Hold，
  联调时按[接口说明](INTERFACE.md#配置与调用示例)解除 Hold 或注入故障。
- 静态雷达排查时保持车身停稳，使用独立 ROS Master 启动仅采集定位及感知的入口。
  `roslaunch slicer_bringup ugv.launch start_lio:=true start_perception:=true`。
  输出稳定后用 `rosrun slicer_bringup record_run.py --perception-snapshot` 录制约 8 秒。
  对比两次录包时固定车位与朝向。
