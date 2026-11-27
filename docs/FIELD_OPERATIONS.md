# Slicer 现场操作

## 启动车端与界面

日常从新项目工作区操作。`current` 指向选定的稳定发布。迁入常驻服务前先退出此前的手工车端启动，确认旧底盘和 ROS Master 已退出。

```bash
cd /home/ugv/workspace/slicer-navigation-next
tools/install_field_services.sh
./current/run service start base
./current/run service status
```

安装是一次性操作。Jetson 的用户级 systemd 在开机时启动 `master` 和 `base`，底盘先进入 Hold。新装车辆若尚未启用用户驻留，由管理员执行 `sudo loginctl enable-linger "$USER"`。日常按需启动定位、导航及视觉。

```bash
./current/run service start localization
./current/run service start navigation
./current/run ui
```

统一入口是 `./current/run service <status|start|stop|restart|logs> [组件]`。组件为 `master`、`base`、`localization`、`navigation`、`vision`。例如 `./current/run service logs base` 查看底盘日志，`./current/run service stop navigation` 结束车端导航。车端服务和界面使用本机同一个 ROS Master，BT 从同一发布安装调用 [公开接口](INTERFACE.md)。

场地配置放在 `~/.config/slicer/services/field.env`。默认启用户外 WGS84、NTRIP 和配准后的 RTK 平移融合，速度上限为 0.10 m/s。`SLICER_OUTDOOR=false` 只用本地坐标。`SLICER_MOTION_CONFIG` 指向已确认的绝对路径 YAML 可选择实测档位。换车时用 `SLICER_GNSS_SERIAL_DEVICE`、`SLICER_GNSS_CONFIG` 和 `SLICER_NTRIP_CREDENTIALS_FILE` 指定 GNSS 配置。UM982 默认请求 1 Hz GST 用于停稳复核。

接口版 `interfaces-v1.0.0` 使用 `slicer_interfaces` 1.0.0。切换到这版发布时，从选定发布的同一个安装环境重新构建视觉与 BT，再启动车端和各客户端。用 `rosmsg md5 slicer_interfaces/UgvStatus` 核对各环境的消息指纹。

更换发布前用 `./current/run service stop master` 取消活动任务并确认停车。待服务退出后切换 `current`，再用 `./current/run service start base` 启动，按需重新启动其他组件。Master 重启后重新打开界面与 BT 客户端。软件急停跨重启保留。清除急停前先断开实体驱动，停掉 `base` 后由负责人删除 `${XDG_STATE_HOME:-$HOME/.local/state}/slicer-field/estop`，重启底盘会停在 Hold。

## 发目标、停车与录包

1. 在界面确认底盘在线、定位有效、车已停稳。WGS84 任务再确认 RTK Fixed，并保留「地理到点复核」。
2. 点击「开始录包」和「解除 Hold」，等状态更新。选择「本地 map」输入米制 x、y，或选择
   「WGS84」输入十进制度的纬度与经度。核对「预览目标」后点击「启动导航」。
   首个 WGS84 目标可触发短距自动配准，服务端在反馈里给出投影及变换版本。
3. 停止任务时点击「取消本目标」，观察停车反馈，再点击「驻车 Hold」。手柄 B 键是软件急停。
   需要立刻切断驱动时使用车上实体 Motor 开关。
4. 停稳后点击「结束录包」，等界面显示保存成功。默认目录为 `~/.local/share/slicer/field-data/`，
   每次运行包含 bag、有效运动配置和发布标识。停稳后可退出界面，按需停止导航服务。

## 视觉伺服

将[视觉仓库](https://github.com/20DCYy06/slicer-visual-grasp)按同一导航消息版本独立构建。`field.env` 中的 `SLICER_GRASP_INSTALL` 指定其安装路径，缺省为 `~/workspace/slicer-visual-grasp/install`。视觉使用常驻底盘和同一个 Master，可独立启停。

```bash
./current/run service start vision
./current/run ui
```

导航到点或取消后，等停稳并解除 Hold，再用「启动抓取」、「释放」或「取消本动作」。视觉终态的持物恢复提示由现场人员处理。等底盘停稳、速度源归零后可以继续导航。接口和顺序调用见 [BT 接入说明](INTERFACE.md)。

## 配套与回退

| 组件 | V1.10 配套版本 |
| --- | --- |
| 固件 | 硬件标签 `c30d-fw-v1.10`（`6a09d77`），封存于 `firmware/c30d-v1.10/` |
| 主机驱动 | 硬件库提交 `eeb56f78843655e505bbee9baa4e287b4d2051df` |
| 车端配置 | `install/share/slicer_base/config/c30d_v110.yaml`，车身能力版本 2，SLC2 HELLO 为 60000 CPR，主机里程计为 120000 计数/圈 |

发布目录内的 `release.json` 记录源码提交、硬件库提交和有效配置摘要。可用 `sha256sum firmware/c30d-v1.10/firmware.bin` 对照同目录 `manifest.json`。已配套验收的镜像 SHA-256 为 `f959b58d712e77aa8b617d0d9b56006ac926aac3e8b2c7571b8eba160abdfed0`。

V1.9 回退同时恢复固件和软件发布。

1. 用 `./current/run service stop master` 取消目标、驻车 Hold，在确认实车停稳后退出新车端栈。临时切回旧软件期间，用 `systemctl --user disable slicer-field-master.service slicer-field-base.service` 关闭新底盘开机自启。
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
