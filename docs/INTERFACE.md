# BT 与现场客户端接口

ROS1 Noetic。导航、视觉与 BT 统一使用 `slicer_interfaces` **1.0.0**（导航仓库标签 `interfaces-v1.0.0`）生成的消息。`UgvStatus` 的 MD5 为 `87dfec19f130b37600b8f255ea260cce`，三个工程须使用同一导航安装重新构建。客户端和服务端使用同一个 `ROS_MASTER_URI`。配套视觉版本为 [slicer-visual-grasp v1.1.0](https://github.com/20DCYy06/slicer-visual-grasp/tree/v1.1.0)。

| 名称 | 类型 | 调用及功能 | 结果与取消 |
| --- | --- | --- | --- |
| `/ugv/navigate` | `slicer_interfaces/NavigateToPointAction` | 发送本地 `map` 或 WGS84 目标。首个 WGS84 目标可触发配准准备。 | `SUCCEEDED`、`code=SUCCESS` 且 `stop_valid`、`stopped` 均为真表示到点停稳。取消本目标并等终态。失败读 `code`、`reason`。 |
| `/ugv/status` | `slicer_interfaces/UgvStatus` | 订阅约 10 Hz 的车辆状态，含地图位姿、WGS84、ENU 航向、速度、GNSS、配准与当前目标。 | 用有效位和对应观测时间判断数据可用性。`last_goal_id`、`last_result_code` 描述最近终态。 |
| `/ugv/control/hold`、`/ugv/control/release` | `slicer_interfaces/Control` | 空请求。Hold 抑制自主与任务运动，Release 解除 Hold。解除后等待新鲜状态显示 `hold=false`、`base_online=true`。 | 读 `accepted` 和 `reason`。旧任务需重发，急停独立处理。 |
| `/ugv/cmd_vel/mission` | `geometry_msgs/TwistStamped` | 视觉等任务节点向常驻 Mux 发布 `base_link` 指令。`linear.x` 为 m/s，`angular.z` 为 rad/s，时间戳用本机 ROS 时间并递增。 | Mux 接受任务流时收回旧导航权。停发后指令在 0.25 秒内过期并归还任务速度权。BT 调抓取 Action 时由视觉节点管理该 Topic。 |
| `/ugv/actuators/servo` | `slicer_interfaces/Servo` | 调用 C30D 通道 1，脉宽 500–2500 µs。视觉节点负责抓取流程中的升降。 | 读 `accepted`、`acknowledged` 和 `reason`。应答确认的是协议回执。 |
| `/ugv/grasp` | `slicer_visual_grasp/GraspAction` | 发送 `command=1`（`PICK`）执行视觉对位及抓取，或 `command=2`（`RELEASE`）执行释放及按记录回升。 | `SUCCEEDED` 且 `success=true` 表示完成。失败读 `message`，需恢复或仍持物时读 `needs_release`。取消运行中的本目标，等待 `PREEMPTED` 及新的停稳状态。 |

## 导航目标和状态

- 本地目标 `LOCAL_POSE=1` 使用 `local_target`，参考系为 `map`，单位为米。时间戳可填零或近 5 秒内的 ROS 时间。`has_yaw=true` 时从四元数读航向。
- 地理目标 `WGS84=2` 使用十进制度经纬度，表示 `base_link` 原点的地面停车点。可选椭球高单位为米，可选 `yaw_enu_rad` 从正东起逆时针计。`require_geographic_verification=true` 时做本次停稳 RTK 到点复核。
- `xy_tolerance_m`、`yaw_tolerance_rad`、`timeout` 填零时默认分别为 0.20 m、0.30 rad、120 s。反馈含 GoalID、阶段、剩余距离、速度与目标投影。有有效位时才使用对应数据，投影的 `transform_version` 表示其配准版本。`gnss_stamp` 为接收机适配节点的 ROS 接收时间，`gnss_received_stamp` 为导航侧收到该观测的 ROS 时间。
- 合法新导航目标可替换旧目标。用各自 GoalHandle 取消，旧取消不会撤销新目标。格式无效的目标走 `REJECTED`，格式有效但受控制锁阻止的目标走 `ABORTED` 并返回相应原因。客户端断连时从 `/ugv/status` 核对当前任务和停稳。
- `stop_commanded=true` 表示导航自身发起的目标停止请求已收到确认。外部 Hold、急停、人工或任务速度源接管通过错误码报告，停车有效位使用事件后的实测反馈。`stop_valid=true` 后再看 `stopped`，接管者仍在行驶时可报告有效但未停稳。`REPLACED` 表示目标已交接。误差仅在对应有效位为真时使用。

## 当前地理状态

- `wgs84_latitude_deg`、`wgs84_longitude_deg` 是车体 `base_link` 原点的水平经纬度。`wgs84_valid=true` 时，`wgs84_stamp` 是配对的 Fixed GNSS 观测时间。已提交配准、观测配对与新鲜度满足要求后，按同一时刻航向去除天线水平杆臂。
- `yaw_enu_rad` 以正东为零，逆时针为正，范围为 ±π。`yaw_enu_valid` 和 `yaw_enu_stamp` 对应已配准的 LIO 航向与其源时间。配准之前两组字段均无效。配准后若 GNSS 暂时不可用，LIO 航向仍可能有效。需要天线原始观测时订阅 `/gnss/fix`（`sensor_msgs/NavSatFix`）。

| Code | 含义 |
| --- | --- |
| 0 `SUCCESS` | 满足到点要求并确认停稳 |
| 1 `CANCELED` | 取消且确认停车 |
| 2 `REPLACED` | 被新目标替换 |
| 3 `INVALID_GOAL` | 目标、坐标系、时间戳或参数非法 |
| 4 `LOCALIZATION_UNAVAILABLE` | 定位或地理条件不可用 |
| 5 `ALIGNMENT_SPACE_INSUFFICIENT` | 可验证的配准空间不足 |
| 6 `OUTSIDE_MAP` | 目标在有效地图覆盖外 |
| 7 `NO_PATH` | 规划或执行找不到可行路径 |
| 8 `MANUAL_OVERRIDE` | 人工接管 |
| 9 `ESTOP` | 急停 |
| 10 `BASE_FAULT` | 底盘不可用或故障 |
| 11 `TIMEOUT` | 总期限耗尽 |
| 12 `GEOGRAPHIC_VERIFICATION_FAILED` | 地理到点复核失败 |
| 13 `STOP_NOT_CONFIRMED` | 停车确认超时 |
| 14 `HELD` | Hold 锁定或任务速度源接管导航。结合 `reason` 与最新速度源判断 |

## 抓取与控制权交接

1. 导航到点，或取消导航并等待终态和实测停稳。发抓取目标前确认底盘在线、无人接管、Release 生效且没有活动导航目标。逐个发送抓取或释放目标。
2. 视觉节点通过 `/ugv/cmd_vel/mission` 和 `/ugv/actuators/servo` 接管任务动作。底盘驱动、Mux 和 Guard 在整个过程中保持运行。`PICK` 成功时 `needs_release=true` 表示保留吸附。失败或取消且 `needs_release=true` 时先处理持物或恢复，再决定后续动作。
3. 视觉任务结束时清零并退出任务速度流。继续导航前等待新鲜的 `/ugv/status` 显示 `stop_valid=true`、`stopped=true`、`active_velocity_source=zero`，且 `goal_id`、`nav_goal_id` 为空。收到导航的 `STOP_NOT_CONFIRMED` 或缺失结果时，先核对车辆和任务状态。

现场 UI 从 Action 生命周期、终态 `message` 和 `needs_release` 显示结果。视觉节点日志包含更详细的失败原因。相机预览、识别结果和细分进度待视觉组提供对应 ROS 接口。

## 配置与调用示例

在使用中的导航发布目录加载 `install/setup.bash`。视觉节点在同一 ROS Master 单独启动，在视觉工作区用 `SLICER_NAV_INSTALL=/path/to/navigation/install ./tools/build.sh -j2` 配套构建。加载视觉安装环境，再运行 `roslaunch slicer_visual_grasp visual_grasp.launch`。现场 UI 由 `slicer_operator_ui` 提供，在导航工作区运行 `./current/run ui`。带视觉区时增加 `SLICER_GRASP_INSTALL=/path/to/slicer-visual-grasp/install` 环境变量。

单次导航示例覆盖本地与地理目标。以下经纬度变量需事先设为现场十进制度坐标。

```bash
source install/setup.bash
/usr/bin/python3 docs/examples/navigate.py local 1.0 0.0
/usr/bin/python3 docs/examples/navigate.py wgs84 "$TARGET_LATITUDE" "$TARGET_LONGITUDE"
```

组合示例按顺序执行导航到点、抓取、在同一地点释放、确认底盘空闲停稳及继续导航。现场用事先确认的两组 `map` 坐标。执行需要车端已处于 Release 状态，由一个任务执行流负责。

```bash
source /path/to/slicer-visual-grasp/install/setup.bash
source install/setup.bash --extend
/usr/bin/python3 docs/examples/nav_grasp_nav.py "$PICK_X" "$PICK_Y" "$NEXT_X" "$NEXT_Y"
```

两个示例以 JSON 输出终态，Ctrl+C 仅取消当前调用的 GoalHandle。组合示例在失败、超时或未确认停稳时停止流程，并在视觉终态保留 `needs_release`。

在独立 ROS Master 下用 `roslaunch slicer_bringup ugv_mock.launch` 联调导航。组合示例须加载视觉安装环境，再以 `vision:=true` 启动 Mock。Mock 默认 Hold，运行示例前调用 `rosservice call /ugv/control/release '{}'`。Mock 的地理位置和 ENU 航向有效位保持为假。

Mock 可在运行中设置 `/ugv_mock/fault`，例如 `rosparam set /ugv_mock/fault gnss_loss`。支持 `none`、`hold`、`estop`、`manual_override`、`base_fault`、`gnss_loss`、`path_failure`、`stop_unconfirmed`。用 `none` 清除故障，再按需要显式 Release。该入口仅用于 BT 软件测试。
