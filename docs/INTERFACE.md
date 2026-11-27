# BT 与现场客户端接口

ROS1 Noetic。BT 使用新导航仓库的 `slicer_interfaces` 消息与下表公开入口。正式 Action、现场客户端与独立 mock 使用这套消息。接入车端时加载新导航发布的 `install/setup.bash`，并使用与车端相同的 `ROS_MASTER_URI`。

| 名称 | 类型 | 作用 |
| --- | --- | --- |
| `/ugv/navigate` | `slicer_interfaces/NavigateToPointAction` | 每次执行一个目标。首个 WGS84 目标可进入配准准备。 |
| `/ugv/status` | `slicer_interfaces/UgvStatus` | 约 10 Hz。每条观测使用其原始时间和有效性，终态与当前任务分别保存。 |
| `/ugv/control/hold`、`/ugv/control/release` | `slicer_interfaces/Control` | 空请求，返回 `accepted`、`reason`。Hold 抑制自主及任务运动。Release 解除 hold，原任务需重发，急停独立处理。 |
| `/ugv/cmd_vel/mission` | `geometry_msgs/TwistStamped` | 同机 ROS 时钟。位置系 `base_link`，`linear.x` m/s、`angular.z` rad/s。持续发送时戳递增的新指令，0.25 秒过期。可用任务源接管时撤销旧导航权限。 |
| `/ugv/actuators/servo` | `slicer_interfaces/Servo` | C30D 通道 1，脉宽 500–2500 μs。应答只确认协议回执。 |

## 目标和状态

- 如果客户端先调用 Release，应等待新鲜的 `/ugv/status` 显示 `hold=false`、`base_online=true` 后再发目标。服务响应与状态广播异步。

- `LOCAL_POSE=1` 使用 `local_target`，只接受 `map`，位置单位 m。时间戳填零表示使用固定会话参考系中的指定点，也可填近 5 秒内的 ROS 时间。指定 `has_yaw` 时从四元数读航向。
- `WGS84=2` 使用十进制度经纬度。椭球高单位 m，省略时采用会话参考平面。可选 `yaw_enu_rad` 为正东零、逆时针正。在 `has_yaw=false` 时不强加绝对终点航向。地理目标表示车体 `base_link` 原点的地面停车位置。
- `xy_tolerance_m`、`yaw_tolerance_rad`、`timeout` 填零使用服务端默认值（0.20 m、0.30 rad、120 s）。WGS84 设 `require_geographic_verification` 后要求独立的本次 GNSS 到点复核。
- `gnss_stamp` 是接收机适配节点为该 NMEA 观测记录的主机 ROS 接收时间，`gnss_received_stamp` 是导航侧收到该观测的 ROS 时间。原始 GGA/GST 的 UTC 历元用于同历元筛选，当前串口源无法给出已校正的卫星测量延迟。
- Feedback 带 GoalID 与 `PREPARING`、`PLANNING`、`NAVIGATING`、`VERIFYING`、`STOPPING`。剩余距离、速度、投影目标各有有效位。投影携带 `transform_version`。
- 合法新目标可直接替换当前目标。无效目标只拒绝自身。取消用各自的 GoalHandle。接受新目标后，旧取消与旧结果不会撤销新目标。普通客户端断连没有可靠取消语义。
- 成功包含停稳确认。处理取消时先抑制该目标输出，再在有界时间内确认停车。替换与人工接管释放旧目标输出权，后续来源可接手。结果中的 `stop_commanded`、`stop_valid` 和 `stopped` 分别表示已抑制目标输出、停车观测是否有效及车体是否停稳。误差仅在对应有效位为真时使用，来源见 `error_source`。

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
| 14 `HELD` | 控制锁抑制任务 |

## Python 客户端示例

进入封存目录，加载 `install/setup.bash`。在 mock 或现场确认 Release 状态后执行下面的本地示例，或在设置十进制度的 `TARGET_LATITUDE`、`TARGET_LONGITUDE` 后执行 WGS84 示例。

```bash
source install/setup.bash
/usr/bin/python3 docs/examples/navigate.py local 1.0 0.0
/usr/bin/python3 docs/examples/navigate.py wgs84 "$TARGET_LATITUDE" "$TARGET_LONGITUDE"
```

示例只调用 Action，Ctrl+C 取消本次 GoalHandle 并等待停车结果。终态以 JSON 输出。退出码 `0` 表示到点并实测停稳，`130` 表示取消并实测停稳，`1` 表示其余结果或未收到终态。执行 BT 时按 GoalHandle 关联本节点的反馈和终态，收到 `REPLACED` 后按当前任务处理，收到 `STOP_NOT_CONFIRMED` 时从 `/ugv/status` 核对车辆状态。
