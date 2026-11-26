# 迁移与验证记录

## 车端基线

| 对象 | 读取到的版本 |
| --- | --- |
| 正式选择和回退入口 | `/home/ugv/workspace/slicer/run`，`current → releases/20261125-c30d-front-pwm-v19` |
| 回退发布源码 | `13e0afb1c16531997a922f19a23fe6987a8c05fb`，`run status` 校验成功 |
| 回退发布的硬件库 | `0373548750bb3daf7332936652f2c5bb78cb89c7` |
| 旧工作区检出 | `93ccd8c82fe6af332c37692ec34510baeb0f70c3`，保留未跟踪的 `.vscode/` |
| 新源码参考 | PR14 合并提交 `34e599365da1f8a79b034d447649d0f43049754f`，与 PR 源分支 `693cc6e` 文件树差异为空 |
| 本地现场参考 | `20261126-rtk-b-field-nav-rc2` / `84b8511`，以及 `20261126-rtk-b-grasp-v19-rc3` / `0585da7` |
| 目标硬件库、LIO、驱动与 SDK | 见 [dependencies.repos](../dependencies.repos) |

现场 `site/hardware.yaml`、发布输入及发布 NoRTK 配置的 SHA-256 均为
`d8d2a2e3ebeca597fc97f0bbce7e906c425e315157fe2d910a9786185fce93b1`。
C30D 是 `/dev/c30d → ttyACM2`，115200 baud、60000 CPR，PWM 极性
`[-1, 1, 1, 1]`、编码器极性 `[-1, 1, -1, 1]`。现场上电后设备节点可见。
静态读取时未观察到 ROS 导航进程，未通过串口探测固件或发指令。

## 实施记录

- 独立工程在 `candidate` 分支；接口为独立 Catkin 包。
- 先建立固定 `map` 局部导航及静态验证，随后接入 WGS84。室内在线建图、旧 CLI、总控、空地通信与任务队列未迁移。

- Git 逐文件比较：旧正式发布到 PR14 有 62 个改动文件，包括方案 B 规划、Guard、配准和旧客户端流程。RC2 到 PR14 仅 `dependencies.repos` 硬件 pin 改动。旧发布本地硬件树与公开 pin 的主机协议代码相同，公开 pin 移除了部分旧构件和刷写脚本，并将串口默认改为显式现场配置。
- 独立 Catkin 接口与 mock 完成 install。回环 ROS Master 从无旧 overlay 的终端启动 mock，已执行：无效目标不影响活动目标、合法替换、迟到取消、定向取消停稳、终态时间戳保留、hold/release、舵机无效通道拒绝。
- 新薄客户端与录包工具已安装。后续使用同一份记录追加软件闭环、候选部署和实车证据。

### 常驻底盘

- 从 PR14 RC2 的 V1.9 body-speed profile 迁入串口签名、A/B/C/D 轮序与 65 mm 轮径、0.179 m 轮距、0.13 m 轴距。新驱动独占设备并从同连接转发舵机；20 Hz 速度链与 0.25 s 看门狗保持现场 body profile，OTA/固件资料未纳入运行安装。
- 共用运动参数以 RC2 限幅为参考。规划器和 Guard 均为 0.8 m/s 与 0.7854 rad/s，前进加速度采用 Guard 的 0.4 m/s²，角加速度 1.0 rad/s²，独立制动预算 0.2 m/s²。选择向前为正的对称车宽外框后，待现场以底盘外沿复测足迹。
- 常驻 Mux 初始 Hold，人工、任务微调、导航分权，失效缓存不可恢复。软件急停锁存与 Motor OFF 分开。手柄按既有轴和键映射串到 Mux，采用新鲜轴事件与断连停车。接收器失联时是否仍枚举，以及静置摇杆的事件行为待现场验证。
- 测试 `test_control_ownership.py` 12 项通过。独立 ROS Master 从 install 运行 `rostest slicer_base base.test` 通过，覆盖导航代际切换、手动与任务超时、急停和 release、舵机返回与源时效。MCU 回传现在单独记录原始时钟与接收时戳。上述测试使用合成输入。

- 只读确认当前车端 GNSS 外部配置在 `/home/ugv/.config/slicer/gnss.yaml`，指定 USB 设备 `5185028488` 的 by-id 链接，解析为 `ttyACM1`。C30D、手柄和 GNSS 对现场用户均具设备权限。读取到 NTRIP 凭据文件存在，未打开文件或连接网络接收机。默认 `/dev/gnss` 容器别名在本机不存在。
- Tk 8.6 现场桌面隐藏窗口接入独立回环 mock，完成预览、本地目标发送、取消、状态、录包及实际 bag 文件检查。

### 固定地图的软件闭环

- 新顶层 launch 分别启动局部和全局 EKF，定位层静态安装 TF 只有一个所有者。局部源为 LIO x/y/yaw、轮速 vx；全局使用固定会话 LIO 航向、独立轮速和来自 LIO 的平移观测，由同一全局估计器维持 `map → odom`。GNSS xy 融合由显式候选选项控制。
- 方案 B 迁入 B-spline、Dijkstra 连续性软成本（权重 4）、侧别滞回、初始航向锁与终段减速。先用 60 m 滚动全局栅格和 9 m 局部栅格。旧配准编排、桌面任务执行器与原观察锚点/Guard 反馈互锁未迁入。
- DWA 单因素软件对照发现 10 Hz 下从静止只可采样到 0.04 m/s，小于旧 0.08 m/s 下限。已将采样阈值设为 0.035 m/s 并加启动检查，保留共同加速度和速度硬约束。实车低速起步仍待独立确认。
- 新 install 从回环 ROS Master 运行 `tests/check_local_ros.py`，实际用到原始 LIO 投影、两级 EKF、move_base/插件、Guard、Mux 和仿真反馈。1.2 m 本地替换目标在 0.20 m 容差中成功并停稳，非法新目标及迟到取消不影响正确任务。移动取消及导航节点退出后的独立停车均通过。日志 `field-data/verification/local-synthetic-launch.log`。

### WGS84 准备与复核

- 迁入 PR14 的 GNSS 解析、时效和串口身份检查。新增同接收机会话、同 UTC 历元的 GGA/GST 关联，保留原主机接收时间。传输凭据选择使用外部文件。
- `tests/check_geographic_bag.py` 对同车历史的 `064123` 和 `073422` 运行新适配和配准数学。分别关联 3,578/3,578 与 3,008/3,008 对 Fix/质量消息，合格 Fixed 分别 3,578 与 2,754。候选拟合在旧历史数据开始后约 23.45 s 与 93.09 s 形成。两包都缺同历元 GST 协方差，回放结果为降级，不能据此确认实机地理复核。
- 独立安装的合成 WGS84 运行已覆盖行进中取消准备、保持变换版本 0，随后重新发点、提交版本 1、刷新 costmap、地理投影、导航和停稳后已测协方差的 GNSS 复核。该次使用前一版全局试验参数，融合开关关闭。日志 `field-data/verification/geographic-synthetic-launch.log`。
- 当前 28 项 Python 单元测试通过。候选现场动作属于下一阶段。
