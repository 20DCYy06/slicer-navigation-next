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

### 第一轮候选前的定位收敛

- 真实历史 `064123` 含 178.8 s 原始 LIO 与轮速。全局候选 V1 用 EKF 差分平移，vx 对局部 p95 差异 0.383 m/s。V2 显式 LIO 速度降低至 0.01425 m/s，长时间积分位置仍相差 13.68 m。这两项实验配置不用于现场。
- 首轮候选 V3 默认对两个 EKF 分别输入同一固定会话中的直接 LIO x/y/yaw 和原始轮速 vx。全局 EKF 等待局部 TF 并保持它独占 `map → odom`；地理融合仍默认关闭，真实 GNSS 协方差与全局纠偏须另行验证。
- V2 的合成 GNSS 测试也验证了开启融合后的软件链。转换观测携带实测协方差与原 Fix 时间戳，地理任务准备、导航和复核通过。V3 继续用同一套软件用例。V3 复核另要求 GNSS 水平 1σ 主轴不大于请求容差。
- 现场操作先由用户切换车辆电池，再恢复连接。静态首跑内置 Hold，验证阶段仅观察串口、轮速停止反馈、手柄、雷达和 TF。

### 候选 41a5150 的回放与实车

- `releases/41a5150f1e1332bb61fc2ee4f84eec528f1ca7b5` 已独立落盘。无 GNSS 的 `064123` V3 回放中，两级 EKF 的 x/y、yaw、vx 差异 p95 分别为 0.00487 m、0.00219 rad、0.00066 m/s。以同一车心 odom 点比较连续全局纠偏，步长 p95 0.00742 m，最大 0.07637 m。两个动态 TF 各只有一个发布者，时间戳单调。详见 `field-data/verification/localization-064123-v3.txt`。
- 电池切换后第一次 Motor OFF 静态启动未收到 HELLO，应答超时。MID360、扫描、定位和 TF 正常。按历史记录先做一次只收不发的 3 秒串口监听，收到 75 帧 0x90 和 75 帧 0x93，再保持 Motor OFF 重启同一候选，握手和停稳反馈恢复。原因仍未确定。第二次静态检查的 70 帧底盘指令均为零，MCU 四轮 PWM 为零，body vx=0、wz=-0.003，C30D 与手柄各只由一个现场进程持有。证据目录 `field-data/verification/field-static-retry-41a5150-iTe6WL`。
- 第一次前行请求紧跟 Release 服务响应，导航仍读到上一帧 Hold 而返回 `HELD`，速度输出为零。UI 及 BT 操作说明加入等待新鲜状态确认的规则。
- 用户监督下用同一候选重试一次。本地目标沿初始车头前方 1.000 m，局部栅格全程前方检查通过。Action 返回 `SUCCESS`，定位位移 0.819 m、目标残差 0.1815 m，命令 vx 峰值 0.200 m/s，遥测 vx 峰值 0.213 m/s。结束 Hold，停止观测有效、vx/wz 均为零、四轮 PWM 为零。录包记录 1 个 goal、1 个 result、284 帧 MCU 遥测。有效运动配置摘要为 `914ea68c127fb2a7aa274904caf5030f1ec0313b86ad70607faa4df2655e9282`。证据目录 `field-data/verification/field-local-retry-41a5150-ZgEWrR`。
- 现场继续验证转向、行进中目标替换和取消、避障及 WGS84。WGS84 实机复核依赖实时 Fixed 和同历元误差数据。原正式回退发布仍由 `slicer/run status` 校验。

- 从候选 41a5150 install 复跑本地和 WGS84 两套合成闭环。本地覆盖替换、旧取消、正常取消及导航退出停车。WGS84 打开模拟 GNSS 融合，覆盖准备取消、配准提交、带源时间的融合输入和停稳后复核，模拟最终 x=2.513 m。更新后的 UI 同样在独立 mock 上验证 Hold 禁止发送、状态确认释放后可发送。

- 用户对直行的独立观察为行驶目测无明显异常、自动停稳，未用尺量。定位残差不能替代实际地面距离测量。
- 第二个监督运行复用 41a5150 和同一低速配置，原目标运动中被新目标替换。录包含两个独立 GoalID，旧目标结果 `REPLACED`，迟到旧取消后新目标仍取得实测前进；定向取消新目标返回 `CANCELED`，停车有效。按录包接收时间，最后取消到零输出约 0.018 s，到停稳结果约 1.323 s。随后进入 Hold，vx/wz 都为零。证据目录 `field-data/verification/field-handoff-41a5150-pAk7FH`。
- `f85717a` 候选另行安装了等待 Release 状态的已验证 UI。现场用过的 41a5150 仍保留，用于对照实车运行。

- 室内被动 GNSS 检查收到 GGA/GSA/RMC/GSV，全部有效性状态为无 Fix。随后使用现车 NTRIP 配置、仅启动 GNSS 的 45 s 读数中，1,017 个质量快照的 GGA 质量均为 0，差分客户端处于 `waiting_gga`，没有同历元 GST。这两次未提供可开始 WGS84 准备动作的证据。记录 `field-data/verification/field-gnss-passive-rSNwj5`、`field-data/verification/gnss-configured-site-A5sChk`。

### 室外真实 RTK 与首个 WGS84 预检

- 室外首次静态 75 s 有效 GGA 最高质量 2。第二次核对实际回注后收到 670 帧可用 Fixed，23 星，NTRIP 收到 352 个 RTCM 帧且串口写入年龄约 0.134 s。原输出无 GST。证据 `field-data/verification/gnss-outdoors-o9xO4J`、`field-data/verification/gnss-outdoors-rtcm-OCdauA`。
- 按 UM982 官方手册，在接收机当前端口临时启用 1 Hz GST。6 帧都与 GGA 同 UTC 时刻，纬度/经度 1σ 约 0.026/0.029–0.030 m。现场将天线正下方的地面点标记为 base_link 停车目标，另一次静态记录取五个有同历元 GST 的 Fixed 中位数，水平散布 0.027 m。证据 `field-data/verification/gnss-outdoors-gst-ESPLM7` 与 `field-data/verification/field-wgs84-target-lXNcwk`。
- 预检在 3.74 m 与 1.80 m 起点因路线距离拒绝；连续独立 Fixed 确认最后起点稳定在 2.70 m 左右。该起点的 2.657 m 运行预检触发配准走廊检查，录包没有任何 Action goal 或非零底盘速度。离线用录包的同帧栅格，旧配准圆形包络在全局和局部图都拒绝，既有 Guard 的实际足迹在高分辨率局部图沿 .4、1.2、2.0、2.757 m 直线均可通行。证据 `field-data/verification/field-wgs84-measured-f85717a-O4rg7Z`。下一候选针对这一个差异修复。

- 配准候选改用 5 cm 局部栅格及 Guard 的同一底盘足迹扫掠。未知、致命、地图外区域保留拒绝，转弯候选先检查原地转向，再检查 2 m 前行段。将局部候选用同一公共时间戳的 TF 投到固定 map；地图、速度、制动和输出仲裁沿用既有实现。新增真实现象对应的窄走廊、未知/致命障碍和受阻转向回归用例。

- 新足迹几何在同一份真实录包的 4 组局部栅格与 odom 配对上均选出直行 2 m。未知/占用栅格横穿实际足迹、以及转向扫掠未知时仍拒绝。运行记录 `field-data/verification/field-wgs84-measured-f85717a-O4rg7Z/local-footprint-replay.txt`。

- 候选 `645a1dd` 用真实局部栅格回归和 31 项单测运行。更新合成试验改为等待局部栅格，复测 WGS84 准备取消、变换提交、导航、GNSS 融合输入和停稳复核均通过。
- 用户监督的 `645a1dd` 从距标记 2.704 m 发送一个带复核的真实 WGS84 目标。新配准足迹检查通过，配准实际行进，LIO 位移 1.8264 m。443 个质量样本均为可用 Fixed，包含 22 组同历元 GST。约 21.49 s 时配准因有效基线不足返回 `LOCALIZATION_UNAVAILABLE`，未提交变换或进入正式地理导航。取消输出、停稳和 Hold 均有效。指令/实测前向峰值分别为 0.200/0.213 m/s。证据 `field-data/verification/field-wgs84-645a1dd-bnp6mz`。该实车配置关闭实时融合。

- 已用该实车录包重放确诊基线丢失。共配对 425 个观测，最大间隔 0.184 s。旧 256 帧缓存剔除起步段，抽取的有效 map/ENU 基线为 1.024/1.020 m，未达到 1.2 m 门限；保留全程后在约 16.56 s 可拟合，RMS 0.01255 m，前后子轨迹航向差约 0.76°。
- 新候选按任务原有准备时限保留数据，并用仍新鲜的最近一次配对实测协方差做停稳后复核。20 Hz GGA 中间的无 GST 帧仍按降级处理，不能覆盖已关联的 1 Hz 精度观测。时间戳、失效、同会话和误差门限继续有效。新增 20 Hz 慢速轨迹、按时限剪裁、混合精度与过期停稳复核回归。

- 修复候选 `2bcf420` 已独立安装并封存。33 项单测通过；从该候选 install 以 0.20 m/s 指令上限、20 Hz GGA 和 1 Hz GST 执行合成闭环，准备取消、再次配准、投影、地理任务与停稳复核均通过，模拟终点 x=2.474 m。软件记录 `field-data/verification/wgs-real-cadence-result.txt`；用该候选安装重新执行真实录包分析的证据为 `field-data/verification/field-wgs84-645a1dd-bnp6mz/alignment-exact-2bcf420.json`。实车等待复测，保留本地直行和交接通过的独立记录。

### 候选 2bcf420 的室外复测

- 重新监督运行一个 WGS84 目标，启动预检到标记距离 3.169 m。约 14.70 s 达到配准门限，停稳后提交版本 1，刷新 costmap 后正常投影并进入导航。LIO/轮速固定 map 继续使用同一控制链；这次室外运行关闭实时 GNSS 融合。52 s 试验总期限耗尽，结果 `TIMEOUT`，map 残差 0.542 m，停止已确认并进入 Hold。未进入停稳地理复核。速度指令峰值 0.200 m/s，反馈峰值 0.226 m/s。录包中 1,069 个质量样本均为 Fixed，其中 54 个匹配 GST；录包和摘要在 `field-data/verification/field-wgs84-2bcf420-OVnG6l`。
- 末段停滞由两项直接证据解释。DWA 记录 76 次 odom→map 向未来查询，最大超前 5.892 ms。全局规划另有 15 次拒绝；起点附近左后方车体相对位置重复出现近距点，致命栅格距车心约 0.20–0.283 m，小于 0.315 m 规划硬净距，目标栅格本身空闲。实际近距点来源未确定。Guard 的 3 次拦截发生在准备阶段。
- 针对已量化的 TF 请求，下一候选把全局 EKF 的 TF 发布偏移从 50 ms 改成 60 ms。按录包日志时序计算，额外 10 ms 覆盖了全部 76 次查询差值。定位测量时间和 TF 发布所有权保持原值。真实近距回波须先查明来源，场地需能保留现有规划净距，然后做下一次监督 WGS84。偏移配置使用 [robot_localization Noetic 文档的标准参数](https://github.com/cra-ros-pkg/robot_localization/blob/noetic-devel/doc/state_estimation_nodes.rst#transform_time_offset)。
- 实车会话结束后撤销 UM982 的临时 GST 输出。恢复后继续读到 GGA/GSA/GSV/RMC，四秒内 GST 为零，未将临时设置写入接收机持久配置。

- TF 时序候选 `40b150c5c805889ca5cc53df1616f8d2556aec00` 已独立安装封存。新 install 从空 overlay 终端可定位接口、bringup 和 UI，33 项单测及 20 Hz/1 Hz、0.20 m/s 指令上限的 WGS84 合成闭环通过。模拟终点 x=2.472 m，记录 `field-data/verification/wgs-tf-timing-result.txt`。实车以本地直行、运动中替换/取消、WGS84 停稳配准和进入导航为已验证范围；WGS84 到点、实时 GNSS 融合、转向与绕障需监督复测。
