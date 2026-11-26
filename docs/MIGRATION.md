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
