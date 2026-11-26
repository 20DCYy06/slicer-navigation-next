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
