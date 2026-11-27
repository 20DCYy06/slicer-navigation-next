# Slicer 导航

Slicer 小车的 ROS Noetic 导航项目，提供短距本地导航、WGS84 到点、实时 RTK 融合、避障、现场界面和 BT 接口。

团队仓库位于 [20DCYy06/slicer-navigation-next](https://github.com/20DCYy06/slicer-navigation-next)。

车端日常开发与调试工作区为 `/home/ugv/workspace/slicer-navigation-next`。稳定入口从工作区的 `current` 选择一份已封存发布。旧 `/home/ugv/workspace/slicer` 保留历史和 V1.9 成套回退。

```bash
cd /home/ugv/workspace/slicer-navigation-next
./current/run
# 在另一个终端运行
./current/run ui
```

- [现场操作、停车、录包及 V1.9 回退](docs/FIELD_OPERATIONS.md)
- [BT Action、状态与控制接口](docs/INTERFACE.md) · [Python 调用示例](docs/examples/navigate.py)
- [迁移与验证记录](docs/MIGRATION.md)

## 从源码构建

在已配置 Ubuntu 20.04、ROS Noetic 软件源及 `/opt/ros/noetic` 的 Jetson 上使用系统 Python 3。需要 Git、CMake、C++ 编译器、`python3-dev`、`rosdep` 和 `vcs`。本仓的 [dependencies.repos](dependencies.repos) 固定 FAST-LIO、Livox ROS 驱动、Livox SDK2 和 C30D 主机代码的提交。首次配置 `rosdep` 时运行 `sudo rosdep init` 和 `rosdep update --include-eol-distros`。

首次从团队仓库检出后运行以下命令。构建脚本会就地编译 Livox SDK2，随后以 Catkin 构建车端。已有工作区从 `source` 开始。

```bash
git clone https://github.com/20DCYy06/slicer-navigation-next.git
cd slicer-navigation-next
source /opt/ros/noetic/setup.bash
vcs import . < dependencies.repos
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
tools/build_candidate.sh -j2
release_dir=$(tools/stage_candidate.sh)
```

封存目录位于 `releases/<源码提交>/`，包含运行入口、安装空间、现场文档、V1.10 固件和 `release.json`。封存需要干净的已提交源码。首次部署已选定的稳定版时，在工作区根目录用 `ln -sT "$release_dir" current` 建立本地启动指针。更新稳定版时先退出当前导航，再用 `ln -sfnT "$release_dir" current` 切换指针。NTRIP 凭据和录包保存在车端，路径见[现场操作](docs/FIELD_OPERATIONS.md)。

## 开发分支

`main` 以交付候选 `94ecf19` 为基线。`37c2033` 到 `94ecf19` 只更新了 `docs/MIGRATION.md`，车端代码与配置一致。

`performance/wgs84-dwa-170` 保留封存候选 `b0633df`。性能调试在新仓库进行，实车从候选自身的 `run` 启动。先用 V1.10、0.20 m/s、开启实时 RTK 融合完成 5～10 米 WGS84 到点，再验证复杂绕障、行进换点和逐档提速。通过实车验证后将性能改动合入 `main`。
