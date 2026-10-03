# turtlebot3-fast-lio2

Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11 中，使用独立的 TurtleBot3 Burger 3D LiDAR 仿真模型，实验 Fast-LIO2 点云建图、保存 PCD、在保存的 3D PCD 上 ICP 重定位，以及 Nav2 地面导航。支持在 RViz 中点击 **2D Pose Estimate** 给 ICP 初始猜测、点击 **Nav2 Goal** 导航。已知免费空间之外的目标及无人值守导航**不适用**。

> 本仓库仅用于 ROS 2 Humble / Gazebo Classic 仿真实验。Fast-LIO2 采用上游 Git 子模块＋本项目补丁；克隆后须运行 `scripts/prepare.sh`，普通 `git clone` 不会得到可编译的改动。

## 目录及用途

```text
turtlebot3-fast-lio2/
├── README.md
├── .gitignore                         # 排除 build/install/log、缓存、密钥
├── .gitmodules                        # 固定上游 FAST_LIO ROS2 分支提交
├── scripts/prepare.sh, build.sh       # 获取第三方依赖、补丁、编译三工作区
├── scripts/smoke-test.sh               # 可选：独立 ROS domain 的无图形联调
├── patches/fast_lio2_gazebo.patch     # 上游 ROS2 分支的 Gazebo XYZI/建图代码差异
├── patches/fast_lio2_assets/         # 两套 Gazebo YAML 配置、建图 RViz 配置
├── turtlebot3_ws/
│   ├── src/turtlebot3_lio_sim/        # 独立 3D Burger；SDF、URDF、launch、RViz 配置
│   ├── start-world.sh                  # 官方 2D / 新 3D / 轻量 3D / 导航 3D 世界
│   └── start-rviz.sh                   # 原 2D 或普通 3D 传感器可视化
├── fast_lio2_ws/
│   ├── src/FAST_LIO/                  # 上游 hku-mars/FAST_LIO 固定提交的 Git 子模块
│   ├── maps/tb3_fast_lio2.pcd         # 可选：从仿真保存的 3D 示例地图（约 1.8 MB）
│   ├── start-fast-lio2.sh             # 建图
│   ├── start-fast-lio2-nav.sh         # 独立调试：导航时的局部 LIO
│   └── start-lio-rviz.sh              # 建图用 RViz
└── tb3_nav3d_ws/
    ├── src/FASTLIO2_ROS2_reference/
    │   ├── interface/                 # 上游 MIT 协议 ROS 服务定义
    │   └── localizer/                 # 上游 MIT 协议 ICP 定位器及本项目的修改
    ├── maps/tb3_lio_ground.{yaml,pgm} # 从同一份 PCD 派生的保守 2D 地面规划图
    ├── config/nav2_3d.yaml           # Nav2 规划器、控制器、3D 体素障碍物层等参数
    ├── launch/                        # Fast-LIO2 + ICP + Nav2 分阶段启动
    ├── rviz/nav3d.rviz                # 导航地图、机器人、初始位姿与 Nav2 Goal
    ├── tools/                          # 地图转换、初始位姿适配、定位新鲜度检查等
    ├── start-navigation.sh            # 常规：整套导航，一个终端
    ├── start-nav3d-rviz.sh            # 导航 RViz
    ├── start-localizer.sh             # 仅调试：单独启动 PCD 定位器
    ├── start-nav3d.sh                 # 仅调试：单独启动 Nav2
    └── relocalize.sh                  # 无 RViz 时手动提供 ICP 初始位姿
```

官方 `turtlebot3` 和 `turtlebot3_simulations` 的模型网格、原有 2D 世界和源码**不复制**进本仓库，须作为外部依赖安装。自定义 SDF/URDF 仍会通过 `model://turtlebot3_common/...` 和 `package://turtlebot3_gazebo/...` 引用官方资源。PCD 是保存的 **3D 点云地图**；其派生的 YAML/PGM 仅用于 Nav2 的地面规划，不是旧的 `tb3_map.yaml`。

## 安装与编译

需要 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11、Nav2、RViz、PCL/Eigen、`colcon` 与 `rosdep`。可在 WSL2/WSLg 使用；图形窗口会消耗 CPU。先按官方指南安装 ROS 2 Humble；系统依赖可用 `rosdep` 从各工作区 `src` 解析（需要网络和 sudo）。**无需复制或覆盖已有的 `~/turtlebot3_ws` 等工作区**；以下均在克隆出的仓库内运行。

```bash
git clone --recurse-submodules https://github.com/Mach1ne333/turtlebot3-fast-lio2.git
cd turtlebot3-fast-lio2
./scripts/prepare.sh          # 上游固定提交、Gazebo 补丁、YAML/RViz、官方 TB3 依赖
# 初次安装的电脑可运行：rosdep install --from-paths turtlebot3_ws/src fast_lio2_ws/src tb3_nav3d_ws/src --ignore-src -r -y --rosdistro humble
./scripts/build.sh            # 按 turtlebot3 / fast_lio2 / nav3d 顺序 colcon build
```

请勿在自己已有的同名工作区中直接执行补丁或复制产物。`prepare.sh` 会把本项目的补丁应用到工作区的 FAST_LIO 子模块，**因此本地显示该子模块 modified 是预期现象**。保存 PCD 时，配置中的 `maps/tb3_fast_lio2.pcd` 是相对 `fast_lio2_ws/` 的路径；建图脚本会先切换到该目录，不依赖 Linux 用户名。

## 脚本、参数和模式

| 脚本（位于对应工作区根目录） | 参数 / 选择 | 用途 |
| --- | --- | --- |
| `turtlebot3_ws/start-world.sh` | `[2d\|3d\|3d-light\|3d-nav] [额外 ROS launch 参数]`，默认 `2d`；如 `gui:=false`、`x_pose:=-2.0`、`y_pose:=-0.5` | `2d`=原版；`3d`=16×360 点云建图模型；`3d-light`=16×240；`3d-nav`=关闭 Gazebo 轮速 TF，由 LIO 提供导航位姿。启动前须停止旧 Gazebo。 |
| `turtlebot3_ws/start-rviz.sh` | `[2d\|3d]`，默认 `2d` | 普通 2D/3D 传感器视图；**导航时另用** `start-nav3d-rviz.sh`。 |
| `fast_lio2_ws/start-fast-lio2.sh` | 无 | 用 Gazebo 的瞬时 XYZI 点云与 IMU 进行 Fast-LIO2 建图；默认不自动打开 RViz。 |
| `fast_lio2_ws/start-lio-rviz.sh` | 无 | 建图时看累积点云和轨迹。 |
| `fast_lio2_ws/start-fast-lio2-nav.sh` | 无 | 调试用导航 LIO 配置；不要与 `start-navigation.sh` 同时运行。 |
| `tb3_nav3d_ws/start-navigation.sh` | 无 | **推荐**；启动 LIO、3D PCD ICP 定位器和 `/initialpose` 桥接。ICP 有效且 LIO/map TF 新鲜后，自动启动 Nav2；不运行 AMCL。 |
| `tb3_nav3d_ws/start-nav3d-rviz.sh` | 无 | 导航 RViz：先用 **2D Pose Estimate** 指定机器人当前概略位置和朝向，后用 **Nav2 Goal** 指定目的地。 |
| `tb3_nav3d_ws/start-localizer.sh` | 无 | 仅调试：单独启动 ICP 和初始位姿适配节点。 |
| `tb3_nav3d_ws/start-nav3d.sh` | 无 | 仅调试：定位有效并检查 TF 新鲜后，单独启动 Nav2。 |
| `tb3_nav3d_ws/relocalize.sh` | `[x y z yaw]`，默认均为 `0`；可用环境变量 `PCD_MAP` 覆盖地图路径 | 无 GUI 时提供 **IMU body 在 PCD 地图坐标系下**的米／弧度初值；`0 0 0 0` 仅适用于建图时的出生位置。须以 ICP `valid=True` 为准，而非 PCD 加载成功。 |

## 建图（分别开终端）

```bash
./turtlebot3_ws/start-world.sh 3d
./fast_lio2_ws/start-fast-lio2.sh
./fast_lio2_ws/start-lio-rviz.sh  # 可选
```

等待 IMU 初始化，移动机器人覆盖需要的区域；保存实际 PCD：

```bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

PCD 保存在仓库的 `fast_lio2_ws/maps/tb3_fast_lio2.pcd`。PCD 更新后，可用 `python3 tb3_nav3d_ws/tools/pcd_to_ground_grid.py fast_lio2_ws/maps/tb3_fast_lio2.pcd tb3_nav3d_ws/maps/tb3_lio_ground.yaml` 重建 YAML/PGM 地面规划图；未观测区域不假定为可通行。**重新建图和上传前检查 PCD 内容，避免将真实环境或私有数据公开。**

## 使用已保存地图导航（分别开三个终端）

```bash
./turtlebot3_ws/start-world.sh 3d-nav
./tb3_nav3d_ws/start-navigation.sh
./tb3_nav3d_ws/start-nav3d-rviz.sh
```

RViz Fixed Frame 为 `map`。加载／预览 PCD **不等于定位有效**：点击 **2D Pose Estimate** 给 ICP 初始猜测，等导航终端显示 `ICP 3D PCD localization VALID` 和 `automatically starting Nav2`，确认 `/navigate_to_pose` action 可用，再用 **Nav2 Goal** 在已知空闲区域内发送短目标。全零初值只适用于与建图时相同的出生位置。不要同时运行旧的独立调试启动脚本。若导航中定位丢失，立即取消目标并人工检查；目前仅有**启动前**新鲜度门禁，没有持续监护和自动急停。请结合 Gazebo 实际位移验证，不凭 action `SUCCEEDED` 断言成功。

## 第三方来源、许可与限制

详细来源、上游固定版本与许可证冲突见 [`THIRD_PARTY_LICENSES.md`](THIRD_PARTY_LICENSES.md)。

- 3D 模型改编自 ROBOTIS TurtleBot3 的 Apache-2.0 模型；官方网格与 2D 包不纳入本仓库，保留来源及适用的版权声明。
- Fast-LIO2 基于 [`hku-mars/FAST_LIO`](https://github.com/hku-mars/FAST_LIO) 的 ROS2 分支、提交 `a4743b0`，并使用其 ikd-Tree 子模块提交 `e2e3f4e`；上游根目录 `LICENSE` 为 **GPL-2.0**，尽管 `package.xml` 声称 `BSD`。发布前按实际授权条款审查，勿直接声称整个仓库均为 MIT/BSD。
- ICP `interface`/`localizer` 取自 [`liangheming/FASTLIO2_ROS2`](https://github.com/liangheming/FASTLIO2_ROS2) 提交 `7baa59f`，仅纳入 MIT 授权的必要包并保留原 `LICENSE`；**不运行该仓库另一套 Fast-LIO2**。
- Gazebo ray 点云是瞬时 XYZI 数据，没有真实旋转雷达的 `ring`/逐点时间；此实验不验证真实雷达运动去畸变。PCD 地面规划图覆盖有限；仅供有人监督的慢速仿真实验。
