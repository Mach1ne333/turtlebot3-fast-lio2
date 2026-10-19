# turtlebot3-fast-lio2

Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11 中的 TurtleBot3 Burger 仿真实验。**所有机器人模式现在默认启动 1/3 平面尺寸实验楼 shiyanlou_third**，通过现有启动参数选择二维或三维机器人。二维 AMCL/Nav2 支持 DWB/Fast Planner；实验楼二维地图由用户自行建立，实际导航效果待用户测试。

Fast-LIO2 建图、PCD ICP 定位和三维感知导航流程保留；建图源码和集成已归入根目录的 `mapping_ws/`，不改变算法与传感器语义。旧示例地图属于官方世界，使用它们时必须显式选择 `scene:=turtlebot3_world`。

> 仅用于有人监督的仿真，不适用于实机或无人值守。二维构建无需 FAST_LIO 子模块；原三维流程仍须运行 `scripts/prepare.sh` 获取并补丁化上游源码。

## 目录及用途

```text
turtlebot3-fast-lio2/
├── README.md
├── .gitignore                         # 排除 build/install/log、缓存、密钥
├── .gitmodules                        # 固定上游 FAST_LIO ROS2 分支提交
├── scripts/build-2d.sh                # 仿真、Cartographer 集成与 Fast Planner，不编译 LIO/ICP
├── scripts/humble-2d-env.bash          # 专用终端的干净 Humble + 本仓库二维 overlay
├── scripts/workspaces.bash            # 工作区和默认 PCD 路径的集中定义
├── scripts/prepare.sh, build.sh       # 获取依赖、补丁、编译四个工作区
├── scripts/smoke-test.sh              # 可选：独立 ROS domain 的无图形联调
├── turtlebot3_ws/
│   ├── src/tb3_sim/        # 独立 3D Burger；SDF、URDF、launch、RViz 配置
│   ├── start-world.sh                  # 2d / 3d / 3d-light / 3d-nav，默认均为实验楼
│   └── start-rviz.sh                   # 原 2D 或普通 3D 传感器可视化
├── mapping_ws/                        # 分类父目录；两个算法子目录分别是工作区
│   ├── cartographer/                  # Cartographer 集成工作区，算法仍用系统安装版本
│   │   ├── src/cartographer_bringup/ # Lua、launch、RViz 与测试
│   │   └── start-mapping-2d.sh         # 二维建图
│   └── fast_lio2/                     # FAST-LIO2 工作区
│       ├── src/FAST_LIO/              # 固定提交的上游 Git 子模块
│       ├── src/fast_lio_bringup/       # 本项目 config/launch/rviz、显示过滤与测试
│       ├── patches/fast_lio2_gazebo.patch # 仅算法必要的 Gazebo 适配补丁
│       ├── maps/tb3_fast_lio2.pcd      # 3D 示例地图及用户生成的 PCD
│       ├── start-fast-lio2.sh          # 建图
│       └── start-lio-rviz.sh           # 建图用 RViz
└── tb3_nav_ws/                       # 二维/三维共用的定位导航工作区
    ├── src/FASTLIO2_ROS2_reference/
    │   ├── interface/                 # 上游 MIT 协议 ROS 服务定义
    │   └── localizer/                 # 上游 MIT 协议 ICP 定位器及本项目的修改
    ├── src/fast_planner_nav2/         # 迁入的 Nav2 Controller 插件及已有测试
    ├── src/third_party/Fast-Planner-2D-ROS2/ # 已修复核心与来源记录
    ├── config/2d/                    # 独立二维 DWB/Fast Planner 参数
    ├── start-navigation-2d.sh         # 用户 map + AMCL，不启动 Gazebo/LIO/ICP
    ├── maps/tb3_lio_ground.{yaml,pgm} # 旧官方世界 PCD 派生地图，不是实验楼地图
    ├── config/nav2_3d.yaml           # Nav2 规划器、控制器、3D 体素障碍物层等参数
    ├── config/tb3_gazebo_nav.yaml    # 导航专用 LIO 参数，禁用地图保存
    ├── launch/                        # Fast-LIO2 + ICP + Nav2 分阶段启动
    ├── rviz/nav3d.rviz                # 导航地图、机器人、初始位姿与 Nav2 Goal
    ├── tools/                          # 地图转换、初始位姿适配、定位新鲜度检查等
    ├── start-navigation.sh            # 常规：整套导航，一个终端
    ├── start-nav3d-rviz.sh            # 导航 RViz
    ├── start-fast-lio2-nav.sh          # 仅调试：单独启动导航 LIO 里程计
    ├── start-localizer.sh             # 仅调试：单独启动 PCD 定位器
    ├── start-nav3d.sh                 # 仅调试：单独启动 Nav2
    └── relocalize.sh                  # 无 RViz 时手动提供 ICP 初始位姿
```

跨工作区脚本使用 `scripts/workspaces.bash` 集中定义目录；工作区内部的脚本/工具从自身位置解析资源，不依赖调用时的当前目录。路径维护约定见 [工作区与路径](docs/workspaces.md)。

`mapping_ws/` 的项目集成只负责建图；导航专用脚本、LIO 参数和 launch 全部放在 `tb3_nav_ws/`。导航通过自己的 `launch/lio_odometry.launch.py` 复用已编译的 `fast_lio` 算法，不调用建图 launch，不保存/覆盖 PCD，也不重复维护算法源码。

仿真源码只维护 `turtlebot3_ws/src/tb3_sim/`；不再克隆或编译官方 `turtlebot3`、`turtlebot3_simulations`、`turtlebot3_msgs` 源码仓库。官方网格、原有 2D 模型和示例世界通过 ROS Humble 系统包安装。自定义 SDF/URDF 仍会通过 `model://turtlebot3_common/...` 和 `package://turtlebot3_gazebo/...` 引用这些资源，不能把“不需要官方源码”理解为“不需要系统资源包”。PCD 是保存的 **3D 点云地图**；其派生的 YAML/PGM 仅用于 Nav2 的地面规划，不是旧的 `tb3_map.yaml`。

## 实验楼与二维 Fast Planner

以下命令在仓库根目录运行。先安装系统 Humble、Gazebo Classic、Nav2 和官方 Burger 资源，例如：

```bash
sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description \
  ros-humble-nav2-bringup ros-humble-gazebo-ros-pkgs ros-humble-robot-localization \
  ros-humble-cartographer-ros
# 解析本次新增包依赖（包括 Eigen/Boost/NLopt 和已有测试工具）：
rosdep install --from-paths turtlebot3_ws/src/tb3_sim \
  mapping_ws/cartographer/src \
  tb3_nav_ws/src/fast_planner_nav2 \
  tb3_nav_ws/src/third_party/Fast-Planner-2D-ROS2/fast_planner \
  --ignore-src -r -y --rosdistro humble
./scripts/build-2d.sh
```

无需编译或启动 FAST_LIO/ICP，也不依赖外部 `~/turtlebot3_ws/install`。构建和新入口使用系统 Python，避免 Conda 的 Python/库污染；环境助手会重置 PATH/ROS overlay，请在专用终端使用。

### 1. 启动场景，选择机器人

```bash
./turtlebot3_ws/start-world.sh             # 默认 2d + shiyanlou_third
./turtlebot3_ws/start-world.sh 2d          # 二维激光 + IMU/底盘里程计 EKF，后续用于二维建图
./turtlebot3_ws/start-world.sh 3d          # 原 3D LiDAR + IMU 模型，仅启动仿真
# 保留原模型变体，默认场景也为实验楼：
./turtlebot3_ws/start-world.sh 3d-light
./turtlebot3_ws/start-world.sh 3d-nav
```

**以上是互斥示例，切换前先 Ctrl+C 停止旧 Gazebo。** 默认出生位置为 `x=-2.0, y=0.4, yaw=0`，可附加 `x_pose:=... y_pose:=... yaw_pose:=...`；`gui:=false` 可关闭窗口。各模式均可附加 `scene:=shiyanlou` 使用原尺寸实验楼，或 `scene:=turtlebot3_world` 回退官方世界。当前默认场景基于 1/3 版本，窄过道和小门口的净宽已扩大为之前的 2.5 倍，总体约 25.43 × 14.71 m，墙高仍为 2.5 m；不是整栋建筑统一放大。`scene:=shiyanlou_quarter` 可选择 1/4 版本，其对应窄过道和门口现为该版本自身原净宽的 **2 倍**（已从之前 2.5 倍回调），总体约 18.00 × 10.42 m。原尺寸、1/4 版本及加宽前的地图都不能直接用于当前场景，需要重新建图。尺寸、备份和修改前后对照见 [过道加宽记录](docs/shiyanlou_passage_widening.md)。

实验楼较大，初始镜头为建筑总览，机器人可能很小；可在 Gazebo 左侧 Models 中选中机器人并双击聚焦。二维实体名 `burger`，三维为 `burger_lidar3d`。3d-nav 仍禁用轮速 TF，不能用作本次二维 AMCL 机器人。选择 3d 不会自动启动建图或导航。

### 二维 EKF：IMU 与底盘里程计融合

`start-world.sh 2d` 现在默认同时启动 `robot_localization/ekf_node`：

```text
Gazebo 底盘 /wheel/odom ─┐
                       ├→ EKF → /odom + odom→base_footprint
Gazebo IMU /imu ────────┘
```

Gazebo 原始底盘 TF 被关闭，避免与 EKF 冲突；建图工具使用该 TF，Nav2/速度平滑器仍使用 `/odom`，命令无需改。仅融合底盘前向速度、零侧向速度约束和 IMU Z 轴角速度，不融合绝对 yaw 或加速度。滤波器局部位姿从零附近开始，不等于 Gazebo 世界中的出生坐标。

配置：`turtlebot3_ws/src/tb3_sim/config/ekf_2d.yaml`。只影响 2d，原始官方 SDF 不修改，三维模型和 LIO 流程不变。若要回退，先停止旧仿真和建图/导航，再运行：

```bash
./turtlebot3_ws/start-world.sh 2d use_ekf:=false
```

此时恢复 Gazebo 原始 `/odom` 和底盘 TF。不要在仿真运行中切换或同时启动第二个 EKF。默认 `use_ekf:=auto` 只为 2d 开启；3d 不启动 EKF。检查和测试记录见 [二维 EKF 说明](docs/ekf_2d.md)。

### 2. 用户自行建立二维地图

当前二维建图入口使用 **Cartographer**，不再默认推荐 SLAM Toolbox。先停止旧 SLAM/AMCL/Nav2，按步骤 1 启动 **2d**；若使用最新 quarter 场景，启动时显式加 `scene:=shiyanlou_quarter`。另开终端：

```bash
cd ~/turtlebot3-fast-lio2
./mapping_ws/cartographer/start-mapping-2d.sh  # 自动打开干净的 RViz；可加 rviz:=false
```

使用 `/scan` + `/odom`，Cartographer 只发布 `map → odom`，不与 EKF/底盘争抢 TF；IMU 经 EKF 间接参与，没有再次直接融合。RViz Fixed Frame 已设为 `map`。另开专用终端遥控：

```bash
source scripts/humble-2d-env.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.4
```

机器人停下、建图节点仍运行时保存（新文件名前缀避免覆盖旧地图）：

```bash
source scripts/humble-2d-env.bash
mkdir -p tb3_nav_ws/maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/tb3_nav_ws/maps/shiyanlou_cartographer" \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
```

生成 YAML/PGM。保存后停止建图和遥控，再进入下面的 AMCL/Nav2 流程。场景改为 2 倍后必须重新建图，不能接续旧地图/SLAM 会话。详细配置、最终优化与测试边界见 [Cartographer 二维建图](docs/cartographer_2d.md)。不提供预建实验楼地图，不要求 PCD；换算法不保证消除采样空洞或墙体重影。

### 3. 提供地图后进行二维导航测试

保留 2d Gazebo，在第二个终端执行：

```bash
./tb3_nav_ws/start-navigation-2d.sh map:=/绝对路径/shiyanlou.yaml controller_mode:=fast_planner
# 停止原导航后切换（省略 controller_mode 也默认 DWB）：
./tb3_nav_ws/start-navigation-2d.sh map:=/绝对路径/shiyanlou.yaml controller_mode:=dwb
```

默认打开导航 RViz；可附加 `rviz:=false`。先用 **2D Pose Estimate** 在你自己的地图上设置初始位姿，再用 **Nav2 Goal** 测试短目标。地图 YAML 引用的图像必须存在，未提供地图会明确报错，不会使用旧示例地图。

这是独立的 `map → odom → base_footprint` AMCL 链，使用 `/scan`、`/odom`，不启动第二个 Gazebo，也不启动 LIO/ICP；不能与原三维导航/定位同时运行。正常跟踪命令经 `/cmd_vel_nav → velocity_smoother → /cmd_vel`；Nav2 标准恢复行为沿用原配置，直接发布 `/cmd_vel`。插件不是官方 Fast-Planner 的完整移植，不宣称论文复现。

**交付边界：场景/插件迁移就绪，实际导航待用户测试。** 已检查构建、插件加载、2d/3d 实验楼 GUI 和基本接口；未测试实验楼闭环导航或性能指标。见 [迁移检查记录](docs/fast_planner_migration.md)。

## 安装与编译（三维 LIO 原流程）

需要 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11、Nav2、RViz、PCL/Eigen、`colcon` 与 `rosdep`。可在 WSL2/WSLg 使用；图形窗口会消耗 CPU。先按官方指南安装 ROS 2 Humble；系统依赖可用 `rosdep` 从各工作区 `src` 解析（需要网络和 sudo）。**无需复制或覆盖已有的 `~/turtlebot3_ws` 等工作区**；以下均在克隆出的仓库内运行。

```bash
git clone --recurse-submodules https://github.com/Mach1ne333/turtlebot3-fast-lio2.git
cd turtlebot3-fast-lio2
./scripts/prepare.sh          # 仅初始化 FAST_LIO 子模块并应用 Gazebo 算法补丁；不下载官方 TB3 源码
# 初次安装的电脑可运行：rosdep install --from-paths turtlebot3_ws/src mapping_ws/cartographer/src mapping_ws/fast_lio2/src tb3_nav_ws/src --ignore-src -r -y --rosdistro humble
./scripts/build.sh            # 仿真 → Cartographer 集成 → FAST-LIO2 → 导航
```

完整构建脚本在仿真工作区也只选择 `tb3_sim`；即使旧官方源码目录还在，也不会把它们作为编译目标。`scripts/smoke-test.sh` 固定使用系统官方世界与配套示例地图，不继承实验楼默认场景；这不是新坡道场景的导航验收。

请勿在自己已有的同名工作区中直接执行补丁或复制产物。`prepare.sh` 会把本项目的补丁应用到工作区的 FAST_LIO 子模块，**因此本地显示该子模块 modified 是预期现象**。配置、launch 和 RViz 资源独立安装在 `fast_lio_bringup`，不再复制到上游源码。建图入口通过 `map_file_path` 传入默认 PCD 的绝对路径（由 `scripts/workspaces.bash` 定义），不依赖执行时的当前目录或 Linux 用户名。

## 脚本、参数和模式

| 脚本（位于对应工作区根目录） | 参数 / 选择 | 用途 |
| --- | --- | --- |
| `turtlebot3_ws/start-world.sh` | `[2d\|3d\|3d-light\|3d-nav] [额外 ROS launch 参数]`，默认 `2d`；所有模式默认 `scene:=shiyanlou_third`；支持 x/y/yaw_pose、gui | 保留原模型选择；`3d-nav` 仍关闭轮速 TF。原三维地图流程请显式使用 `scene:=turtlebot3_world`。切换前须停止旧 Gazebo。 |
| `mapping_ws/cartographer/start-mapping-2d.sh` | 可选 `rviz:=false` | 二维 Cartographer 建图，使用单独运行的 2d 仿真，默认带 map 坐标系 RViz。 |
| `tb3_nav_ws/start-navigation-2d.sh` | 必填 `map:=...yaml`；可选 `controller_mode:=dwb\|fast_planner`、`rviz:=false` | 独立二维 AMCL/Nav2，默认 DWB，不启动仿真或 LIO。 |
| `turtlebot3_ws/start-rviz.sh` | `[2d\|3d]`，默认 `2d` | 普通 2D/3D 传感器视图；**导航时另用** `start-nav3d-rviz.sh`。 |
| `mapping_ws/fast_lio2/start-fast-lio2.sh` | 无 | 用 Gazebo 的瞬时 XYZI 点云与 IMU 进行 Fast-LIO2 建图；默认不自动打开 RViz。 |
| `mapping_ws/fast_lio2/start-lio-rviz.sh` | 默认 `min_z:=-0.25`，可覆盖；离线 PCD 显示用 `use_sim_time:=false` | 自动启动显示高度过滤与 RViz，默认隐藏低处地面点；不改变建图或保存的 PCD。 |
| `tb3_nav_ws/start-fast-lio2-nav.sh` | 可选 `use_sim_time:=false` | 仅调试导航 LIO 里程计，不启动 ICP/Nav2、不保存地图；不要与建图或 `start-navigation.sh` 同时运行。 |
| `tb3_nav_ws/start-navigation.sh` | 可选 `pcd_map:=...pcd map:=...yaml`，须成套选择 | **推荐**；启动 LIO、3D PCD ICP 定位器和 `/initialpose` 桥接。ICP 有效且 LIO/map TF 新鲜后，自动启动 Nav2；不运行 AMCL。 |
| `tb3_nav_ws/start-nav3d-rviz.sh` | 无 | 导航 RViz：先用 **2D Pose Estimate** 指定机器人当前概略位置和朝向，后用 **Nav2 Goal** 指定目的地。 |
| `tb3_nav_ws/start-localizer.sh` | 无 | 仅调试：单独启动 ICP 和初始位姿适配节点。 |
| `tb3_nav_ws/start-nav3d.sh` | 可选 `map:=...yaml`，须与正在使用的定位 PCD 对齐 | 仅调试：定位有效并检查 TF 新鲜后，单独启动 Nav2。 |
| `tb3_nav_ws/relocalize.sh` | `[x y z yaw]`，默认均为 `0`；可用环境变量 `PCD_MAP` 覆盖地图路径 | 无 GUI 时提供 **IMU body 在 PCD 地图坐标系下**的米／弧度初值；`0 0 0 0` 仅适用于建图时的出生位置。须以 ICP `valid=True` 为准，而非 PCD 加载成功。 |

## 三维建图原流程（分别开终端）

以下沿用官方世界与原地图文件名（目录已迁到 `mapping_ws/fast_lio2/`），**不是实验楼建图集成**，以免覆盖或混用地图：

```bash
./turtlebot3_ws/start-world.sh 3d scene:=turtlebot3_world
./mapping_ws/fast_lio2/start-fast-lio2.sh
./mapping_ws/fast_lio2/start-lio-rviz.sh  # 可选
```

`start-lio-rviz.sh` 自动启动独立的显示过滤节点，RViz 默认订阅 `/Laser_map_display`，保留当前用户设定，只显示 `camera_init` 中 `z >= -0.25 m` 的点；可用 `min_z:=0.05` 覆盖。不需要另开过滤终端；关闭 RViz 或 Ctrl+C 会同时停止该显示节点，不停止 FAST-LIO2。可用 `./mapping_ws/fast_lio2/start-lio-rviz.sh min_z:=0.2` 提高裁剪高度。此操作不是地面分割，也会隐藏阈值以下的墙脚和低矮物体；原始 `/Laser_map`、建图输入、PCD 和 bag 均不受影响。若改回原始话题或启用原始 aligned scan 图层，仍会看到地面。仅直接打开 `.rviz` 文件不会自动启动过滤节点，需使用此脚本。地图显示链路使用 Reliable QoS。仅加载已保存的 PCD、没有 Gazebo/bag 时，使用 `./mapping_ws/fast_lio2/start-lio-rviz.sh use_sim_time:=false`（系统时间）；实时仿真和 bag 重建保留默认仿真时间。

等待 IMU 初始化，移动机器人覆盖需要的区域；**保持 FAST-LIO2 运行**，保存实际 PCD（RViz 的 Save Config 不保存地图）：

```bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

PCD 保存在仓库的 `mapping_ws/fast_lio2/maps/tb3_fast_lio2.pcd`。PCD 更新后，可用 `python3 tb3_nav_ws/tools/pcd_to_ground_grid.py mapping_ws/fast_lio2/maps/tb3_fast_lio2.pcd tb3_nav_ws/maps/tb3_lio_ground.yaml` 重建 YAML/PGM 地面规划图；未观测区域不假定为可通行。**重新建图和上传前检查 PCD 内容，避免将真实环境或私有数据公开。**

## 三维原流程：使用已保存地图导航（分别开三个终端）

仓库自带 PCD/地面图只匹配官方世界；本次未接入实验楼三维导航。

```bash
./turtlebot3_ws/start-world.sh 3d-nav scene:=turtlebot3_world
./tb3_nav_ws/start-navigation.sh
./tb3_nav_ws/start-nav3d-rviz.sh
```

RViz Fixed Frame 为 `map`。加载／预览 PCD **不等于定位有效**：点击 **2D Pose Estimate** 给 ICP 初始猜测，等导航终端显示 `ICP 3D PCD localization VALID` 和 `automatically starting Nav2`，确认 `/navigate_to_pose` action 可用，再用 **Nav2 Goal** 在已知空闲区域内发送短目标。全零初值只适用于与建图时相同的出生位置。不要同时运行旧的独立调试启动脚本。若导航中定位丢失，立即取消目标并人工检查；目前仅有**启动前**新鲜度门禁，没有持续监护和自动急停。请结合 Gazebo 实际位移验证，不凭 action `SUCCEEDED` 断言成功。

## 第三方来源、许可与限制

详细来源、上游固定版本与许可证冲突见 [`THIRD_PARTY_LICENSES.md`](THIRD_PARTY_LICENSES.md)。

- 3D 模型改编自 ROBOTIS TurtleBot3 的 Apache-2.0 模型；官方网格与 2D 包不纳入本仓库，保留来源及适用的版权声明。
- Fast-LIO2 基于 [`hku-mars/FAST_LIO`](https://github.com/hku-mars/FAST_LIO) 的 ROS2 分支、提交 `a4743b0`，并使用其 ikd-Tree 子模块提交 `e2e3f4e`；上游根目录 `LICENSE` 为 **GPL-2.0**，尽管 `package.xml` 声称 `BSD`。发布前按实际授权条款审查，勿直接声称整个仓库均为 MIT/BSD。
- ICP `interface`/`localizer` 取自 [`liangheming/FASTLIO2_ROS2`](https://github.com/liangheming/FASTLIO2_ROS2) 提交 `7baa59f`，仅纳入 MIT 授权的必要包并保留原 `LICENSE`；**不运行该仓库另一套 Fast-LIO2**。
- Gazebo ray 点云是瞬时 XYZI 数据，没有真实旋转雷达的 `ring`/逐点时间；此实验不验证真实雷达运动去畸变。PCD 地面规划图覆盖有限；仅供有人监督的慢速仿真实验。
