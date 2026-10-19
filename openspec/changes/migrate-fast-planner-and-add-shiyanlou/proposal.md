## Why

后续以 `turtlebot3-fast-lio2` 为主仓库，需要把外部工作区中已修复的 Fast Planner 迁入，并在 Gazebo 中加载用户提供的实验楼。采用分阶段方式：本次完成场景和源码迁移，用户随后自行建立二维地图并测试导航，三维点云建图和导航以后再考虑。

## What Changes

- 将已修复的 Fast-Planner-2D-ROS2 核心、`fast_planner_nav2` 及已有测试迁入 `tb3_nav_ws/src`；保留来源和许可，不修改原外部工作区，不依赖其 install。
- 沿用 `turtlebot3_ws/start-world.sh`，通过现有位置参数选择 `2d` 或 `3d` 机器人，并兼容现有 `3d-light`、`3d-nav` 变体；所有模式默认加载 `gazebo_building_editor_models/shiyanlou/shiyanlou.world`，省略机器人模式时仍选择 2d。Gazebo 中正常显示实验楼与所选机器人即满足场景接入验收。
- 提供独立于现有 LIO 流程的二维导航入口：用户传入 `map:=地图.yaml`，由 AMCL、Nav2 和 DWB/Fast Planner 完成后续导航；支持 `controller_mode:=dwb|fast_planner`，默认 DWB。
- **BREAKING（默认场景变化）**：原仿真入口所有机器人模式的默认场景从官方世界改为实验楼，可显式选择官方世界回退；不新增 start-world-2d.sh。仅调整三维仿真场景选择，不改变模型传感器/TF 语义；LIO 建图、定位、导航脚本及旧地图保持不动。
- 保留插件既有安全逻辑与可复用的二维 bringup 参数，仅做路径、资源及入口适配；不为本次迁移重新设计算法或制定性能指标。
- 本次只要求独立构建、插件加载、必要参数/接口检查及 Gazebo 场景显示；不要求采集地图或完成实验楼导航闭环。

## Capabilities

### New Capabilities

- `fast-planner-nav2-controller`: 可独立构建加载的既有二维 Fast Planner 控制器，保留原控制及安全行为。
- `lio-controller-bringup`: 主仓库新增的独立二维 AMCL/Nav2 入口，接收用户地图并选择控制器。此路径沿用初稿名称，仅为保持现有规划文件位置；本次不集成 LIO。
- `scene-map-profiles`: 实验楼场景启动与用户地图输入的解耦；不再包含建图、PCD 配套或地图元信息管理。

### Modified Capabilities

无。本仓库此前没有主规格，不修改旧工作区规格。

## Impact

影响主仓库新增的控制器源码、现有统一仿真入口及关联 launch、新增二维导航入口、必要构建支持、README 与来源说明。运行环境仍为 Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11。

明确排除：代用户建图、PCD 转换、ICP/LIO 导航改造、地图 hash/profile 管理、实验楼闭环导航验收、停车时延/RTF/计算耗时定量验收、额外评测平台、Scout2/RL 和真实机器人。除仿真场景加载外，已有三维建图、定位、导航流程及其参数不修改。本次完成只代表场景和插件迁移就绪，不代表实验楼导航效果已验证。
