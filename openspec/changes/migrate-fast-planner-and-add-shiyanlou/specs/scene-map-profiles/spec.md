## Purpose

沿用现有 Gazebo 统一入口，按启动参数选择二维或三维 TurtleBot3 Burger，所有机器人模式默认显示实验楼。场景加载与地图准备解耦，不要求地图存在；三维支持仅指模型和传感器仿真，不包含三维建图或导航集成。

## ADDED Requirements

### Requirement: Shared building scene with selectable robot
系统 SHALL 沿用 `turtlebot3_ws/start-world.sh` 的位置参数选择 2d 或 3d，并兼容现有 3d-light、3d-nav 模式；省略模式 SHALL 选择 2d。所有模式及项目自有仿真 launch SHALL 默认加载仓库内的 `gazebo_building_editor_models/shiyanlou/shiyanlou.world`，生成所选机器人。SHALL 允许覆盖出生 x/y/yaw，默认出生点 SHALL 在地面空闲位置，不与建筑碰撞体重叠。

#### Scenario: Start without a map
- **WHEN** 用户尚未建图，运行 start-world.sh，省略模式或指定 2d/3d
- **THEN** Gazebo 正常显示实验楼、地面和机器人，不要求二维地图或 PCD，不启动 AMCL/Nav2/LIO 建图节点

### Requirement: Scene fallback without duplicate models
统一入口各模式 SHALL 支持 `scene:=shiyanlou|turtlebot3_world`，未知模式或场景值 SHALL 报错；实验楼 world 已包含建筑，SHALL 不再次生成同一建筑。场景选择 SHALL 不改变机器人模式。

#### Scenario: Preserve existing model variants
- **WHEN** 用户选择 3d-light 或 3d-nav
- **THEN** 使用原轻量模型或关闭轮速 TF 的模型，默认世界同为实验楼，不自动启动 LIO/ICP 或改变模型语义

#### Scenario: Select a world
- **WHEN** 用户选择实验楼或官方世界
- **THEN** 加载所选世界且仅生成一个机器人，不出现重复建筑或偷偷回退其他场景

### Requirement: Scope limited scene acceptance
场景接入 SHALL 以分别选择 2d、3d 时在 Gazebo 中正常看到实验楼与对应机器人作为完成依据；二维 SHALL 提供原二维激光和轮速接口，三维 SHALL 保留原传感器配置。SHALL 不把二维地图采集、PCD 生成或实际导航成功作为本次场景完成条件。原三维建图、定位及导航入口 SHALL 不因场景选择被自动运行或改造。

#### Scenario: Scene ready and navigation deferred
- **WHEN** 场景显示正常而用户尚未建立实验楼地图
- **THEN** 可以记录场景接入完成，文档明确地图由用户建立、导航效果由用户后续测试，不声称导航已经通过
