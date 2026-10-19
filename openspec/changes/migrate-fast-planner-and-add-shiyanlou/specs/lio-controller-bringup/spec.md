## Purpose

为主仓库提供不依赖 Fast-LIO2 或 ICP 的二维 AMCL/Nav2 导航入口，让用户在自行建图后传入地图并选择 Fast Planner 或 DWB。沿用初稿的规格目录名称，不表示本次要求集成 LIO 导航。

## ADDED Requirements

### Requirement: User supplied two dimensional map
二维导航入口 SHALL 接受 `map:=地图.yaml`，使用该二维占据栅格地图，不依赖 PCD、地图转换或额外关联元信息。未提供地图或路径无效时 SHALL 明确报错，不静默使用旧示例地图。

#### Scenario: Map supplied by user
- **WHEN** 用户提供有效的二维地图 YAML 及其图像文件
- **THEN** map server 加载该地图供 AMCL 和 Nav2 使用，无需启动 Fast-LIO2 或 ICP

#### Scenario: Map not yet created
- **WHEN** 用户未提供地图便尝试启动二维导航
- **THEN** 入口提示提供地图，但不影响独立 Gazebo 场景启动

### Requirement: Independent AMCL navigation chain
二维入口 SHALL 使用二维激光 `/scan`、轮速 `/odom`，由 AMCL 提供 `map → odom`、仿真底盘提供 `odom → base_footprint`；SHALL 不启动 LIO/ICP 或使用 `/lio/nav_odom`。原有三维导航入口 SHALL 保持不变，两条流程 SHALL 不同时运行。

#### Scenario: Start two dimensional navigation
- **WHEN** 用户运行二维机器人仿真并启动新导航入口
- **THEN** 使用轮速反馈和 AMCL，无重复 LIO TF 发布器，导航节点使用仿真时钟

### Requirement: Startup controller selection
二维入口 SHALL 接受 `controller_mode:=dwb|fast_planner`，默认 DWB；非法值 SHALL 报错。两模式 SHALL 使用相同 AMCL、地图和机器人接口，切换通过停止后重新启动实现。

#### Scenario: Load selected plugin
- **WHEN** 用户选择 fast_planner 或 dwb
- **THEN** FollowPath 配置并加载相应插件，不启动第二套机器人或定位器

### Requirement: Preserve controller output contract
Fast Planner 模式 SHALL 复用既有二维 Burger 控制器及相匹配的输出链配置，包括速度限制、预算、smoother 及停车包络假设。SHALL 使用 `/odom` 反馈，明确配置命令路由，不绕过既有安全检查。

#### Scenario: Check migrated configuration
- **WHEN** 对二维入口执行参数检查和插件加载检查
- **THEN** 插件资源来自本仓库，反馈/速度输出接口与仿真匹配，配置保留现有安全限制；检查结果不被描述为实验楼闭环导航已通过
