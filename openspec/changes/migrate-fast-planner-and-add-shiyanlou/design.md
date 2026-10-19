## Context

范围见 proposal.md。用户已确认分阶段推进：本次先让 Gazebo 显示实验楼与机器人、完成 Fast Planner 迁入并提供二维地图导航入口；实验楼二维地图由用户自行建立，实际导航由用户随后测试，3D 点云版本以后再考虑。

主仓库已有三工作区，以及 LIO/ICP 的独立 3D 流程。该流程的 camera_init、/lio/nav_odom、关闭轮速 TF 的 model_nav.sdf 不适用于本次二维 AMCL 路线，因此保持原文件不动，新增并行二维入口。实验楼 world 已内嵌建筑和地面，不需要再插入 model.sdf。

## Goals / Non-Goals

**Goals:** 主仓库独立构建加载插件；沿用统一仿真入口选择二维/三维机器人，所有模式默认实验楼；独立二维导航入口接受用户提供的 map YAML；保留 DWB 回退与旧三维建图导航流程。

**Non-Goals:** 本次不采集任何地图，不安装/集成新的 SLAM 建图流程，不修改 LIO/ICP，不建立场景地图 profile/hash 系统，不执行实验楼导航闭环、停车时延、RTF 或性能基准验收，不改写控制/优化算法。

## Decisions

### 1. 迁入原有核心与插件，保持单一主维护仓库

在 `tb3_nav_ws/src/fast_planner_nav2` 与 `tb3_nav_ws/src/third_party/Fast-Planner-2D-ROS2` 放置已修复源码、原有测试和来源说明；不复制旧 .git、build/install/log。记录上游 `9ed668e9d7f47292b7256a1ee539f0164bf664f0` 及已有补丁，保留许可证。不修改外部 `~/turtlebot3_ws`。

二维与三维共用导航工作区，不代表二维运行依赖 LIO；构建文档提供只构建核心/插件及二维入口所需包的命令，避免必须编译或启动 FAST_LIO 才能测试本次迁移。保留原完整构建脚本的三工作区用途。替代方案中的外部软链接或直接 source 旧 install 会产生隐式依赖，故不采用。

### 2. 统一仿真入口选择机器人，二维导航单独启动

修改现有 `turtlebot3_ws/start-world.sh` 及关联仿真 launch，不新增 start-world-2d.sh。沿用现有位置参数和额外 ROS launch 参数的形式：

```bash
./turtlebot3_ws/start-world.sh            # 默认 2d + shiyanlou
./turtlebot3_ws/start-world.sh 2d         # 二维机器人 + shiyanlou
./turtlebot3_ws/start-world.sh 3d         # 三维机器人 + shiyanlou
./turtlebot3_ws/start-world.sh 3d-light   # 保留轻量三维变体
./turtlebot3_ws/start-world.sh 3d-nav     # 保留关闭轮速 TF 的三维变体
./turtlebot3_ws/start-world.sh 2d scene:=turtlebot3_world  # 显式回退场景
```

2d/3d 是主要用户选择，3d-light/3d-nav 保持现有兼容，不删除或重定义。所有模式及项目自有仿真 launch 的默认世界统一为 shiyanlou；scene:=turtlebot3_world 对各模式可用，机器人选型与场景选择相互独立。

仅二维导航另增 `tb3_nav_ws/start-navigation-2d.sh`，对应 launch/config 位于工作区合适的包或现有 launch 目录。这些是实施时的代码修改，本轮仅规划。

二维链路：

```text
Gazebo shiyanlou + 标准二维 Burger
  /scan + /odom + odom → base_footprint
                  ↓
用户地图 → map server + AMCL（map → odom）
                  ↓
Nav2 → FollowPath（DWB / Fast Planner）→ 速度输出 → Gazebo
```

统一仿真入口支持 scene 回退以及 x/y/yaw。默认世界改变是有意的兼容性变化；三维模式仅改世界加载，保持原有模型和传感器配置，3d-nav 继续关闭轮速 TF，不把它当二维 AMCL 模型使用。LIO 建图、定位、导航入口不改造，也不会因选择 3d 而自动启动。启动前提醒/检查已有 Gazebo，切换模式须停止旧实例。新二维导航不得同时运行原 LIO/ICP，防止竞争 TF。

使用官方标准二维 Burger 的 SDF/URDF、二维扫描和轮速驱动，不使用关闭轮速 TF 的 3d-nav 模型。优先复用官方安装资源，不复制全部 TurtleBot3 仓库。用户之后可用自己的二维 SLAM 工具建图，本次只提供标准话题接口。

### 3. 直接加载 world，场景与地图解耦

加载现有 `gazebo_building_editor_models/shiyanlou/shiyanlou.world`，路径相对仓库或安装资源解析。保留用户资源，必要时设置本地模型搜索路径。world 已包含建筑，不能重复 spawn 建筑。Gazebo ROS 插件由标准 gzserver launch 加载。检查编辑器保存状态及出生位置，使机器人在地面空闲位置可见；具体默认出生坐标在实施时确认。

场景启动完全不检查地图。相比初稿的集中 profile、PCD 关联、hash 校验和自动转换，直接 world 参数即可满足当前需求，避免额外配置系统。

### 4. 二维导航接收 map，复用原控制输出配置

新导航入口要求显式 `map:=/path/to/map.yaml`；省略或文件不可用时给出错误，不回退旧地图。只做 Nav2 所需的 YAML/图像可用性检查，不证明地图与实际世界匹配，不要求元信息。地图正确性与初始位姿由用户在后续导航测试中确认。

使用 Humble AMCL/map server/navigation launch；支持 `controller_mode:=dwb|fast_planner`，默认 dwb，非法值报错。复用旧二维 bringup 的控制器配置和可用启动逻辑，拆开场景启动与导航启动，不照搬旧世界硬编码，也不连带启动第二个 Gazebo。

Fast Planner 模式保留旧配置的 10 Hz Controller、80 ms 预算/15 ms 最终检查预留、0.22 m/s 与 1.0 rad/s 限速、20 Hz CLOSED_LOOP smoother、/odom 反馈及已有反应时间/停车包络配置；参数成组迁入，不能只降低检查门槛。DWB 使用既有回退配置，不在本次要求两模式性能等价或统一输出链。检查实际命令话题连接与仿真兼容，保留原有安全处理。

不修改现有 nav2_3d.yaml 的 0.50 m/s 限速或 LIO 输出链；初稿关于统一改造原 3D 参数的决策取消。替代方案是重构现有 LIO 导航入口，但会把二维迁移耦合到不需要的 ICP 地图与定位门禁，故不采用。

### 5. 最小验证与完成边界

本次需要：
- 核心/插件及二维入口所需包在主仓库构建成功，包前缀不来自旧工作区。
- 用已有插件 lifecycle 测试或最小加载方式确认插件可发现、配置和加载；必要的静态参数/launch 检查确认 map、模式、TF 和话题接口。
- 分别以 2d、3d 启动，Gazebo GUI 实际显示实验楼与所选机器人；二维简单确认 /scan、/odom，三维确认原有传感器配置未被替换，不要求建图。3d-light/3d-nav 只检查模式到模型及默认 world 的参数映射，不扩展成完整测试矩阵。GUI 无法观察时如实记录待用户确认，不把 headless 进程存在当作视觉验收。
- README 提供先启动实验楼、用户自行建图、提供 map 后选择控制器的操作说明。

已有核心和控制测试随源码保留，不删除、不禁用；不新增大规模测试集，不把历史全部回归及上游 lint 作为本次强制完成门槛。若实施时确需改变算法，应先重新讨论范围。

没有实验楼地图不会阻塞场景/插件迁移交付。后续用户导航测试不列为本次未完成任务，也不虚称已通过。取消初稿的停车 0.5/1.0 s、RTF、计算耗时分布、地图覆盖和两模式直行转弯等交付要求；这只缩小验证范围，不移除已有控制器安全逻辑。

## Risks / Trade-offs

- [只通过加载并不证明导航效果] → 交付区分“场景/插件就绪”和“导航由用户后续验证”。
- [二维 AMCL 与原 LIO 同时运行导致 TF 冲突] → 入口独立、文档明确不能并行运行。
- [Gazebo 资源或出生点错误] → 使用实际 GUI 检查，不修改或覆盖用户原始建筑文件。
- [Conda 或外部 overlay 污染] → 使用系统 Humble/Python 和本仓库构建产物，检查包前缀。
- [二维地图与世界不匹配] → 不静默选择旧地图，用户显式传入并在后续测试中设置 AMCL 初始位姿。
- [三维模式默认新世界但旧 PCD 仍属官方场景] → 文档明确此次只验证三维机器人仿真显示；使用原三维地图流程时须显式回退官方世界，不能把旧地图用于实验楼。

## Migration Plan

1. 迁入核心、插件和来源记录，独立构建并检查插件加载。
2. 修改统一仿真入口，所有模式默认实验楼，确认 2d/3d 显示与原模型接口。
3. 新增 AMCL 二维地图导航入口，检查参数选择与资源路径。
4. 更新最简使用说明和本次完成记录，将建图/闭环测试明确交给用户。

回退：统一仿真入口显式选择官方世界，二维导航可选 DWB；原三维建图/定位/导航入口和外部工作区保持原状。本 change 不接入三维导航，后续如需集成另行规划。

## Open Questions

实验楼默认出生位置和朝向在实施时通过场景检查确定，不影响已确定的二维接口或验收范围。
