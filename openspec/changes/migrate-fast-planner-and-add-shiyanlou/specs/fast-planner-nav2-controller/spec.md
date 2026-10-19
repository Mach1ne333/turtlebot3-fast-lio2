## Purpose

为 turtlebot3-fast-lio2 主仓库的二维 AMCL/Nav2 入口迁入可独立构建的 Fast Planner 局部控制能力，保留已有几何优化、速度、碰撞和失败边界。本次验证限于迁移就绪，不以用户后续的实验楼闭环导航测试为完成条件。

## ADDED Requirements

### Requirement: Independent reproducible controller
系统 SHALL 在本仓库准备依赖和构建后提供 `fast_planner_nav2::FastPlannerController`，不依赖外部旧工作区的源码或安装产物；SHALL 保留上游固定版本、补丁来源、许可证和迁入的功能回归测试。

#### Scenario: Clean workspace build
- **WHEN** 用户仅加载系统 Humble 及本仓库工作区并执行文档化构建
- **THEN** Controller 可被 Nav2 加载，核心和插件包解析到本仓库，外部 `~/turtlebot3_ws/install` 不参与解析

### Requirement: Geometric differential drive control
Controller SHALL 从 Nav2 路径、TF、速度反馈和局部二维 costmap 生成有限的前进线速度及角速度；SHALL 支持起步对齐、路径跟踪及目标朝向对齐，服从 GoalChecker 和 Nav2 速度限制。输出 SHALL 不使用第三方曲线时间或 `PathPoint.v` 作为执行时序，并限制线速度不超过 0.22 m/s、角速度绝对值不超过 1.0 rad/s。

#### Scenario: Two dimensional navigation inputs
- **WHEN** 有效全局路径和二维激光更新后的局部 costmap 可用
- **THEN** Controller 在 costmap 坐标系中生成安全参考并输出差速命令，横向线速度为零

#### Scenario: Goal orientation and speed limit
- **WHEN** 机器人进入目标位置容差或收到更低的速度上限
- **THEN** Controller 分别处理终点朝向或遵守新上限，不因迁移忽略 GoalChecker 或限速

### Requirement: Bounded safe computation and failure isolation
Controller SHALL 保留可配置的总计算预算、最终安全检查预留、机器人 footprint 保守包络、参考碰撞检查和实际命令停车包络检查。默认总预算 SHALL 为 80 ms，预留 SHALL 为 15 ms；过期结果 SHALL 被拒绝，不声称底层优化可硬实时抢占。未知空间 SHALL 按障碍处理。

#### Scenario: Invalid or unsafe result
- **WHEN** 输入退化、TF 不可用、地图过期、优化失败、参考或命令不安全，或计算结果超预算
- **THEN** Controller 不返回该非零命令或复用旧参考，而通过 Humble Nav2 的失败机制交由 Server 处理

#### Scenario: New task or lifecycle transition
- **WHEN** Controller 接收新计划、停用、清理或再次激活
- **THEN** 旧任务状态被隔离，取消任务仍由 Controller Server 负责，不增加插件私有速度发布器
