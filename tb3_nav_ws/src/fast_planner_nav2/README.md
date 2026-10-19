# fast_planner_nav2

ROS 2 Humble `nav2_core::Controller` 插件：
`fast_planner_nav2::FastPlannerController`。

```text
Nav2 全局规划 → 局部路径截取 → 第三方 B-Spline 几何优化
            → 航向对齐/纯追踪/终点 yaw → 速度约束与碰撞预测
            → Controller Server → velocity_smoother → cmd_vel
```

- 不是官方 Fast-Planner 的完整移植或论文复现；上游版本/补丁见
  `../third_party/Fast-Planner-2D-ROS2/UPSTREAM.md`。
- 不使用上游样条时间或 `PathPoint.v` 作为执行速度，不提供 Smoother 插件。
- 保留 Nav2 全局规划、AMCL、行为树和恢复，替换的是 DWB 的 FollowPath。
- 目标平台仅为标准 Burger / Humble / Gazebo Classic。尚需人工导航验收，不适用于直接实机部署。

## 控制与安全边界

参数定义见 `include/fast_planner_nav2/control.hpp` 的 `Settings` 和 bringup 的
`../../config/2d/nav2_fast_planner.yaml`。参数在 configure 时读取；运行时速度限制使用
Nav2 `setSpeedLimit`（包括 Humble `0 == NO_SPEED_LIMIT`，不是零速停车）。

机器人几何取 costmap padded footprint 的外接圆。unknown 默认按障碍处理；
地图外区域不视为自由空间。ESDF 不重复膨胀，优化距离含栅格化误差；独立检查
原始 costmap 上的参考路径、起点连接和最终命令。最终检查同时使用差速运动采样
和保守圆形停止可达包络，覆盖反应时间内的方向/速度过渡及制动；因此可能比实际
必要空间更保守，宁可失败停车也不放宽碰撞判据。不预测移动障碍的未来运动。

单次同步计算默认 80 ms 预算，控制频率 10 Hz；输入点数/地图规模有上限。
steady clock 统计总耗时，ROS clock 用于控制间隔；暂停/回退的仿真时间导致失败。
时间预算不是硬实时抢占，返回过晚的结果被拒绝。DEBUG 日志包含成功计算耗时，
失败日志包含耗时和原因。同步计算若持续不能达标，应停止验收并重新讨论架构，
不要简单放大预算或复用旧轨迹。

规划/碰撞/TF/时间错误通过 Humble `PlannerException` 上报，不用普通零速度伪装失败。
取消由 Controller Server 处理，插件没有取消回调；新路径、停用和清理重置状态。
FollowPath 失败后标准 BT 可能旋转/后退恢复；这不等同于显式取消整个导航任务。

## 使用与验证

构建、启动和回退见主仓库 `README.md` 的“实验楼与二维 Fast Planner”。
本次仅检查迁移就绪；地图由用户建立，实验楼闭环导航效果留待用户测试。
包级测试为 `test_control` 和 `test_controller`，后者通过 pluginlib 加载插件并验证
生命周期、目标转向、无效输入、TF 失败、可控超时、ROS 时间跳变和连续任务隔离。
第三方核心库另有一个小型 `test_geometric_planner` 测试集。
