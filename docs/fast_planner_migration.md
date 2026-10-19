# 实验楼 / Fast Planner 迁移检查记录

Change：`migrate-fast-planner-and-add-shiyanlou`，2026-10-14。

**结论：场景/插件迁移就绪，实际导航待用户测试。** 实验楼地图由用户建立，本记录不证明 AMCL 定位或实验楼导航效果。

## 已检查

| 项目 | 方式及结果 |
| --- | --- |
| 源码导入 | 核心及插件共 92 个原始文件逐文件 SHA-256 对照；导入快照见 `MIGRATION_SHA256.json`。只在主仓库修改，未改旧工作区。 |
| 独立构建 | `./scripts/build-2d.sh` 成功；turtlebot3_lio_sim、fast_planner、fast_planner_nav2 均来自本仓库 install，不使用旧 overlay，不编译 FAST_LIO/ICP。核心保留若干历史编译警告。 |
| 插件加载 | 迁入的 `Controller.PluginLifecycleFailureAndNewTask` 通过，覆盖 pluginlib 加载/configure/activate/失败隔离；其中故意触发超时日志是预期行为，不是性能实测。 |
| 场景参数 | 2d、3d、3d-light、3d-nav × 实验楼/官方世界共八种参数解析成功。变体仍选择原 model_light/model_nav；未做变体完整 GUI 测试。 |
| 出生点 | 实验楼默认 (-2.0, -0.5, yaw=0)，按 world 墙盒计算最近墙面约 5.306 m；GUI 确认未与墙重叠。 |
| 二维 GUI | 实验楼建筑与标准 Burger 显示，spawn 成功；接收 `/scan`（base_scan、360 rays）、`/odom`（child base_footprint），TF 包含 odom→base_footprint。 |
| 三维 GUI | 实验楼建筑与 burger_lidar3d 显示，spawn 成功；接收 `/points_raw`（lidar_3d_link）、`/imu`（imu_link）。未运行 LIO/ICP。 |
| 二维导航输入 | DWB/Fast Planner 两模式解析成功；缺 map、无效模式、地图图像缺失均拒绝。入口 `--show-args` 可用，不启动仿真或 LIO。 |
| 控制配置 | 两份 YAML 与旧二维集成逐字节相同；Fast Planner 输出链校验保留。运行参数为 FastPlannerController、10 Hz、CLOSED_LOOP smoother、轮速 odom。 |
| 正常命令路由 | 使用临时空白地图做无目标启动检查：`controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel → turtlebot3_diff_drive`。标准 behavior_server 各恢复插件也直接发布 /cmd_vel，保持原 Humble 路由，不是额外启动的控制器。 |

启动检查使用 ROS_DOMAIN_ID=87 隔离；没有发送导航目标，没有移动机器人。临时空白地图仅用于检查节点和参数，**不是实验楼地图**；未设置 AMCL 初值，因此 AMCL 提示等待初始位姿、全局 costmap 等待 map TF 属于该检查边界，不能解读为定位成功。检查启动的进程随后已停止。

## GUI 图像

建筑较大，初始镜头中的机器人很小。使用 Gazebo 自身 camera follow 查看机器人，不修改建筑或模型。

- [二维建筑总览](images/shiyanlou-2d.png) / [二维机器人聚焦](images/shiyanlou-2d-follow.png)
- [三维建筑总览](images/shiyanlou-3d.png) / [三维机器人聚焦](images/shiyanlou-3d-follow.png)

二维图中的蓝色线条是官方激光模型的可视化，不是额外障碍物。

## 可重复的最小检查

仓库根目录、专用终端：

```bash
./scripts/build-2d.sh
source scripts/humble-2d-env.bash
export ROS_DOMAIN_ID=87
(cd tb3_nav_ws && colcon test --packages-select fast_planner_nav2 \
  --ctest-args -R '^test_controller$' --event-handlers console_direct+)
./tb3_nav_ws/start-navigation-2d.sh --show-args
./turtlebot3_ws/start-world.sh 2d
# Ctrl+C 停止后再运行：
./turtlebot3_ws/start-world.sh 3d
```

默认界面若看不清机器人，可在 Models 中双击机器人；也可在仿真运行时使用 Gazebo 自带 `gz camera -c gzclient_camera -f burger`（三维为 burger_lidar3d）聚焦。

## 未执行且不属于本次完成条件

- 实验楼地图采集、地图配准、AMCL 定位、真实导航目标及避障效果。
- 停车时延、RTF 或计算性能定量验收。
- 三维建图/ICP/Nav2 新场景集成。
- 完整历史核心/控制测试与上游全树 lint；已有测试文件仍保留。此次只运行指定插件 lifecycle 测试，不能声称其他测试或 cppcheck 已通过。

原 PCD/YAML/PGM 和 LIO 脚本未改；旧三维地图流程须用 `scene:=turtlebot3_world`。后续使用方法见根 README。
