# 二维 IMU + 底盘里程计 EKF

仅针对 `start-world.sh 2d`，不改变三维模型、LIO、世界文件或用户设置的出生位置。

## 使用

已安装的依赖是 `ros-humble-robot-localization`。在专用终端、仓库根目录：

```bash
./scripts/build-2d.sh
./turtlebot3_ws/start-world.sh 2d
```

默认 `use_ekf:=auto` 为二维自动启用 EKF；三维不启用。要关闭，先停止仿真及建图/导航，再运行 `./turtlebot3_ws/start-world.sh 2d use_ekf:=false`。不能在运行中切换，不能另外启动第二个 EKF。

后续二维建图已切换为 Cartographer，地图保存、AMCL、DWB/Fast Planner 的路线不变。例如另开终端：

```bash
cd ~/turtlebot3-fast-lio2
source scripts/humble-2d-env.bash
./mapping_ws/cartographer/start-mapping-2d.sh
```

Cartographer 订阅 `/scan` 和融合后的 `/odom`，不再直接订阅 IMU；Nav2/速度平滑器继续使用 `/odom`。任何 SLAM（Cartographer、SLAM Toolbox）与 AMCL 都不能同时发布 map→odom。以下 SLAM Toolbox 测试记录是 EKF 初次接入时的历史证据；当前入口见 [Cartographer 说明](cartographer_2d.md)。

## 数据与坐标系

| 来源 | 话题 / TF | 用途 |
| --- | --- | --- |
| Gazebo 底盘 | `/wheel/odom` | 原始底盘里程计；EKF 只用 vx、vy，不用其 pose 或 yaw rate |
| Gazebo IMU | `/imu` | EKF 只用 Z 轴角速度，不用绝对 yaw 和加速度 |
| EKF | `/odom`、`odom → base_footprint` | 融合后局部里程计，唯一底盘 TF 来源 |
| robot_state_publisher | 底盘到雷达/IMU/轮子的 TF | 保持原 URDF |
| SLAM 或 AMCL | `map → odom` | 全局地图定位，与 EKF 分工 |

`two_d_mode: true`；差速底盘 vy≈0 作为非完整运动约束。避免把同一轮速积分得到的 pose、twist 重复融合。IMU 的绝对姿态通常对齐仿真世界，故此处不用它约束地图朝向；不使用加速度，以免引入重力、偏置及重复积分问题。

配置文件：`turtlebot3_ws/src/tb3_sim/config/ekf_2d.yaml`。参数使用上游 EKF 默认过程噪声；输入协方差使用传感器消息中的数值，未伪造标定结果。此仿真样本中底盘 vx/vy 方差为 1e-5，IMU gyro 方差为 4e-8。实际机器人需要重新标定和调参，仿真底盘输出也不能等同于具有真实打滑误差的编码器。

滤波器从局部零位姿附近开始积分，**不等于 Gazebo world 的出生坐标**；这不表示机器人被移动了。AMCL 初始位姿仍应在实际地图中指定。只改里程计融合不意味着地图几何已改变，也不能保证地图精度一定提高。

30 Hz 是配置的滤波频率，实际发布率也受 Gazebo /clock 和机器负载影响；未把频率配置当作定量性能证明。滤波器也不是传感器失效急停机制。

## 如何避免 TF 冲突

`launch/tb3_sim.launch.py` 在二维启用 EKF 时：

1. 读取官方/指定 SDF，在临时副本中仅将底盘 `publish_odom_tf` 设为 false，并增加 `odom:=/wheel/odom` remap。
2. 用该副本 spawn；spawn 退出或 launch 关闭时删除临时文件。不会修改 `/opt/ros/humble` 或复制整套官方模型。
3. 启动 `robot_localization/ekf_node`，将 `odometry/filtered` remap 成 `/odom`。

关闭 EKF 时直接使用原 SDF，恢复原始 `/odom` 与底盘 TF。若在 Gazebo 保存整个 world，仍需避免把机器人保存进去，以免与 launch 生成的机器人冲突。

## 检查

仿真运行时，另开同一 ROS domain 的专用终端：

```bash
source scripts/humble-2d-env.bash
ros2 node info /ekf_filter_node
ros2 topic echo /wheel/odom --once
ros2 topic echo /imu --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

`/odom` 发布者应只有 ekf_filter_node；turtlebot3_diff_drive 不应再发布 `/tf`。robot_state_publisher 仍会发布轮子等 TF，并不是底盘 TF 冲突。

单元检查：

```bash
source scripts/humble-2d-env.bash
(cd turtlebot3_ws && colcon build --packages-select tb3_sim \
  --symlink-install --cmake-args -DBUILD_TESTING=ON)
(cd turtlebot3_ws && colcon test --packages-select tb3_sim \
  --event-handlers console_direct+)
```

## 本次检查记录（2026-10-14）

- 三项配置测试：模型只改变底盘输出/不改官方文件、二维启用/回退/三维不启用、15 维融合字段选择。
- 无 GUI 仿真检查：raw `/wheel/odom` 与 IMU 均输入 EKF，融合输出有限且姿态四元数归一，odom/base_footprint frame 正确。
- /tf 发布节点仅 EKF 与 robot_state_publisher，Gazebo 底盘无 /tf 发布者；底盘 TF 仅由 EKF 提供。
- 原地转动 0.25 rad/s：样本融合值约 0.25006，IMU 约 0.24989 rad/s；低速直行 0.05 m/s：融合值约 0.05009 m/s。只是输入/输出连通性检查，不是定位精度评测。
- SLAM Toolbox 启动后收到 map（131×48 格）和 map→odom→base_footprint TF；未保存、覆盖用户地图，也未完成全楼建图或导航任务。
- `use_ekf:=false` 实测恢复 Gazebo `/odom` 和底盘 TF，无 EKF 节点或 `/wheel/odom` 输出。

测试使用独立 ROS domain，测试进程已停止。导航配置继续使用 `/odom`，没有改动算法、地图或既有用户配置。
