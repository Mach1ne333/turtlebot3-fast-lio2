# Cartographer 二维建图

替代此前推荐的 SLAM Toolbox 手动启动方式；仅二维，不改变三维 LIO 流程或 Nav2 控制器。依赖 `ros-humble-cartographer-ros`（本机已安装）。第一次安装后执行 `./scripts/build-2d.sh`。

## 安装位置与源码

当前使用 apt 安装的 `ros-humble-cartographer` / `ros-humble-cartographer-ros`，本仓库没有完整 Cartographer 算法源码：

- ROS 可执行程序：`/opt/ros/humble/lib/cartographer_ros/`。
- 编译库：`/opt/ros/humble/lib/libcartographer.a`、`libcartographer_ros.a`。
- 系统配置资源：`/opt/ros/humble/share/cartographer/`、`/opt/ros/humble/share/cartographer_ros/`。
- 开发头文件：`/opt/ros/humble/include/cartographer/`、`cartographer_ros/`；它们不等于完整源码树。
- 本项目的 Lua、launch 和 RViz 配置位于 `mapping_ws/cartographer/src/cartographer_bringup/`，独立于仿真包，详见下文。

如需修改算法，应另行获取匹配 Humble 发布版本的源码，在工作区中编译 overlay；不要直接修改 `/opt/ros/humble`。根目录 `mapping_ws/` 按算法分类；`mapping_ws/cartographer/` 本身是 colcon 工作区。

## 1. 仿真

先停止旧 Gazebo、SLAM/Cartographer、遥控、AMCL/Nav2。仓库根目录：

```bash
cd ~/turtlebot3-fast-lio2
./turtlebot3_ws/start-world.sh 2d scene:=shiyanlou_quarter
```

当前 quarter 的对应窄过道净宽已回调为 **quarter 加宽前原净宽的 2 倍**（不是 2.5 倍），门口 0.62 m，主要过道约 1.24 m；其他窄段约 1.18/1.31 m。默认场景仍是 third，故 quarter 要显式选择。所有地图必须与当前场景版本匹配。

EKF 默认保持开启；也可在启动仿真时加 `use_ekf:=false` 使用原始 odom，Cartographer 参数无需改。不要运行中切换里程计源。

## 2. 建图和遥控

另开终端：

```bash
cd ~/turtlebot3-fast-lio2
./mapping_ws/cartographer/start-mapping-2d.sh
```

自动打开 RViz，Fixed Frame 为 **map**，仅地图、激光、TF 和网格，没有旧 coverage 图层；`rviz:=false` 可关闭 RViz。该命令不启动第二个 Gazebo或 robot_state_publisher。

再开终端遥控：

```bash
cd ~/turtlebot3-fast-lio2
source scripts/humble-2d-env.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.4
```

低速移动，多角度观测，不要只沿长直走廊扫一遍；回到已扫区域形成闭环。灰色未知格不应强行填白，雷达射线和量程仍受真实配置限制。新算法不保证消除长廊退化、错误回环或时间/里程计问题。

入口会保守检查已有 SLAM/Cartographer、AMCL/Nav2 和 LIO 进程；导航入口也会拒绝与 SLAM 同时启动。不要绕过脚本同时发布多套 map→odom。

## 3. 保存 YAML/PGM

机器人停止，停止遥控，但保持 Cartographer/occupancy grid 节点运行。新终端：

```bash
cd ~/turtlebot3-fast-lio2
source scripts/humble-2d-env.bash
mkdir -p tb3_nav_ws/maps
```

可选：全新单轨迹会话默认 trajectory_id 为 0。若确实已结束建图，可先结束该轨迹触发最终优化；结束后不再接收该轨迹的新扫描，要继续建图则跳过此步：

```bash
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory '{trajectory_id: 0}'
# 等待最终优化及 /map 更新后保存，期间不要关闭 Cartographer：
```

保存使用新名字，避免覆盖以前的地图：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f "$PWD/tb3_nav_ws/maps/shiyanlou_quarter_2x_cartographer" \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
```

输出 `.yaml` 和 `.pgm`。不要求 PCD 或 pbstream；此 YAML/PGM 用于后续 AMCL，不是 Cartographer 可继续优化的完整轨迹状态。

保存后 Ctrl+C 停止 Cartographer（同时关闭它的 RViz），保留 Gazebo；停止其他建图/遥控节点，再启动：

```bash
./tb3_nav_ws/start-navigation-2d.sh \
  map:="$PWD/tb3_nav_ws/maps/shiyanlou_quarter_2x_cartographer.yaml" \
  controller_mode:=fast_planner
```

在导航 RViz 设置 2D Pose Estimate 后测试短目标；不能与 Cartographer 同时运行。

## 配置与 TF 分工

- Lua：`mapping_ws/cartographer/src/cartographer_bringup/config/cartographer_2d.lua`，使用系统 Cartographer 自带的 `map_builder.lua` / `trajectory_builder.lua` 默认项。
- Launch：同包 `launch/cartographer_2d.launch.py`。
- RViz：同包 `rviz/cartographer_2d.rviz`。
- 激光 `/scan`，有效量程 0.12–3.5 m；无回波射线长度 3.5 m；一帧扫描一次处理，单次累积 1 帧，地图分辨率 0.05 m。
- `use_odometry=true`，订阅 `/odom`；`use_imu_data=false`，不直接订阅 `/imu`，IMU 经已有 EKF 间接进入里程计。
- `tracking_frame=base_footprint`、`published_frame=odom`、`provide_odom_frame=false`：Cartographer 只发布 map→odom；EKF（关闭时则 Gazebo 底盘）继续拥有 odom→base_footprint。
- motion filter：0.5 s、0.1 m、5°；submap 45 帧，启用实时相关扫描匹配。是初始可运行参数，不是已完成全楼精度调优的结果。
- occupancy grid 独立节点发布 `/map`，周期配置为 1 s。

## 本次验证（2026-10-14）

7 项配置测试通过（既有 EKF 3 项 + Cartographer 4 项）。在独立 ROS domain 的 quarter 2 倍场景中：

- Cartographer 成功解析 Lua，收到 `/scan` + `/odom`，没有直接 IMU 订阅。
- 唯一地图发布者为 cartographer_occupancy_grid_node；`/odom` 唯一发布者为 EKF；TF 发布节点仅 Cartographer、EKF、robot_state_publisher，收到 map→odom→base_footprint。
- 低速直行、转动后仍收到地图，样本地图 149×65、5 cm 分辨率；未替用户完成全楼建图。
- 测试了 finish_trajectory（返回 code 0），随后 map_saver 成功保存临时 YAML/PGM 到 `/tmp`，未覆盖用户地图。
- 新 RViz 配置实测加载、显示地图成功；重复建图、建图时启动导航均明确拒绝。

仅确认启动、接口、局部运动和保存链路，不宣称比 SLAM Toolbox 更准确或已完成新场景闭环导航。测试进程已停止。
