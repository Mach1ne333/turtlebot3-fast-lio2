# shiyanlou PCD 派生二维地图

## 文件与生成命令

- 输入：`mapping_ws/fast_lio2/maps/shiyanlou.pcd`，未压缩 binary PCD，301756 个有限点。
- 导航地图：`tb3_nav_ws/maps/shiyanlou_ground.yaml` 与 `shiyanlou_ground.pgm`。
- 预览：同目录 `shiyanlou_ground.png`；Nav2 使用 YAML/PGM，不使用此 PNG。
- 工具：本项目 `tb3_nav_ws/tools/pcd_to_ground_grid.py`（Python、NumPy、PyYAML），不是 Nav2 内置的 PCD 转图工具。

在仓库根目录执行：

```bash
/usr/bin/python3 tb3_nav_ws/tools/pcd_to_ground_grid.py \
  mapping_ws/fast_lio2/maps/shiyanlou.pcd \
  tb3_nav_ws/maps/shiyanlou_ground.yaml \
  --resolution 0.10
```

该工具会覆盖同名输出；本次生成前确认输出不存在，没有覆盖原地图。

## 本次结果

- 分辨率：0.10 m/格；尺寸：497 × 500 格，对应 49.7 × 50.0 m。
- YAML 左下角原点：约 `[-22.7, -27.0, 0.0]`；保持输入 PCD 的 XY 坐标，不重新居中或旋转点云。
- 自动估计地面 Z：`-0.041403889656066895 m`。
- 地面取点范围：约 `(-0.126404, 0.043596) m`。
- 障碍取点范围：约 `(0.158596, 0.858596) m`。
- 白色 254：6661 格（2.68%），地面观测支持的空闲格。
- 黑色 0：4446 格（1.79%），满足高度/点数条件的障碍格。
- 灰色 205：237393 格（95.53%），未知格，不是已确认通行域。

## 匹配场景与尝试启动

当前 `shiyanlou.pcd` 的 SHA-256 为 `d7dfffd4b919f1c116f13552fc83ff94a7a2e66d30bc3f83a91a8414d8dbb31d`，与迁移前备份记录的 `shiyanlou_quarter_replay_20261014_212630.pcd` 完全一致。因此应使用 `scene:=shiyanlou_quarter`，不是默认的全尺寸 `shiyanlou`；quarter 场景本身也必须与建图时的版本一致。

每个终端先在仓库根目录加载 `source scripts/humble-2d-env.bash`。

```bash
# 终端 1：停止旧仿真/建图后运行。
./turtlebot3_ws/start-world.sh 3d-nav scene:=shiyanlou_quarter

# 终端 2：自动启动 LIO + ICP；有效定位后才启动 Nav2。
./tb3_nav_ws/start-navigation.sh \
  pcd_map:="$PWD/mapping_ws/fast_lio2/maps/shiyanlou.pcd" \
  map:="$PWD/tb3_nav_ws/maps/shiyanlou_ground.yaml"

# 终端 3：PCD 预览、初始位姿和导航目标。
./tb3_nav_ws/start-nav3d-rviz.sh
```

RViz 中先使用 2D Pose Estimate，在三维 PCD 上给出机器人实际位置与朝向。等终端 2 确认定位有效、自动启动 Nav2 后，启用 `PCD-derived ground planning grid (optional)` 图层查看 `/map`。地图在门禁通过前尚未启动，属于预期行为。

发目标前，可在另一个已加载环境的终端执行 `ros2 param set /planner_server GridBased.allow_unknown false`，禁止规划进入未知区域（本次运行有效，重启恢复配置）。只在机器人所在的连续白色区域选择近距离目标，不要尝试穿越右侧密集黑点区。若无法规划，不要通过填白障碍、关闭碰撞检查或强行放开未知区域绕过。

这是有监督的仿真接线/局部运动测试，不是地图可用性验收；没有运行闭环导航验证。无需另开建图、独立 LIO 调试或独立定位脚本，避免重复节点与 TF。

## 算法

1. 读取 XYZ，移除非有限点。
2. 在 Z 的较低 40% 范围内用 2.5 cm 直方图寻找频数最高的高度，估计一个水平地面。
3. 按 XY 和分辨率划格。地面高度 ±8.5 cm 内的点作为地面证据；地面上方 20～90 cm 的点作为障碍证据。
4. 至少两个地面点的网格及其一格邻域标为空闲；至少两个障碍点的网格标为占据，障碍优先。其余保持未知。
5. 输出 PGM，按图像坐标翻转行；YAML 记录分辨率、原点和图像相对路径。Nav2 map_server 将其解释为 OccupancyGrid。

## 使用边界

这是简化投影的初始结果，不是导航质量验收。整体边界包含主体外的稀疏点，周围大量区域未知；图内也有未知、离散点。单一水平地面不能适应所有地面高度变化；地面偏高可能被排除，甚至落入障碍高度窗。地面不足、每格点数不足也会留下灰色。当前高度窗不代表机器人的完整碰撞体积，低矮障碍可能被忽略。

该工具不使用轨迹、传感器原点或射线清空模型，不能仅凭没有点就认定可通行。未自动填白、裁剪、校正点云或调整转换脚本。本次没有切换导航默认 PCD/YAML，没有启动仿真或编译。

当前导航配置的 `GridBased.allow_unknown` 为 `true`，规划器可能经过未知格；这不代表地图把未知格认定为安全。使用前需要核对点云水平性、障碍高度窗、可通行区和实际场景，并显式配套选择相同坐标系的 PCD 与二维图。只替换 PCD 不会自动切换规划图；现已支持 `start-navigation.sh pcd_map:=... map:=...`，必须成套传入。
