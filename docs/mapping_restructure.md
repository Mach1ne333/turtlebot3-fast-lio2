# 建图目录迁移验证

以下是此前整理时的验证记录，命令和路径已同步当前目录/包名。本次父目录更名仅做静态检查，没有重新编译或重复运行本记录中的进程测试。

## 变更范围

根目录的 `mapping_ws/` 下面分别是 Cartographer 集成工作区和 FAST-LIO2 工作区。`fast_lio_bringup` 是本项目集成包的正式名称；Cartographer 集成包现为 `cartographer_bringup`。

- Cartographer 的 Lua、launch、RViz 和四项配置测试从仿真包迁出；仿真包不再声明 `cartographer_ros` 依赖。
- FAST-LIO 上游源码子模块搬到 `mapping_ws/fast_lio2/src/FAST_LIO/`，提交仍为 `a4743b0`，ikd-Tree 仍为 `e2e3f4e`。
- 新 `fast_lio_bringup` 独立安装项目配置、launch、RViz 和显示过滤器；`prepare.sh` 不再向上游注入这些资源。
- 算法补丁搬到 `mapping_ws/fast_lio2/patches/`；原有全部 PCD 随工作区一起搬迁，不统一或覆盖地图。
- 工作区路径和默认 PCD 路径集中在 `scripts/workspaces.bash`。导航通过 `pcd_map` 参数传入地图，桥接节点通过 `pcd_path` 参数接收，不再硬编码算法所在目录。
- 两个映射/建图入口、三维导航和定位入口支持从无关当前目录调用。用户脚本参数继续透传。

## 构建与自动测试

四个工作区中本项目所需的八个包重建成功：

```text
tb3_sim
cartographer_bringup
fast_lio
fast_lio_bringup
fast_planner
fast_planner_nav2
interface
localizer
```

检查所有包前缀均位于本仓库的新 install。FAST-LIO、Fast Planner 和 localizer 保留已有上游编译警告，未为消除警告修改算法。没有重新编译本地额外下载的全部官方 TurtleBot3 包，仿真使用既有系统依赖。

已通过：

- Cartographer 配置测试 4 项。
- 显示过滤测试 3 项、显示 launch/资源测试 3 项、FAST-LIO launch/配置测试 2 项。
- EKF 配置测试 3 项。
- Fast Planner 插件 lifecycle 测试 1 项（覆盖插件加载与失败隔离）。
- RViz 初始位姿到 ICP 的 fake-service 隔离测试，显式传入新的 PCD 路径。
- 从 `/tmp` 调用二维建图、三维建图、导航 LIO、LIO RViz、三维导航和定位入口的 `--show-args`。
- `prepare.sh` 迁移后重复运行成功，报告补丁已应用；不再复制 YAML/RViz 到上游。
- `build-2d.sh` 重复构建成功；公共 Humble 基线环境避免仿真包反向依赖导航或 LIO overlay。

共 15 项 Python 单元/配置检查、1 项插件测试，另加桥接隔离检查。`colcon test-result` 也会统计 CTest 包装测试，因此其计数可能大于用例数。

新 launch 测试模拟含空格的不同安装目录，确认资源通过 ROS 包索引解析，而不是访问源码目录。保留了用户迁移前 `min_z=-0.25` 的显示默认值和实际保存的 RViz 设置；修正旧测试仍期待 0.05 的过期预期，不改变算法过滤行为。

## 无 Gazebo 的实际进程启动检查

使用独立 `ROS_DOMAIN_ID=92`，从 `/tmp` 顺序启动、检查并停止：

1. Cartographer：实际读取新安装包中的 Lua，并从系统 Cartographer 解析默认 Lua；`use_sim_time=true`。
2. 已安装显示过滤器：`ros2 run fast_lio_bringup filter_map_display.py` 可以找到可执行程序，`min_z=-0.25` 参数生效。
3. FAST-LIO 建图：实际读取新集成配置，`common.lid_topic=/points_raw`、`publish.map_en=true`，`map_file_path` 为传入的临时绝对路径。
4. 三维整套导航入口：LIO 使用导航配置，`publish.map_en=false`；ICP 使用迁移后的 PCD；没有有效 ICP 时 Nav2 保持未启动。

没有 Gazebo、扫描输入或导航目标，不调用 map_save，不生成/覆盖 PCD。上游程序启动会打开 Log 文件，测试前备份并在测试后恢复原文件，测试日志另存临时备份。

检查所启动进程均已停止。此验证确认启动、参数与资源接线，不等于建图精度或闭环导航验收；没有替用户重新采集地图，也未重新做 GUI 建图/导航测试。

## 数据与 Git 完整性

- 对全部原有 PCD 做迁移前后 SHA-256 校验。
- 比较 FAST-LIO 子模块的迁移前后源码 diff，保持逐字节一致。
- 检查递归子模块状态、固定提交、prepare 幂等性、安装软链接，以及源码/新生成文件中旧工作区路径残留。
- Git 子模块使用标准 `git mv` 维护，只有子模块 gitlink 和 `.gitmodules` 被暂存用于迁移；其他用户改动未批量暂存，未创建提交。

临时备份及路径维护说明见 [工作区与路径](workspaces.md)。

## 可重复的主要命令

在仓库根目录的专用终端：

```bash
./scripts/build-2d.sh
source scripts/humble-2d-env.bash
(cd mapping_ws/cartographer && colcon test --packages-select cartographer_bringup)

# 三维包构建后，在同一终端额外加载 LIO 和导航 overlay：
source mapping_ws/fast_lio2/install/local_setup.bash
source tb3_nav_ws/install/local_setup.bash
export ROS_DOMAIN_ID=92
(cd mapping_ws/fast_lio2 && colcon test --packages-select fast_lio_bringup)
python3 -m unittest discover -s turtlebot3_ws/src/tb3_sim/test -v
(cd tb3_nav_ws && colcon test --packages-select fast_planner_nav2 \
  --ctest-args -R '^test_controller$')
python3 tb3_nav_ws/tools/test_initialpose_bridge.py "$TB3_LIO_MAP"
```

## 后续建图/导航职责分离（未编译）

- 导航 LIO 调试入口移到 `tb3_nav_ws/start-fast-lio2-nav.sh`，导航参数移到 `tb3_nav_ws/config/tb3_gazebo_nav.yaml`。
- 整套导航和调试入口共用 `tb3_nav_ws/launch/lio_odometry.launch.py`，直接启动已编译的 `fast_lio` 算法，不再依赖建图集成包的 launch/config。
- 导航关闭 PCD 保存，且不再把 ICP 使用的地图路径传给 LIO 作为保存目标；建图配置及已有地图不变。
- 对应源码级回归检查（不启动 ROS 节点）：`python3 -m unittest discover -s tb3_nav_ws/tools -p 'test_lio_odometry_launch.py' -v`。运行前先加载系统 Humble 环境。
- 本次不运行编译或仿真；上面的历史进程/构建验证不代表已经重新验证本次分离后的实际运行。

