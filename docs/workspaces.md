# 工作区与路径约定

## 当前职责

| 工作区 | 职责 |
| --- | --- |
| `turtlebot3_ws/` | Gazebo、机器人模型、传感器和二维 EKF，不再存放 Cartographer 建图集成 |
| `mapping_ws/cartographer/` | `cartographer_bringup` 的 Lua、launch、RViz 和测试；算法使用系统 apt 版本 |
| `mapping_ws/fast_lio2/` | 上游 FAST-LIO2 源码子模块、只负责建图的 `fast_lio_bringup` 集成包、必要算法补丁及原有 PCD |
| `tb3_nav_ws/` | 二维 AMCL/Nav2、Fast Planner 插件、导航专用 LIO 里程计、三维 PCD ICP 定位与 Nav2 |

`mapping_ws/` 放在项目根目录，目前仍是分类父目录，不是额外的统一 colcon 工作区。目录增加 `_ws` 后缀并不改变构建边界：请在两个算法子目录分别构建，或使用根目录构建脚本，不要在父目录下混建所有包。

## 建图源码与集成分层

```text
mapping_ws/
├── cartographer/
│   ├── src/cartographer_bringup/
│   │   ├── config/
│   │   ├── launch/
│   │   ├── rviz/
│   │   └── test/
│   └── start-mapping-2d.sh
└── fast_lio2/
    ├── src/
    │   ├── FAST_LIO/             # 上游固定提交，仍是 Git 子模块
    │   └── fast_lio_bringup/     # 本项目集成，不修改算法包来配置传感器
    │       ├── config/
    │       ├── launch/
    │       ├── rviz/
    │       ├── scripts/         # 显示过滤器，安装为 ros2 run 可执行程序
    │       └── test/
    ├── patches/fast_lio2_gazebo.patch
    ├── maps/                    # 保留全部原 PCD 文件，不合并地图目录
    ├── start-fast-lio2.sh
    └── start-lio-rviz.sh
```

- 调整话题、参数、launch 或 RViz：优先修改对应 bringup 包。
- 修改 FAST-LIO 算法：修改上游子模块，并同步维护必要补丁；不把本项目 YAML/RViz 再复制进去。
- `scripts/prepare.sh` 只固定 FAST-LIO 提交、递归初始化子模块并幂等应用 Gazebo 补丁，不再注入配置资源，也不克隆官方 TurtleBot3 三个源码仓库。
- 仿真源码只有 `tb3_sim`；两套构建脚本在仿真工作区均只选择这个包。其官方网格以及可选的 2D 模型/示例世界仍来自系统安装的 `turtlebot3_gazebo` 等 Humble 包。
- Cartographer 的完整算法源码不在仓库中；安装位置见 [二维建图](cartographer_2d.md#安装位置与源码)。未来需要修改它时，再获取兼容 Humble 的源码加入其工作区。
- `mapping_ws/` 的项目集成只负责建图。FAST-LIO 算法源码只维护一份；导航复用其已编译的 `fast_lio` 包，不复用建图 bringup。
- 导航专用的 `start-fast-lio2-nav.sh`、`config/tb3_gazebo_nav.yaml` 和 `launch/lio_odometry.launch.py` 全部放在 `tb3_nav_ws/`；整套导航也包含这个导航 launch，不再调用 `fast_lio_bringup/mapping.launch.py`。
- 导航配置关闭地图发布、累积路径发布和 PCD 保存，不向 LIO 传递已有地图的保存路径；已有 PCD 仅交给 ICP 定位使用。LIO 内部仍维护匹配所需的局部点云，这是里程计算法的组成部分。

## 使用入口

```bash
# 二维：构建仿真 + Cartographer 集成 + Fast Planner，不编译 LIO/ICP
./scripts/build-2d.sh
./turtlebot3_ws/start-world.sh 2d
# 另开终端：
./mapping_ws/cartographer/start-mapping-2d.sh

# 完整四工作区构建：
./scripts/prepare.sh
./scripts/build.sh

# 三维建图：先单独启动对应仿真
./mapping_ws/fast_lio2/start-fast-lio2.sh
./mapping_ws/fast_lio2/start-lio-rviz.sh
```

停止建图后才能切换导航流程；上述不是应同时运行的一组命令。三维示例地图只匹配官方世界，详见根 README。`scripts/smoke-test.sh` 固定使用 `scene:=turtlebot3_world` 配对这些地图，不验证默认实验楼或新坡道。

脚本回归测试使用 Git/colcon/ROS 命令替身，不下载源码、不编译、不启动 Gazebo：

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover -s scripts/tests -v
```

## 路径维护规则

### 1. 跨工作区路径集中定义

`scripts/workspaces.bash` 从自身位置定位仓库，提供：

```bash
TB3_PROJECT_ROOT
TB3_SIM_WS
TB3_CARTOGRAPHER_WS
TB3_LIO_WS
TB3_LIO_MAP
TB3_NAV_WS
```

公共构建、二维环境、导航和 relocalize 入口使用这些变量。例如在 `scripts/` 下：

```bash
source "$(dirname "${BASH_SOURCE[0]}")/workspaces.bash"
(cd "$TB3_NAV_WS" && colcon build --packages-select fast_planner_nav2)
```

以后工作区改名，公共脚本的映射集中维护；用户文档中的命令仍须同步。配置助手不是让生成的安装目录自动可搬迁。

`scripts/humble-base-env.bash` 统一清理旧 overlay/Conda 变量并加载系统 Humble。构建脚本从这个干净基线开始，按顺序追加依赖，避免重复构建时把导航工作区反向记录成仿真的 underlay；`humble-2d-env.bash` 则在基线上加载仿真、Cartographer 和导航的运行环境。

### 2. 包内资源使用 ROS 包索引

```python
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
share = Path(get_package_share_directory('fast_lio_bringup'))
config = share / 'config/tb3_gazebo_instant.yaml'
```

两个新 bringup 包通过 CMake 把 config/launch/rviz 安装到 `share/${PROJECT_NAME}`。显示过滤器安装到 `lib/${PROJECT_NAME}`，用 `ros2 run fast_lio_bringup filter_map_display.py` 调用。运行时不再访问 `src/FAST_LIO/config` 或猜测邻接工作区的层级。

尚未打包的导航工作区工具继续从 `__file__` 定位自身资源，例如 `tools/make_nav_config.py`；不能将调用者的 `Path.cwd()` 或 `$PWD` 当成资源根目录。

### 3. 外部地图通过参数传递

- 默认 PCD 路径由 `TB3_LIO_MAP` 唯一定义为算法工作区的 `maps/tb3_fast_lio2.pcd`。
- FAST-LIO 启动脚本传入绝对 `map_file_path`；直接调用 `ros2 launch fast_lio_bringup mapping.launch.py` 时必须显式提供该参数，并先创建父目录。
- 三维导航/定位脚本传入 `pcd_map`，仅用于 ICP 与初始位姿桥接器，不作为导航 LIO 的地图保存路径。Python 节点不再硬编码邻接算法目录。
- 二维导航仍显式接受用户 `map:=...yaml`；地图 YAML 的 `image` 优先使用相对 YAML 的路径。

例如自定义三维建图输出，避免覆盖已有地图：

```bash
./mapping_ws/fast_lio2/start-fast-lio2.sh map_file_path:=/绝对路径/新地图.pcd
```

自定义路径的父目录须事先创建。重新定位服务仍支持原来的 `PCD_MAP` 环境变量。

### 4. 改名/搬迁后重新生成产物

CMake 缓存、colcon overlay 链以及 symlink-install 可能记录绝对路径。搬迁后应停止相关节点，在干净终端备份并重新生成受影响的 build/install/log，再检查包前缀、资源加载和插件，不能仅替换缓存文本或保留旧名软链接。

本次迁移保留 FAST-LIO 和 ikd-Tree 的固定提交及已有源码补丁；Git 子模块的工作区路径已更新，逻辑名称改为稳定的 `FAST_LIO`。本机 Git 内部元数据目录可能仍沿用历史名称，它不是源码路径，也不应手工批量替换。

### 本次父目录更名检查（未编译）

- 工作区助手、Git 子模块路径/索引/嵌套 worktree 元数据、PCD 忽略规则和文档示例均已改为 `mapping_ws/`。
- 算法包名、`mapping.launch.py` 文件名和 YAML 中的 `mapping` 参数命名空间与目录名无关，保持不变。
- 仅执行静态语法、包发现、路径、子模块与数据完整性检查；没有运行编译或启动/停止现有仿真。
- `mapping_ws` 下目前没有 build/install/log；`tb3_nav_ws` 的旧构建/安装文件仍含旧父目录路径。重新构建前，请停止相关节点，在专用终端备份/清理受影响的 build/install，再从仓库根目录运行 `./scripts/build.sh`。这些旧产物本次未删除，也没有手工替换。
- 本次备份为 `/tmp/tb3-mapping-ws-rename.YUSTRw/`。子模块路径更新通过 `git mv` 暂存，未创建提交；其他改动未批量暂存。

## 备份与变更边界

- 先前导航工作区改名的临时备份：`/tmp/tb3-workspace-rename.x8y95j/`。
- 本次建图整理的临时备份：`/tmp/tb3-mapping-restructure.fEYACr/`，包含原集成文件、diff、全部 PCD 校验值和受影响的构建产物；不是项目运行依赖。
- 发现上游目录中的 RViz 文件有用户后续保存的视角/线型设置，已将实际使用的版本迁到新集成包，同时备份旧模板。保留当前 `min_z=-0.25` launch 默认值，不借迁移改为历史测试中的 0.05。
- 仿真模型、场景、EKF 和算法数学实现不改；地图只搬迁不重建，不修改二维/三维导航算法或定位门禁。
- 旧入口不保留软链接；已有终端须重开或重新 source 二维环境。

本次检查结果见 [建图目录迁移验证](mapping_restructure.md)。
