# TurtleBot3 Burger simulation (2D / 3D)

ROS 2 Humble / Gazebo Classic. The historical `lidar3d_world.launch.py` name
remains compatible; it now shares world selection for all robot modes.
It does not modify official TurtleBot3 files.

From the **repository root**:

```bash
./scripts/build-2d.sh
./turtlebot3_ws/start-world.sh 2d  # default robot, default world: shiyanlou_third
# Stop the old Gazebo before selecting another mode:
./turtlebot3_ws/start-world.sh 3d
./turtlebot3_ws/start-world.sh 3d-light
./turtlebot3_ws/start-world.sh 3d-nav
```

All modes support `scene:=turtlebot3_world` to explicitly restore the official
world, `gui:=false`, and `x_pose`, `y_pose`, `yaw_pose`. Direct launch defaults
to the historical **3d** robot, but also defaults to **shiyanlou_third**.
Use `scene:=shiyanlou` for the original full-size building, or
`scene:=shiyanlou_quarter` for the 1/4 variant: its narrow passages and doorway
are now widened 2x relative to its OWN original clear widths (reduced from
2.5x; overall 18.00 x 10.42 m, unchanged 0.0375 m wall thickness). The default third-size
world now has its narrow corridors and doorway widened to 2.5 times their
previous clear widths (not a uniform building scale). Overall footprint is
about 25.43 x 14.71 m; wall height/thickness remain 2.5/0.05 m. Rebuild maps;
stop and restart Gazebo to load the changed world. The pre-widening world is
preserved in the building resource's `backups/` directory.

- **2d**: official Burger SDF/URDF, `/scan`, `/imu`; default EKF consumes
  `/wheel/odom` + `/imu`, publishes fused `/odom` and odom→base_footprint TF.
  A temporary SDF only disables wheel TF and remaps raw odometry; the official
  file is untouched. `use_ekf:=false` restores raw `/odom` and wheel TF.
- **3d**: independent SDF/URDF, 16×360 CPU ray sensor, ±15° elevation,
  10 Hz, 0.15–20 m; stock 200 Hz IMU and differential drive retained.
- **3d-light**: original 16×240 lower-density ray model.
- **3d-nav**: original model with wheel-odom TF disabled, intended for the
  separate LIO pose source, **not** the 2D AMCL navigation entry.

3D topics remain `/points_raw` (`lidar_3d_link`), `/imu`, `/odom`, `/cmd_vel`;
3D models do not publish `/scan`. This migration changes scene selection,
not these 3D models or sensor semantics. `use_ekf:=auto` (default) enables the
filter only for 2d; explicitly enabling it for 3d is rejected. No robot mode
starts mapping/navigation automatically. EKF parameters: `config/ekf_2d.yaml`.

## 2D Cartographer mapping

With a separately running 2d simulator, run from the repository root:

```bash
./mapping_ws/cartographer/start-mapping-2d.sh  # rviz:=false disables its clean map-frame RViz
```

Uses `/scan` + `/odom`, with a 3.5 m laser range and 5 cm map resolution.
Cartographer owns only map→odom; existing EKF/diff-drive retains odom→base_footprint.
It does not directly consume IMU data (the EKF already uses it). Stop SLAM
Toolbox, AMCL/Nav2 and other localization before starting; stop mapping before
navigation. Save via nav2_map_server while mapping is still running. See
`mapping_ws/cartographer/src/cartographer_bringup/config/cartographer_2d.lua`
and the root `docs/cartographer_2d.md`. Cartographer launch/config/RViz and its
tests belong to that separate bringup package, not this simulator package.

The ray sensor produces instantaneous XYZI clouds, not a physically faithful
spinning LiDAR stream with `ring` and per-point time. This is not a validation
of real-sensor motion undistortion. Original LIO/PCD navigation instructions
and the new 2D AMCL entry are in the root README. Existing PCD maps belong to
the official world: explicitly select that world when using the old 3D flow.
