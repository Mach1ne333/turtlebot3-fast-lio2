# TurtleBot3 Burger 3D LiDAR (independent simulation)

This ROS 2 Humble / Gazebo Classic package does **not** change the official
`turtlebot3_gazebo` or `turtlebot3_description` files. It retains the stock
200 Hz IMU and differential drive, but replaces the 2D LDS with a 16-beam CPU
ray sensor (360 azimuth samples, +/-15 degrees elevation, 10 Hz, 0.15-20 m).
The optional `model_light.sdf` uses 240 horizontal samples at the same 16 beams
and 10 Hz to reduce Gazebo CPU work; it trades angular detail for throughput.

- SDF: `models/turtlebot3_burger_lidar3d/model.sdf`
- URDF/TF: `urdf/turtlebot3_burger_lidar3d.urdf` (`lidar_3d_link`)
- Launch: `launch/lidar3d_world.launch.py`
- ROS topics: `/points_raw` (`sensor_msgs/msg/PointCloud2`), `/imu`
  (`sensor_msgs/msg/Imu`), `/odom`, `/cmd_vel`; the old `/scan` is absent.

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select turtlebot3_lio_sim
source install/setup.bash
export GAZEBO_MODEL_DATABASE_URI=""
ros2 launch turtlebot3_lio_sim lidar3d_world.launch.py
```

Only run one Gazebo world on the default master at a time. To compare
performance, stop the old world first and use `~/turtlebot3_ws/start-world.sh
3d-light`; the unchanged high-detail mode is `~/turtlebot3_ws/start-world.sh
3d`. A further optional CPU reduction is `3d-light gui:=false` (no Gazebo GUI;
RViz still works). Check `gz stats -d 5` for Real Time Factor; 1.0 is real time.
For a quick headless check, add `gui:=false`. The matching RViz view is at `rviz/tb3_lidar3d.rviz`:

```bash
~/turtlebot3_ws/start-rviz.sh 3d
```

It uses Fixed Frame `odom`, colorizes height (Z), and displays the
`/points_raw` PointCloud2. For the separately tested 3D-PCD localization +
Nav2 experiment, use `~/turtlebot3_ws/start-world.sh 3d-nav` and follow
`~/tb3_nav3d_ws/README.md`: that variant disables Gazebo's wheel-odom TF so
Fast-LIO2 is the only robot pose source for navigation. The 3D navigation RViz
has its own `map` Fixed Frame and command `~/tb3_nav3d_ws/start-nav3d-rviz.sh`. The stock 2D RViz config remains available via
`~/turtlebot3_ws/start-rviz.sh 2d`.

**Fast-LIO2 limitation:** Runtime testing on this PC confirmed the ROS ray
sensor publishes a 3D **unorganized** cloud (`height=1`; observed 5641 points
in one frame) with only `x`, `y`, `z`, `intensity` fields, not a Velodyne/Ouster/
Livox packet stream. In particular it lacks `ring` and per-point `time` data
required by some Fast-LIO2 preprocessors. Inventing per-point timestamps from
a snapshot is not equivalent to a spinning sensor's motion distortion. A tested
ROS 2 branch of the official FAST_LIO repository and a **Gazebo-instantaneous
cloud** input mode are in `~/fast_lio2_ws` (see `README-TB3.md` there). This is
not a physically faithful spinning-LiDAR undistortion test. The original 2D
Nav2/Cartographer setup does not work unchanged with this model, because it no
longer publishes `/scan`.
