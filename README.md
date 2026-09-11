# `slam_robot_test` assignment implementation

The package provides a complete, ROS 2 Python processing chain:

* `wall_controller.py` calls the bridged
  `ros_gz_interfaces/srv/SetEntityPose` service at
  `/world/assignment_world/set_pose`.  The sliding wall starts hidden at
  `y=-6.0 m`, moves toward `y=0.0 m` by `1.5 m` every `15 s` of simulation
  time, then reverses and repeats continuously.
* `auto_drive.py` only publishes a slow forward `/cmd_vel` command. It does
  not subscribe to LiDAR, IMU, odometry, EKF, or map topics and does not stop
  for obstacles. Those measurements are handled by the noise injector, EKF,
  mapping, and diagnostics nodes required by the assignment. The default speed
  is `0.2 m/s`, with a 360-second safety timeout so the robot can traverse
  the `10 <= Position_X <= 15` slip zone and observe multiple wall updates.
* `noise_injector.py` publishes `/odom_noisy` and `/imu_noisy`.  Odometry
  velocity is scaled to `0.70` for `10 <= Position_X <= 15`, with Gaussian
  noise.  Gyro measurements include a per-axis random walk and white noise.
* `ekf_estimator.py` publishes `/ekf/pose_with_covariance` and `/ekf/odom`.
* `mapping_diagnostics.py` performs independent consecutive-scan LiDAR
  matching and publishes `/lidar/pose_measurement` with covariance. It uses
  the fused EKF pose only to place the live ray-cast display map; that display
  input is not reused to generate the LiDAR measurement, so the LiDAR
  constraint path is not circular.

`robot.launch.py` starts Gazebo, the bridge, robot description, RViz, and the
processing chain.  RViz is restarted if its GUI/rendering process crashes; its
failure does not stop Gazebo or the ROS-Gazebo bridge.  Motion readiness depends
on data-producing sensors and localization/mapping nodes, not RViz-only
markers.  Use `core.launch.py` alone when Gazebo and visualization are already
running.  Nodes write CSV files to `logs/`; after a run,
`plot_summary.py --log-dir logs` creates `logs/summary_plot.png`.

The LiDAR is configured as the assignment's 16-channel Gazebo GPU sensor and
bridged as `sensor_msgs/msg/PointCloud2` on `/lidar/points/points`. Gazebo also
advertises `/lidar/points` as its LaserScan output. RViz displays the
full cloud, while the mapping diagnostics node uses the horizontal slice of
that same 3-D cloud for ray-cast free-space updates, scan-matching
information, and `LOCALIZATION_DEGENERACY_WARNING` events.

For a complete evidence run:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
rm -f logs/*.csv logs/summary_plot.png
ros2 launch slam_robot_test robot.launch.py
# Leave the simulation running through the slip-zone and moving-wall response,
# or until the 360-second safety timeout, then stop it.
python3 install/slam_robot_test/lib/slam_robot_test/plot_summary.py --log-dir logs
```

The evidence files are `logs/ground_truth.csv`, `logs/noisy_odom.csv`,
`logs/noisy_imu.csv`, `logs/ekf.csv`, `logs/diagnostics.csv`, and
`logs/summary_plot.png`. The diagnostics CSV includes the exact warning text
in `warning_message` whenever `LOCALIZATION_DEGENERACY_WARNING` is emitted.

RViz starts focused on the full corridor with `odom` as its fixed frame, so
the robot and raw Gazebo odometry follow the physical simulation frame. The
live occupancy grid on `/map` and the labeled, translucent `GHOST OBSTACLE`
marker are transformed through the EKF's `map->odom` correction.

The RViz `LiDAR` cloud shows returned points and `LiDAR Scan Rays` shows the
horizontal scan beams, including free space before each return. Raw `/odom`
and filtered `/ekf/odom` are displayed separately. `robot.launch.py` also
records every ROS topic by default to `logs/assignment_bag`; disable it with
`record_bag:=false` or choose another directory with `bag_output:=...`.
