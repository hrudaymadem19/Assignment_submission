# Cloning the Github Repo

In your local system, create your ROS 2 workspace, set up the source directory, and clone the contents of the repository:

```bash
mkdir -p slam_robot_test/src
cd slam_robot_test/src

git clone https://github.com .

cd ..
colcon build --symlink-install
```

# slam_robot_test

`slam_robot_test` is a ROS 2 simulation package for testing localization and
mapping with a differential-drive robot. The robot operates in a Gazebo
corridor with a 16-channel 3D LiDAR and a 6-axis IMU. The package intentionally
adds odometry slip and IMU noise so that the EKF, LiDAR scan matching, mapping,
and diagnostic nodes can be evaluated under imperfect sensor conditions.

## Project overview

The normal simulation follows this pipeline:

1. Gazebo runs `assignment_world.sdf` and publishes the simulated robot
   sensors.
2. `ros_gz_bridge` converts Gazebo messages into ROS 2 topics.
3. The robot description is published from `urdf/robot.urdf`.
4. The processing nodes create noisy measurements, estimate the robot pose,
   match LiDAR scans, build a map, and publish diagnostics.
5. RViz displays the robot, LiDAR cloud, odometry, map, and world markers.
6. The nodes write CSV evidence files to `logs/`, and `plot_summary.py`
   creates a summary plot after the run.

The robot normally drives toward `x=18 m`. A moving wall changes the corridor
opening every 15 seconds. Between `x=10 m` and `x=15 m`, the odometry velocity
is scaled to 70 percent to simulate wheel slip. The EKF combines noisy
odometry, IMU data, and LiDAR pose measurements to reduce the resulting
localization error.

## Methods and results discussion


The simulation evaluates whether a mobile robot can maintain a useful pose
estimate and build a map when its sensors are imperfect and the environment
changes. The main challenges are:

- odometry becomes unreliable in the `x=10 m` to `x=15 m` slip zone;
- IMU angular velocity contains white noise and a slowly changing bias;
- LiDAR measurements can become weak or ambiguous in a long corridor; and
- the moving wall changes the observed geometry during the run.

### Chosen methods

The package uses several complementary methods rather than relying on a single
sensor:

1. **Sensor-noise simulation:** `noise_injector.py` applies a known odometry
   scale error, Gaussian velocity noise, and IMU gyro bias random walk. This
   creates a repeatable test case while preserving the Gazebo measurements as
   ground truth.
2. **Planar EKF fusion:** `ekf_estimator.py` estimates position, heading,
   velocity, angular velocity, and gyro bias. It predicts motion from noisy
   odometry and uses IMU angular velocity and LiDAR pose measurements as
   updates. This is appropriate because the robot moves on a mostly planar
   surface and the sensor errors are represented well by a probabilistic
   state estimate.
3. **LiDAR scan matching:** `scan_matcher.py` extracts forward and side-wall
   geometry from the 3D point cloud. It estimates forward position from wall
   range, lateral position from the corridor walls, and heading from the
   side-wall line fit. Measurement covariance is derived from the number and
   spread of detected points, and uncertainty is increased when the moving and
   far-wall hypotheses are difficult to distinguish.
4. **Ray-cast occupancy mapping:** `mapping_diagnostics.py` converts the
   horizontal LiDAR returns into free-space and occupied-space evidence using
   ray tracing. The resulting occupancy grid provides a live view of the
   discovered environment in RViz.
5. **Degeneracy diagnostics:** The scan information matrix is monitored using
   an information ratio. When the scan geometry does not constrain one
   direction strongly enough, the node publishes
   `LOCALIZATION_DEGENERACY_WARNING` and records it in `diagnostics.csv`.
6. **Closed-loop safety behavior:** `auto_drive.py` uses the filtered pose to
   drive toward the goal and uses the LiDAR cloud to stop for a nearby
   obstacle. `wall_controller.py` changes the wall pose through Gazebo's
   service, allowing the estimator and mapper to be tested against a dynamic
   scene.

### Interpreting the results

The CSV logs make it possible to compare the ground-truth trajectory, the
slipping/noisy odometry, and the EKF estimate. In a successful run, the noisy
odometry diverges more noticeably while the EKF remains closer to the
ground-truth path because it combines independent measurements. The effect of
the slip zone can be identified from the `scale` column in
`noisy_odom.csv`, where the value changes from `1.0` to `0.70` and then
returns to `1.0`.

The LiDAR and map outputs show how the corridor geometry is recovered while
the robot moves. The moving wall should appear at different positions in RViz
and in the world-marker output. During corridor-like or otherwise ambiguous
views, the diagnostics log may report a degeneracy warning; this is an
expected result indicating reduced geometric information, not necessarily a
software failure. The summary plot brings these observations together by
showing the ground-truth, noisy odometry, and EKF trajectories along with the
scan-information ratio and warning intervals.

The results should therefore be judged using both accuracy and diagnostics:
the EKF trajectory should reduce the error caused by the slip and IMU bias,
the map should follow the observed corridor and wall, and the warning events
should identify periods when LiDAR geometry is insufficient for confident
localization. The simulation is designed as an evaluation environment, so
results can vary slightly when noise parameters or the random seed are
changed.

## Repository layout

```text
slam_robot_test/
├── config/                  ROS-Gazebo bridge configuration
├── launch/                  ROS 2 launch files
├── rviz/                    RViz display configuration
├── scripts/                 Runtime ROS 2 Python nodes and analysis script
├── urdf/                   Robot model, sensors, and Gazebo plugins
├── worlds/                 Gazebo simulation world
├── CMakeLists.txt           Package installation rules
├── package.xml              ROS 2 dependencies and package metadata
└── README.md               Project documentation
```

## Launch files

| File | Purpose |
| --- | --- |
| `robot.launch.py` | Full simulation entry point. Starts Gazebo, the bridge, robot state publisher, RViz, the processing chain, and rosbag recording. |
| `core.launch.py` | Starts the assignment processing nodes. Use it when Gazebo, the bridge, and visualization are already running. |
| `robot_static_wall.launch.py` | Full simulation with the moving-wall controller disabled. Useful for a stationary-wall comparison run. |
| `robot_no_wall.launch.py` | Full simulation with the moving-wall controller disabled. |
| `rsp.launch.py` | Publishes the robot model and TF using `robot.urdf`. |

Useful launch arguments for `robot.launch.py` are:

```text
wall_enabled:=true|false       Enable or disable wall motion
forward_speed:=0.2             Robot forward speed in m/s
record_bag:=true|false         Enable or disable rosbag2 recording
bag_output:=logs/my_bag        Rosbag output directory
```

## Runtime scripts

| Script | Responsibility |
| --- | --- |
| `auto_drive.py` | Publishes forward velocity commands, waits for a valid EKF pose, stops at the goal or timeout, and stops when a close LiDAR obstacle is detected. |
| `wall_controller.py` | Moves the `shifting_wall` model through the Gazebo `set_pose` service and publishes its current position. |
| `noise_injector.py` | Converts true odometry and IMU data into noisy measurements. Applies the 70 percent odometry scale in the slip zone and logs ground truth, noisy odometry, and noisy IMU data. |
| `ekf_estimator.py` | Runs a planar extended Kalman filter using noisy odometry, IMU angular velocity, and LiDAR pose updates. Publishes filtered pose and odometry and broadcasts the map-to-odom correction. |
| `scan_matcher.py` | Extracts corridor geometry from consecutive LiDAR clouds and publishes a LiDAR-based pose measurement with data-dependent covariance. |
| `mapping_diagnostics.py` | Performs consecutive-scan matching, ray-casts LiDAR returns into a 2D occupancy grid, publishes scan information, and reports localization degeneracy warnings. |
| `world_visualizer.py` | Publishes RViz markers for the corridor, fixed walls, moving wall, and world context. |
| `plot_summary.py` | Offline utility that reads the CSV logs and writes a trajectory and scan-information summary plot. |

## Important topics

| Topic | Type | Description |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot velocity command. |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo odometry. |
| `/odom_noisy` | `nav_msgs/msg/Odometry` | Odometry after simulated slip and noise. |
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo IMU data. |
| `/imu_noisy` | `sensor_msgs/msg/Imu` | IMU data after simulated noise and bias. |
| `/lidar/points/points` | `sensor_msgs/msg/PointCloud2` | Bridged 3D LiDAR point cloud. |
| `/lidar/pose_measurement` | `geometry_msgs/msg/PoseWithCovarianceStamped` | LiDAR scan-matching pose constraint. |
| `/ekf/pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Filtered pose used by the drive controller. |
| `/ekf/odom` | `nav_msgs/msg/Odometry` | Filtered odometry. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Live ray-cast occupancy map. |
| `/localization/diagnostics` | `std_msgs/msg/String` | Localization status and degeneracy warnings. |

## Requirements

Install and source a ROS 2 Jazzy environment with the dependencies declared in
`package.xml`, including Gazebo Sim, `ros_gz_bridge`, `ros_gz_sim`, RViz 2,
rosbag2, NumPy, and Matplotlib.

The package must be built from a ROS 2 workspace. The commands below assume
the workspace root contains `src/slam_robot_test`.

## Build and run

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select slam_robot_test
source install/setup.bash
```

Start the simulation:

```bash
ros2 launch slam_robot_test robot.launch.py
```

The full launch records all ROS topics to a timestamped directory under
`logs/assignment_bag_*` by default. To disable rosbag recording:

```bash
ros2 launch slam_robot_test robot.launch.py record_bag:=false
```

Stop the simulation after the robot has passed through the slip zone and the
moving wall has changed position at least once. Then generate the offline
plot:

```bash
python3 install/slam_robot_test/lib/slam_robot_test/plot_summary.py \
  --log-dir logs
```

Expected files are:

```text
logs/ground_truth.csv
logs/noisy_odom.csv
logs/noisy_imu.csv
logs/ekf.csv
logs/diagnostics.csv
logs/summary_plot.png
logs/assignment_bag_*/       # rosbag2 recording, if enabled
```
