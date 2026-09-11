# `slam_robot_test` assignment implementation

For a complete run:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
rm -f logs/*.csv logs/summary_plot.png
ros2 launch slam_robot_test robot.launch.py
# Leave the simulation running through the slip-zone and moving-wall response,
# or until the 360-second safety timeout, then stop it.
python3 install/slam_robot_test/lib/slam_robot_test/plot_summary.py --log-dir logs
```


The RViz `LiDAR` cloud shows returned points and `LiDAR Scan Rays` shows the
horizontal scan beams, including free space before each return. Raw `/odom`
and filtered `/ekf/odom` are displayed separately. `robot.launch.py` also
records every ROS topic by default to `logs/assignment_bag`; disable it with
`record_bag:=false` or choose another directory with `bag_output:=...`.
