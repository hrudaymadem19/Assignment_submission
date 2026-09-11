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
