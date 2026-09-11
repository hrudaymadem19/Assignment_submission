import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    output_dir = LaunchConfiguration("output_dir")
    wall_enabled = LaunchConfiguration("wall_enabled")
    forward_speed = LaunchConfiguration("forward_speed")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time",
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="logs",
                description="Directory for CSV diagnostics",
            ),
            DeclareLaunchArgument(
                "wall_enabled",
                default_value="true",
                description="Start the moving-wall controller",
            ),
            DeclareLaunchArgument(
                "forward_speed",
                default_value="0.2",
                description="Autodrive forward speed in m/s",
            ),
            Node(
                package="slam_robot_test",
                executable="wall_controller.py",
                name="shifting_wall_controller",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(wall_enabled),
                output="screen",
            ),
            Node(
                package="slam_robot_test",
                executable="auto_drive.py",
                name="slam_auto_drive",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "forward_speed": forward_speed,
                        "start_delay_sec": 3.0,
                        "max_run_duration_sec": 360.0,
                    }
                ],
                output="screen",
            ),
            Node(
                package="slam_robot_test",
                executable="noise_injector.py",
                name="slam_noise_injector",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "output_dir": output_dir,
                    }
                ],
                output="screen",
            ),
            Node(
                package="slam_robot_test",
                executable="scan_matcher.py",
                name="simulated_scan_matcher",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="slam_robot_test",
                executable="ekf_estimator.py",
                name="slam_planar_ekf",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "output_dir": output_dir,
                    }
                ],
                output="screen",
            ),
            Node(
                package="slam_robot_test",
                executable="mapping_diagnostics.py",
                name="slam_mapping_diagnostics",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "output_dir": output_dir,
                    }
                ],
                output="screen",
            ),
            Node(
                package="slam_robot_test",
                executable="world_visualizer.py",
                name="slam_world_visualizer",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
