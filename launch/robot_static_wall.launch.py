from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("slam_robot_test"),
                            "launch",
                            "robot.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "wall_enabled": "false",
                    "forward_speed": "0.2",
                }.items(),
            )
        ]
    )
