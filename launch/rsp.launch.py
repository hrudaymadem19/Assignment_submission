import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = "slam_robot_test"

    package_share = get_package_share_directory(package_name)

    robot_description_path = os.path.join(
        package_share,
        "urdf",
        "robot.urdf"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo simulation time"
    )

    robot_description = ParameterValue(
        Command([
            "xacro",
            " ",
            robot_description_path
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description
            }
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher
    ])
