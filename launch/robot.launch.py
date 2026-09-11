import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = "slam_robot_test"

    package_share = get_package_share_directory(package_name)
    record_bag = LaunchConfiguration("record_bag")
    bag_output = LaunchConfiguration("bag_output")
    wall_enabled = LaunchConfiguration("wall_enabled")
    forward_speed = LaunchConfiguration("forward_speed")
    default_bag_output = os.path.join(
        "logs", f"assignment_bag_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": "-r -v 2 "
                       + os.path.join(
                           package_share,
                           "worlds",
                           "assignment_world.sdf"
                       )
        }.items()
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "launch",
                "rsp.launch.py"
            )
        )
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/lidar/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/world/assignment_world/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
        output="screen"
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "slam_robot",
            "-x", "1.0",
            "-y", "0.0",
            "-z", "0.0",
            "-Y", "0.0",
        ],
        output="screen"
    )

    rviz_config = os.path.join(
        package_share,
        "rviz",
        "slam_robot.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        respawn=True,
        respawn_delay=2.0,
        output="screen"
    )

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "core.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "wall_enabled": wall_enabled,
            "forward_speed": forward_speed,
        }.items(),
    )

    recorder = ExecuteProcess(
        condition=IfCondition(record_bag),
        cmd=[
            "ros2", "bag", "record", "--all",
            "--output", bag_output,
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "record_bag",
            default_value="true",
            description="Record every ROS topic to a rosbag2 recording",
        ),
        DeclareLaunchArgument(
            "bag_output",
            default_value=default_bag_output,
            description="rosbag2 output directory",
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
        gazebo_launch,
        rsp,
        bridge,
        spawn_robot,
        core,
        rviz,
        recorder,
    ])
