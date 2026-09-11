#!/usr/bin/env python3
"""Publish a steady forward command for the assignment experiment."""

import math

import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


class AutoDrive(Node):
    def __init__(self):
        super().__init__("slam_auto_drive")
        self.declare_parameter("forward_speed", 0.05)
        self.declare_parameter("goal_x", 18.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_tolerance", 0.20)
        self.declare_parameter("start_delay_sec", 3.0)
        self.declare_parameter("max_run_duration_sec", 360.0)
        self.declare_parameter("obstacle_stop_distance", 0.75)
        self.declare_parameter("obstacle_front_half_angle_deg", 18.0)
        self.declare_parameter("lidar_height_tolerance", 0.12)

        self._speed = float(self.get_parameter("forward_speed").value)
        self._goal_x = float(self.get_parameter("goal_x").value)
        self._goal_y = float(self.get_parameter("goal_y").value)
        self._goal_tolerance = float(
            self.get_parameter("goal_tolerance").value
        )
        self._start_delay = float(self.get_parameter("start_delay_sec").value)
        self._max_run_duration = float(
            self.get_parameter("max_run_duration_sec").value
        )
        self._obstacle_stop_distance = float(
            self.get_parameter("obstacle_stop_distance").value
        )
        self._front_half_angle = math.radians(
            float(self.get_parameter("obstacle_front_half_angle_deg").value)
        )
        self._lidar_height_tolerance = float(
            self.get_parameter("lidar_height_tolerance").value
        )
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._diagnostic_pub = self.create_publisher(
            String, "/autodrive/diagnostics", 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/ekf/pose_with_covariance",
            self._pose_callback,
            10,
        )
        self.create_subscription(
            PointCloud2,
            "/lidar/points/points",
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self._start_time = self.get_clock().now().nanoseconds / 1.0e9
        self._run_start_time = None
        self._last_status_time = self._start_time
        self._robot_x = None
        self._robot_y = None
        self._robot_yaw = None
        self._front_obstacle_distance = None
        self._last_obstacle_warning = False
        self._obstacle_candidate_distance = None
        self._obstacle_candidate_robot_x = None
        self._dynamic_obstacle_confirmed = False
        self._drive_timer = self.create_timer(0.1, self._drive_loop)
        self.get_logger().info(
            f"Auto drive armed; waiting for EKF pose, then driving toward "
            f"({self._goal_x:.2f}, {self._goal_y:.2f}) m"
        )

    def _pose_callback(self, msg):
        self._robot_x = float(msg.pose.pose.position.x)
        self._robot_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self._robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _cloud_callback(self, msg):
        nearest = None
        for point in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            distance = math.hypot(x, y)
            if distance < 0.35 or distance > self._obstacle_stop_distance:
                continue
            if abs(z) > self._lidar_height_tolerance or x <= 0.0:
                continue
            if abs(math.atan2(y, x)) > self._front_half_angle:
                continue
            nearest = distance if nearest is None else min(nearest, distance)
        self._front_obstacle_distance = nearest

    def _drive_loop(self):
        now = self.get_clock().now().nanoseconds / 1.0e9
        if (
            self._robot_x is None
            or self._robot_y is None
            or self._robot_yaw is None
        ):
            self._stop()
            return

        if self._run_start_time is None:
            if now - self._start_time < self._start_delay:
                self._stop()
                return
            self._run_start_time = now
            self.get_logger().info(
                f"Forward drive started at {self._speed:.2f} m/s toward "
                f"({self._goal_x:.2f}, {self._goal_y:.2f}) m"
            )

        distance_to_goal = (
            (self._goal_x - self._robot_x) ** 2
            + (self._goal_y - self._robot_y) ** 2
        ) ** 0.5
        if distance_to_goal <= self._goal_tolerance:
            self._stop()
            self._drive_timer.cancel()
            self.get_logger().info(
                f"Reached goal at ({self._robot_x:.2f}, {self._robot_y:.2f}) m"
            )
            return

        if now - self._run_start_time >= self._max_run_duration:
            self._stop()
            self._drive_timer.cancel()
            message = String()
            message.data = "AUTODRIVE_STOPPED reason=maximum_duration"
            self._diagnostic_pub.publish(message)
            self.get_logger().warning(message.data)
            return

        obstacle_detected = (
            self._front_obstacle_distance is not None
            and self._front_obstacle_distance <= self._obstacle_stop_distance
        )
        if obstacle_detected:
            self._stop()
            if self._obstacle_candidate_distance is None:
                self._obstacle_candidate_distance = self._front_obstacle_distance
                self._obstacle_candidate_robot_x = self._robot_x
            elif (
                abs(self._robot_x - self._obstacle_candidate_robot_x) <= 0.10
                and abs(
                    self._front_obstacle_distance
                    - self._obstacle_candidate_distance
                )
                >= 0.15
            ):
                self._dynamic_obstacle_confirmed = True

            if self._dynamic_obstacle_confirmed and not self._last_obstacle_warning:
                distance = self._front_obstacle_distance
                message = String()
                message.data = f"DYNAMIC_OBSTACLE_DETECTED distance={distance:.2f} m"
                self._diagnostic_pub.publish(message)
                self.get_logger().warning(message.data)
                self._last_obstacle_warning = True
            return
        self._obstacle_candidate_distance = None
        self._obstacle_candidate_robot_x = None
        self._dynamic_obstacle_confirmed = False
        if self._last_obstacle_warning:
            message = String()
            message.data = "DYNAMIC_OBSTACLE_CLEARED"
            self._diagnostic_pub.publish(message)
            self.get_logger().info(message.data)
            self._last_obstacle_warning = False

        command = Twist()
        command.linear.x = self._speed
        self._cmd_pub.publish(command)
        if now - self._last_status_time >= 1.0:
            self.get_logger().info(
                f"DRIVING | pose=({self._robot_x:.2f}, {self._robot_y:.2f}, "
                f"yaw={self._robot_yaw:.3f}) | "
                f"goal_distance={distance_to_goal:.2f} m"
            )
            self._last_status_time = now

    def _stop(self):
        self._cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = AutoDrive()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
