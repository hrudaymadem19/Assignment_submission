#!/usr/bin/env python3
"""Inject the assignment's odometry scale error and IMU random-walk bias."""

import csv
import copy
import math
import os
import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class NoiseInjector(Node):
    def __init__(self):
        super().__init__("slam_noise_injector")
        self.declare_parameter("odom_in", "/odom")
        self.declare_parameter("odom_out", "/odom_noisy")
        self.declare_parameter("imu_in", "/imu")
        self.declare_parameter("imu_out", "/imu_noisy")
        self.declare_parameter("velocity_scale", 0.70)
        self.declare_parameter("scale_x_min", 10.0)
        self.declare_parameter("scale_x_max", 15.0)
        self.declare_parameter("velocity_noise_std", 0.02)
        self.declare_parameter("gyro_noise_std", 0.005)
        self.declare_parameter("gyro_bias_rw_std", 0.0005)
        self.declare_parameter("random_seed", 42)
        self.declare_parameter("output_dir", "logs")

        self._scale = float(self.get_parameter("velocity_scale").value)
        self._x_min = float(self.get_parameter("scale_x_min").value)
        self._x_max = float(self.get_parameter("scale_x_max").value)
        self._velocity_std = float(self.get_parameter("velocity_noise_std").value)
        self._gyro_std = float(self.get_parameter("gyro_noise_std").value)
        self._bias_rw_std = float(self.get_parameter("gyro_bias_rw_std").value)
        self._rng = random.Random(int(self.get_parameter("random_seed").value))
        output_dir = os.path.abspath(str(self.get_parameter("output_dir").value))
        os.makedirs(output_dir, exist_ok=True)
        self._ground_truth_log = open(
            os.path.join(output_dir, "ground_truth.csv"), "w", newline=""
        )
        self._odom_log = open(os.path.join(output_dir, "noisy_odom.csv"), "w", newline="")
        self._imu_log = open(os.path.join(output_dir, "noisy_imu.csv"), "w", newline="")
        self._ground_truth_writer = csv.writer(self._ground_truth_log)
        self._odom_writer = csv.writer(self._odom_log)
        self._imu_writer = csv.writer(self._imu_log)
        self._ground_truth_writer.writerow(
            ["time", "x", "y", "yaw", "linear_velocity", "angular_velocity"]
        )
        self._odom_writer.writerow(
            [
                "time",
                "x_true",
                "y_true",
                "x_noisy",
                "y_noisy",
                "scale",
                "v_true",
                "v_recorded",
            ]
        )
        self._imu_writer.writerow(["time", "gyro_z_true", "bias_z", "gyro_z_recorded"])

        self._odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_out").value), 20
        )
        self._imu_pub = self.create_publisher(
            Imu, str(self.get_parameter("imu_out").value), 50
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_in").value), self._odom_callback, 20
        )
        self.create_subscription(
            Imu, str(self.get_parameter("imu_in").value), self._imu_callback, 50
        )

        self._last_odom_time = None
        self._last_true_pose = None
        self._noisy_x = None
        self._noisy_y = None
        self._noisy_yaw = None
        self._last_imu_time = None
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._last_status_time = None
        self._last_scale = None
        self.get_logger().info(
            f"Noise injection active: slip scale={self._scale:.2f} "
            f"for x=[{self._x_min:.1f}, {self._x_max:.1f}] m"
        )

    def _odom_callback(self, msg):
        now = stamp_seconds(msg.header.stamp)
        true_x = msg.pose.pose.position.x
        true_y = msg.pose.pose.position.y
        true_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self._last_odom_time is None or self._last_true_pose is None:
            true_v = 0.0
            true_vy = 0.0
            true_angular_velocity = 0.0
        else:
            dt = max(1.0e-6, min(now - self._last_odom_time, 0.25))
            previous_x, previous_y, previous_yaw = self._last_true_pose
            delta_x = true_x - previous_x
            delta_y = true_y - previous_y
            cos_yaw = math.cos(previous_yaw)
            sin_yaw = math.sin(previous_yaw)
            true_v = (cos_yaw * delta_x + sin_yaw * delta_y) / dt
            true_vy = (-sin_yaw * delta_x + cos_yaw * delta_y) / dt
            true_angular_velocity = normalize_angle(true_yaw - previous_yaw) / dt
        self._ground_truth_writer.writerow(
            [
                now,
                true_x,
                true_y,
                true_yaw,
                true_v,
                true_angular_velocity,
            ]
        )
        self._ground_truth_log.flush()
        scale = (
            self._scale
            if self._x_min <= true_x <= self._x_max
            else 1.0
        )
        if scale != self._last_scale:
            state = "SLIP ACTIVE" if scale != 1.0 else "NORMAL ODOMETRY"
            message = f"{state} | x={true_x:.2f} m | velocity scale={scale:.2f}"
            if scale != 1.0:
                self.get_logger().warning(message)
            else:
                self.get_logger().info(message)
            self._last_scale = scale
        if self._last_odom_time is None:
            dt = 0.0
            self._noisy_x = msg.pose.pose.position.x
            self._noisy_y = msg.pose.pose.position.y
            self._noisy_yaw = true_yaw
        else:
            dt = max(0.0, min(now - self._last_odom_time, 0.25))
            recorded_v = (
                scale * true_v
                + self._rng.gauss(0.0, self._velocity_std)
            )
            recorded_vy = (
                scale * true_vy
                + self._rng.gauss(0.0, self._velocity_std)
            )
            self._noisy_x += (
                recorded_v * math.cos(true_yaw) - recorded_vy * math.sin(true_yaw)
            ) * dt
            self._noisy_y += (
                recorded_v * math.sin(true_yaw) + recorded_vy * math.cos(true_yaw)
            ) * dt
            self._noisy_yaw = true_yaw

        out = copy.deepcopy(msg)
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = self._noisy_x
        out.pose.pose.position.y = self._noisy_y
        qx, qy, qz, qw = quaternion_from_yaw(self._noisy_yaw)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.pose.covariance = list(msg.pose.covariance)
        out.twist.twist.linear.x = (
            scale * true_v
            + self._rng.gauss(0.0, self._velocity_std)
        )
        out.twist.twist.linear.y = (
            scale * true_vy
            + self._rng.gauss(0.0, self._velocity_std)
        )
        out.twist.twist.angular.z = (
            true_angular_velocity
            + self._rng.gauss(0.0, self._velocity_std)
        )
        out.twist.covariance = list(msg.twist.covariance)
        self._odom_pub.publish(out)
        self._odom_writer.writerow(
            [
                now,
                true_x,
                msg.pose.pose.position.y,
                self._noisy_x,
                self._noisy_y,
                scale,
                true_v,
                out.twist.twist.linear.x,
            ]
        )
        self._odom_log.flush()
        self._last_odom_time = now
        if self._last_status_time is None or now - self._last_status_time >= 2.0:
            self.get_logger().info(
                f"ODOMETRY | true x={true_x:.2f} m | measured x={self._noisy_x:.2f} m | "
                f"true v={true_v:.3f} | "
                f"measured v={out.twist.twist.linear.x:.3f}"
            )
            self._last_status_time = now
        self._last_true_pose = (true_x, true_y, true_yaw)

    def _imu_callback(self, msg):
        now = stamp_seconds(msg.header.stamp)
        dt = 0.01 if self._last_imu_time is None else max(
            0.0, min(now - self._last_imu_time, 0.25)
        )
        random_walk_std = self._bias_rw_std * math.sqrt(dt)
        for i in range(3):
            self._gyro_bias[i] += self._rng.gauss(0.0, random_walk_std)

        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = list(msg.orientation_covariance)
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity.x += self._gyro_bias[0] + self._rng.gauss(0.0, self._gyro_std)
        out.angular_velocity.y += self._gyro_bias[1] + self._rng.gauss(0.0, self._gyro_std)
        out.angular_velocity.z += self._gyro_bias[2] + self._rng.gauss(0.0, self._gyro_std)
        out.angular_velocity_covariance = list(msg.angular_velocity_covariance)
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = list(msg.linear_acceleration_covariance)
        self._imu_pub.publish(out)
        self._imu_writer.writerow(
            [now, msg.angular_velocity.z, self._gyro_bias[2], out.angular_velocity.z]
        )
        self._imu_log.flush()
        self._last_imu_time = now

    def destroy_node(self):
        self._ground_truth_log.close()
        self._odom_log.close()
        self._imu_log.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = NoiseInjector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
