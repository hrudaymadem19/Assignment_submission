#!/usr/bin/env python3
"""Differential-drive EKF for noisy odometry, IMU, and LiDAR pose updates."""

import csv
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PlanarEkf(Node):
    """State is [x, y, yaw, linear_velocity, angular_velocity, gyro_bias]."""

    def __init__(self):
        super().__init__("slam_planar_ekf")
        self.declare_parameter("odom_in", "/odom_noisy")
        self.declare_parameter("tf_odom_in", "/odom")
        self.declare_parameter("imu_in", "/imu_noisy")
        self.declare_parameter("lidar_pose_in", "/lidar/pose_measurement")
        self.declare_parameter("pose_topic", "/ekf/pose_with_covariance")
        self.declare_parameter("odom_topic", "/ekf/odom")
        self.declare_parameter("output_dir", "logs")
        self.declare_parameter("odom_linear_std", 0.02)
        self.declare_parameter("odom_angular_std", 0.02)
        self.declare_parameter("gyro_std", 0.005)
        self.declare_parameter("gyro_bias_process_std", 0.0005)

        self._x = np.zeros(6, dtype=float)
        self._p = np.diag(
            [
                0.05**2,
                0.05**2,
                math.radians(5.0) ** 2,
                0.10**2,
                0.10**2,
                0.02**2,
            ]
        )
        self._last_time = None
        self._initialized = False
        self._last_log_time = None
        self._odom_pose = np.zeros(3, dtype=float)
        self._tf_odom_pose = None
        self._last_stamp = None

        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_topic").value),
            20,
        )
        self._odom_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            20,
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_in").value),
            self._imu_callback,
            50,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_in").value),
            self._odom_callback,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("tf_odom_in").value),
            self._tf_odom_callback,
            20,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("lidar_pose_in").value),
            self._lidar_pose_callback,
            20,
        )

        output_dir = os.path.abspath(str(self.get_parameter("output_dir").value))
        os.makedirs(output_dir, exist_ok=True)
        self._log = open(os.path.join(output_dir, "ekf.csv"), "w", newline="")
        self._writer = csv.writer(self._log)
        self._writer.writerow(
            [
                "time",
                "x",
                "y",
                "yaw",
                "cov_x",
                "cov_y",
                "cov_yaw",
                "cov_bias",
                "cov_x_y",
                "cov_x_yaw",
                "cov_x_bias",
                "cov_y_x",
                "cov_y_yaw",
                "cov_y_bias",
                "cov_yaw_x",
                "cov_yaw_y",
                "cov_yaw_bias",
                "cov_bias_x",
                "cov_bias_y",
                "cov_bias_yaw",
            ]
        )

    def _predict(self, dt):
        if dt <= 0.0:
            return

        x, y, yaw, velocity, angular_velocity, _ = self._x
        yaw_mid = yaw + 0.5 * angular_velocity * dt
        self._x[0] += velocity * math.cos(yaw_mid) * dt
        self._x[1] += velocity * math.sin(yaw_mid) * dt
        self._x[2] = normalize_angle(yaw + angular_velocity * dt)

        f = np.eye(6)
        f[0, 2] = -velocity * math.sin(yaw_mid) * dt
        f[0, 3] = math.cos(yaw_mid) * dt
        f[0, 4] = -0.5 * velocity * math.sin(yaw_mid) * dt**2
        f[1, 2] = velocity * math.cos(yaw_mid) * dt
        f[1, 3] = math.sin(yaw_mid) * dt
        f[1, 4] = 0.5 * velocity * math.cos(yaw_mid) * dt**2
        f[2, 4] = dt

        sigma_v = float(self.get_parameter("odom_linear_std").value)
        sigma_w = float(self.get_parameter("odom_angular_std").value)
        sigma_b = float(self.get_parameter("gyro_bias_process_std").value)
        q = np.diag(
            [
                0.0,
                0.0,
                0.0,
                sigma_v**2,
                sigma_w**2,
                sigma_b**2 * max(dt, 1.0e-3),
            ]
        )
        self._p = f @ self._p @ f.T + q
        self._p = 0.5 * (self._p + self._p.T)

    def _update(self, measurement, expected, h, covariance, angle_index=None):
        innovation = np.asarray(measurement, dtype=float) - np.asarray(
            expected, dtype=float
        )
        if angle_index is not None:
            innovation[angle_index] = normalize_angle(innovation[angle_index])

        s = h @ self._p @ h.T + covariance
        try:
            gain = np.linalg.solve(s, h @ self._p).T
        except np.linalg.LinAlgError:
            self.get_logger().warning("EKF update skipped: singular innovation covariance")
            return

        self._x += gain @ innovation
        self._x[2] = normalize_angle(self._x[2])
        identity = np.eye(6)
        residual = identity - gain @ h
        self._p = (
            residual @ self._p @ residual.T
            + gain @ covariance @ gain.T
        )
        self._p = 0.5 * (self._p + self._p.T)

    def _imu_callback(self, msg):
        if not self._initialized:
            return
        measurement = np.array([msg.angular_velocity.z], dtype=float)
        expected = np.array([self._x[4] + self._x[5]], dtype=float)
        h = np.zeros((1, 6), dtype=float)
        h[0, 4] = 1.0
        h[0, 5] = 1.0
        sigma = float(self.get_parameter("gyro_std").value)
        self._update(
            measurement,
            expected,
            h,
            np.array([[sigma**2]], dtype=float),
        )

    def _tf_odom_callback(self, msg):
        self._tf_odom_pose = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw_from_quaternion(msg.pose.pose.orientation),
            ],
            dtype=float,
        )

    def _odom_callback(self, msg):
        now = stamp_seconds(msg.header.stamp)
        odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._odom_pose[:] = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            odom_yaw,
        ]

        if not self._initialized:
            self._x[:3] = self._odom_pose
            self._x[3] = msg.twist.twist.linear.x
            self._x[4] = msg.twist.twist.angular.z
            self._last_time = now
            self._last_stamp = msg.header.stamp
            self._initialized = True
            self._publish(msg.header.stamp, "base_footprint")
            return

        dt = max(0.0, min(now - self._last_time, 0.25))
        self._predict(dt)
        self._last_time = now
        self._last_stamp = msg.header.stamp

        measurement = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.angular.z,
            ],
            dtype=float,
        )
        expected = self._x[3:5].copy()
        h = np.zeros((2, 6), dtype=float)
        h[0, 3] = 1.0
        h[1, 4] = 1.0
        linear_sigma = float(self.get_parameter("odom_linear_std").value)
        angular_sigma = float(self.get_parameter("odom_angular_std").value)
        self._update(
            measurement,
            expected,
            h,
            np.diag([linear_sigma**2, angular_sigma**2]),
        )
        self._publish(msg.header.stamp, msg.child_frame_id or "base_footprint")

        if self._last_log_time is None or now - self._last_log_time >= 1.0:
            self.get_logger().info(
                f"EKF POSE | x={self._x[0]:.2f} m | y={self._x[1]:.2f} m | "
                f"yaw={self._x[2]:.2f} rad | "
                f"cov=({self._p[0, 0]:.4f}, {self._p[1, 1]:.4f}, "
                f"{self._p[2, 2]:.4f})"
            )
            self._last_log_time = now

    def _lidar_pose_callback(self, msg):
        if not self._initialized:
            return
        measurement = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw_from_quaternion(msg.pose.pose.orientation),
            ],
            dtype=float,
        )
        expected = self._x[:3].copy()
        position_innovation = math.hypot(
            measurement[0] - expected[0],
            measurement[1] - expected[1],
        )
        yaw_innovation = abs(normalize_angle(measurement[2] - expected[2]))
        if position_innovation > 1.5 or yaw_innovation > math.radians(30.0):
            self.get_logger().warning(
                "Rejected discontinuous LiDAR pose update: "
                f"position={position_innovation:.2f} m, "
                f"yaw={math.degrees(yaw_innovation):.1f} deg"
            )
            return
        h = np.zeros((3, 6), dtype=float)
        h[0, 0] = 1.0
        h[1, 1] = 1.0
        h[2, 2] = 1.0
        covariance = np.diag(
            [
                max(float(msg.pose.covariance[0]), 0.05**2),
                max(float(msg.pose.covariance[7]), 0.05**2),
                max(float(msg.pose.covariance[35]), math.radians(2.0) ** 2),
            ]
        )
        self._update(measurement, expected, h, covariance, angle_index=2)
        self._publish(msg.header.stamp, "base_footprint")

    def _publish(self, stamp, child_frame_id):
        qx, qy, qz, qw = quaternion_from_yaw(self._x[2])
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = self._x[0]
        pose.pose.pose.position.y = self._x[1]
        pose.pose.pose.orientation.x = qx
        pose.pose.pose.orientation.y = qy
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw
        pose.pose.covariance[0] = self._p[0, 0]
        pose.pose.covariance[1] = self._p[0, 1]
        pose.pose.covariance[5] = self._p[0, 2]
        pose.pose.covariance[6] = self._p[1, 0]
        pose.pose.covariance[7] = self._p[1, 1]
        pose.pose.covariance[11] = self._p[1, 2]
        pose.pose.covariance[30] = self._p[2, 0]
        pose.pose.covariance[31] = self._p[2, 1]
        pose.pose.covariance[35] = self._p[2, 2]
        self._pose_pub.publish(pose)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = child_frame_id
        odom.pose = pose.pose
        odom.twist.twist.linear.x = self._x[3]
        odom.twist.twist.angular.z = self._x[4]
        odom.twist.covariance[0] = self._p[3, 3]
        odom.twist.covariance[35] = self._p[4, 4]
        self._odom_pub.publish(odom)

        tf_odom_pose = self._tf_odom_pose
        if tf_odom_pose is not None:
            yaw_error = normalize_angle(self._x[2] - tf_odom_pose[2])
            correction_q = quaternion_from_yaw(yaw_error)
            cos_yaw = math.cos(yaw_error)
            sin_yaw = math.sin(yaw_error)
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "map"
            transform.child_frame_id = "odom"
            transform.transform.translation.x = (
                self._x[0]
                - cos_yaw * tf_odom_pose[0]
                + sin_yaw * tf_odom_pose[1]
            )
            transform.transform.translation.y = (
                self._x[1]
                - sin_yaw * tf_odom_pose[0]
                - cos_yaw * tf_odom_pose[1]
            )
            transform.transform.rotation.z = correction_q[2]
            transform.transform.rotation.w = correction_q[3]
            self._tf_broadcaster.sendTransform(transform)

        self._writer.writerow(
            [
                stamp_seconds(stamp),
                self._x[0],
                self._x[1],
                self._x[2],
                self._p[0, 0],
                self._p[1, 1],
                self._p[2, 2],
                self._p[5, 5],
                self._p[0, 1],
                self._p[0, 2],
                self._p[0, 5],
                self._p[1, 0],
                self._p[1, 2],
                self._p[1, 5],
                self._p[2, 0],
                self._p[2, 1],
                self._p[2, 5],
                self._p[5, 0],
                self._p[5, 1],
                self._p[5, 2],
            ]
        )
        self._log.flush()

    def destroy_node(self):
        self._log.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PlanarEkf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
