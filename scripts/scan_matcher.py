#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class CorridorScanMatcher(Node):

    def __init__(self):
        super().__init__('corridor_scan_matcher')

        # Topics
        self.declare_parameter('cloud_topic', '/lidar/points/points')
        self.declare_parameter('output_topic', '/lidar/pose_measurement')
        self.declare_parameter('ekf_topic', '/ekf/odom')
        self.declare_parameter('odom_topic', '/odom_noisy')

        # Corridor geometry
        self.declare_parameter('left_wall_y', 3.15)
        self.declare_parameter('right_wall_y', -3.25)
        self.declare_parameter('far_wall_x', 20.0)
        self.declare_parameter('moving_wall_x', 10.0)

        # LiDAR filtering
        self.declare_parameter('height_tolerance', 0.15)
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_range', 25.0)

        self.declare_parameter('ekf_prior_weight', 1.0)
        self.declare_parameter('odom_prior_weight', 1.0)

        self.left_y = float(self.get_parameter('left_wall_y').value)
        self.right_y = float(self.get_parameter('right_wall_y').value)
        self.far_x = float(self.get_parameter('far_wall_x').value)
        self.moving_x = float(self.get_parameter('moving_wall_x').value)
        self.h_tol = float(self.get_parameter('height_tolerance').value)
        self.min_r = float(self.get_parameter('min_range').value)
        self.max_r = float(self.get_parameter('max_range').value)

       
        self._ekf_x = None
        self._ekf_y = None
        self._ekf_yaw = None
        self._odom_x = None
        self._odom_y = None
        self._odom_yaw = None

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter('output_topic').value),
            10,
        )

        self.create_subscription(
            Odometry,
            str(self.get_parameter('ekf_topic').value),
            self._ekf_callback,
            20,
        )

        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            20,
        )

        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('cloud_topic').value),
            self._cloud_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'Corridor scan matcher active: '
            'x from forward-wall range, y from side-wall ranges, '
            'yaw from side-wall line fit. Covariance is data-derived.'
        )

    # ----------------------------------------------------------------

    def _ekf_callback(self, msg):
        self._ekf_x = msg.pose.pose.position.x
        self._ekf_y = msg.pose.pose.position.y
        self._ekf_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _odom_callback(self, msg):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _prior(self):
        # Prefer EKF; fall back to noisy odom before EKF warms up.
        if self._ekf_x is not None:
            return self._ekf_x, self._ekf_y, self._ekf_yaw
        if self._odom_x is not None:
            return self._odom_x, self._odom_y, self._odom_yaw
        return None, None, None

    # ----------------------------------------------------------------

    def _cloud_callback(self, msg):

        forward_ranges = []
        left_ys = []
        right_ys = []
        left_pts = []
        right_pts = []

        for px, py, pz in point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        ):
            x = float(px)
            y = float(py)
            z = float(pz)

            r = math.hypot(x, y)

            if r < self.min_r or r > self.max_r:
                continue
            if abs(z) > self.h_tol:
                continue

            angle = math.atan2(y, x)

            # Forward cone (±10 deg)
            if abs(angle) < math.radians(10.0):
                forward_ranges.append(r)

            # Left side
            if math.radians(60.0) < angle < math.radians(120.0):
                left_ys.append(y)
                left_pts.append((x, y))

            # Right side
            if -math.radians(120.0) < angle < -math.radians(60.0):
                right_ys.append(y)
                right_pts.append((x, y))

                if len(forward_ranges) < 3:
            return

        
        # Forward-wall x measurement
       
        forward_ranges.sort()
        n_f = len(forward_ranges)

        d_forward = forward_ranges[max(0, n_f // 10)]

        # Two hypotheses for which wall we are seeing
        x_hypo_far = self.far_x - d_forward
        x_hypo_moving = self.moving_x - d_forward

        prior_x, _, _ = self._prior()

        if prior_x is None:
            
            x_meas = x_hypo_moving
            wall_confidence = 0.5
        else:
            d_far = abs(prior_x - x_hypo_far)
            d_moving = abs(prior_x - x_hypo_moving)
            if d_far < d_moving:
                x_meas = x_hypo_far
                wall_confidence = 1.0 - d_far / max(d_far + d_moving, 1e-3)
            else:
                x_meas = x_hypo_moving
                wall_confidence = 1.0 - d_moving / max(d_far + d_moving, 1e-3)


        fwd_arr = np.asarray(forward_ranges, dtype=float)
        fwd_spread = float(np.std(fwd_arr))

       
        sigma_x = math.sqrt(
            0.02 ** 2
            + (0.003 * d_forward) ** 2          # 0.003 rad * range
            + (fwd_spread / math.sqrt(n_f)) ** 2
        )

        # If the two wall hypotheses are nearly equally plausible,
        # x is fundamentally ambiguous — inflate the uncertainty.
        if wall_confidence < 0.7:
            sigma_x += 0.30

        
        # Side-wall y measurement

        y_estimates = []
        if left_ys:
            y_estimates.append(self.left_y - float(np.mean(left_ys)))
        if right_ys:
            y_estimates.append(self.right_y - float(np.mean(right_ys)))

        if y_estimates:
            y_meas = float(np.mean(y_estimates))
            # More points on the walls -> tighter y
            n_side = len(left_ys) + len(right_ys)
            sigma_y = max(0.02, 0.15 / math.sqrt(max(n_side, 1)))
        else:
            prior_x, prior_y, prior_yaw = self._prior()
            y_meas = prior_y if prior_y is not None else 0.0
            sigma_y = 0.50

        # Yaw
        
        slopes = []
        for pts in (left_pts, right_pts):
            if len(pts) < 5:
                continue
            arr = np.asarray(pts, dtype=float)
            xs = arr[:, 0]
            ys = arr[:, 1]
            if float(np.std(xs)) < 0.1:
                continue
            m, _ = np.polyfit(xs, ys, 1)
            slopes.append(m)

        if slopes:
            mean_slope = float(np.mean(slopes))
            yaw_meas = math.atan(mean_slope)
            # Residual of the line fit -> yaw uncertainty
            residuals = []
            for pts in (left_pts, right_pts):
                if len(pts) < 5:
                    continue
                arr = np.asarray(pts, dtype=float)
                m, b = np.polyfit(arr[:, 0], arr[:, 1], 1)
                residuals.append(float(np.std(arr[:, 1] - (m * arr[:, 0] + b))))
            if residuals:
                residual = float(np.mean(residuals))
                sigma_yaw = math.atan(residual / max(1.0, float(np.ptp(
                    np.asarray([p[0] for p in left_pts + right_pts])))))
                sigma_yaw = max(math.radians(0.3), min(sigma_yaw, math.radians(15.0)))
            else:
                sigma_yaw = math.radians(3.0)
        else:
            _, prior_y, prior_yaw = self._prior()
            yaw_meas = prior_yaw if prior_yaw is not None else 0.0
            sigma_yaw = math.radians(10.0)

        # Publish
        

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'

        out.pose.pose.position.x = x_meas
        out.pose.pose.position.y = y_meas
        out.pose.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_yaw(yaw_meas)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        out.pose.covariance[0] = sigma_x ** 2
        out.pose.covariance[7] = sigma_y ** 2
        out.pose.covariance[35] = sigma_yaw ** 2

        self._pub.publish(out)


def main():
    rclpy.init()
    node = CorridorScanMatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
