#!/usr/bin/env python3
"""Maintain a ray-cast 2-D submap and report scan-matching degeneracy."""

import csv
import math
import os

from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        twice_error = 2 * error
        if twice_error >= dy:
            error += dy
            x0 += sx
        if twice_error <= dx:
            error += dx
            y0 += sy


def transform_point(x, y, yaw, tx, ty):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * x - sin_yaw * y + tx,
        sin_yaw * x + cos_yaw * y + ty,
    )


def match_scans(previous, current):
    if len(previous) < 10 or len(current) < 10:
        return None
    source = current[:: max(1, len(current) // 250)]
    target = previous[:: max(1, len(previous) // 250)]
    pairs = []
    for source_x, source_y, _ in source:
        nearest = min(
            target,
            key=lambda point: (point[0] - source_x) ** 2
            + (point[1] - source_y) ** 2,
        )
        distance_sq = (nearest[0] - source_x) ** 2 + (
            nearest[1] - source_y
        ) ** 2
        if distance_sq <= 1.0:
            pairs.append(((source_x, source_y), (nearest[0], nearest[1])))
    if len(pairs) < 10:
        return None

    source_mean = (
        sum(pair[0][0] for pair in pairs) / len(pairs),
        sum(pair[0][1] for pair in pairs) / len(pairs),
    )
    target_mean = (
        sum(pair[1][0] for pair in pairs) / len(pairs),
        sum(pair[1][1] for pair in pairs) / len(pairs),
    )
    sine = 0.0
    cosine = 0.0
    for source_point, target_point in pairs:
        source_x = source_point[0] - source_mean[0]
        source_y = source_point[1] - source_mean[1]
        target_x = target_point[0] - target_mean[0]
        target_y = target_point[1] - target_mean[1]
        sine += source_x * target_y - source_y * target_x
        cosine += source_x * target_x + source_y * target_y
    yaw = math.atan2(sine, cosine)
    tx, ty = transform_point(
        source_mean[0],
        source_mean[1],
        yaw,
        0.0,
        0.0,
    )
    return target_mean[0] - tx, target_mean[1] - ty, yaw, len(pairs)


class MappingDiagnostics(Node):
    def __init__(self):
        super().__init__("slam_mapping_diagnostics")
        self.declare_parameter("cloud_topic", "/lidar/points/points")
        self.declare_parameter("map_pose_topic", "/ekf/odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("information_topic", "/scan_matching/information_matrix")
        self.declare_parameter("diagnostics_topic", "/localization/diagnostics")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("map_width", 300)
        self.declare_parameter("map_height", 100)
        self.declare_parameter("origin_x", -5.0)
        self.declare_parameter("origin_y", -5.0)
        self.declare_parameter("degeneracy_ratio", 0.90)
        self.declare_parameter("minimum_information_x", 1.0)
        self.declare_parameter("output_dir", "logs")
        self.declare_parameter("max_points_per_scan", 1500)
        self.declare_parameter("min_process_interval_sec", 0.10)
        self.declare_parameter("scan_height_tolerance", 0.12)

        self._resolution = float(self.get_parameter("resolution").value)
        self._width = int(self.get_parameter("map_width").value)
        self._height = int(self.get_parameter("map_height").value)
        self._origin_x = float(self.get_parameter("origin_x").value)
        self._origin_y = float(self.get_parameter("origin_y").value)
        self._map_evidence = [0.0] * (self._width * self._height)
        self._map_pose = (0.0, 0.0, 0.0)
        self._last_warning = False
        self._last_log_time = None
        self._last_process_stamp = None
        self._last_map_stamp = None
        self._last_info = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]

        self._map_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("map_topic").value), 1
        )
        self._info_pub = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("information_topic").value), 10
        )
        self._scan_marker_pub = self.create_publisher(
            Marker, "/slam_robot/lidar_rays", 10
        )
        self._diagnostic_pub = self.create_publisher(
            String, str(self.get_parameter("diagnostics_topic").value), 10
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("map_pose_topic").value),
            self._map_pose_callback,
            20,
        )
        self._map_frame = "map"

        output_dir = os.path.abspath(str(self.get_parameter("output_dir").value))
        os.makedirs(output_dir, exist_ok=True)
        self._log = open(os.path.join(output_dir, "diagnostics.csv"), "w", newline="")
        self._writer = csv.writer(self._log)
        self._writer.writerow(
            [
                "time",
                "information_xx",
                "information_yy",
                "information_yaw",
                "x_ratio",
                "sigma_x",
                "sigma_y",
                "warning",
                "warning_message",
            ]
        )

    def _map_pose_callback(self, msg):
        self._map_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw_from_quaternion(msg.pose.pose.orientation),
        )

    def _index(self, grid_x, grid_y):
        if 0 <= grid_x < self._width and 0 <= grid_y < self._height:
            return grid_y * self._width + grid_x
        return None

    def _world_to_grid(self, x, y):
        return (
            int(math.floor((x - self._origin_x) / self._resolution)),
            int(math.floor((y - self._origin_y) / self._resolution)),
        )

    def _cloud_callback(self, msg):
        now = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9
        min_interval = float(self.get_parameter("min_process_interval_sec").value)
        if self._last_process_stamp is not None and now - self._last_process_stamp < min_interval:
            return
        self._last_process_stamp = now

        max_points = int(self.get_parameter("max_points_per_scan").value)
        height_tolerance = float(
            self.get_parameter("scan_height_tolerance").value
        )
        # Select the return nearest the horizontal plane for each azimuth.
        # A 16-channel lidar has no guarantee that a channel lands within a
        # narrow fixed Z tolerance, especially at longer ranges.
        horizontal_returns = {}
        scan_points = []
        for point in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            local_x, local_y, local_z = (
                float(point[0]),
                float(point[1]),
                float(point[2]),
            )
            distance = math.hypot(local_x, local_y)
            if distance < 0.12 or distance > 30.0:
                continue
            # Keep the near-horizontal returns used for planar scan matching.
            # This prevents floor returns from falsely adding corridor-axis
            # information.
            if abs(local_z) > height_tolerance:
                continue
            azimuth = int(round(math.atan2(local_y, local_x) * 1800.0 / math.pi))
            candidate = horizontal_returns.get(azimuth)
            if candidate is None or abs(local_z) < abs(candidate[2]):
                horizontal_returns[azimuth] = (local_x, local_y, local_z)

        points = list(horizontal_returns.values())
        if len(points) > max_points:
            stride = max(1, len(points) // max_points)
            points = points[::stride][:max_points]
        scan_points = points
        if not points:
            self._publish_scan_marker(msg, scan_points)
            self._publish_map(msg.header.stamp)
            return

        self._publish_scan_marker(msg, scan_points)
        px, py, yaw = self._map_pose
        robot_cell = self._world_to_grid(px, py)
        info = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        for local_x, local_y, _local_z in points:
            world_x = px + math.cos(yaw) * local_x - math.sin(yaw) * local_y
            world_y = py + math.sin(yaw) * local_x + math.cos(yaw) * local_y
            endpoint = self._world_to_grid(world_x, world_y)
            ray = list(bresenham(robot_cell[0], robot_cell[1], endpoint[0], endpoint[1]))
            for free_x, free_y in ray[:-1]:
                index = self._index(free_x, free_y)
                if index is not None:
                    self._map_evidence[index] = max(
                        self._map_evidence[index] - 1.0, -5.0
                    )
            endpoint_index = self._index(endpoint[0], endpoint[1])
            if endpoint_index is not None:
                self._map_evidence[endpoint_index] = min(
                    self._map_evidence[endpoint_index] + 2.0, 10.0
                )

            range_sq = max(local_x * local_x + local_y * local_y, 0.25)
            weight = 1.0 / range_sq
            range_norm = math.sqrt(range_sq)
            nx = local_x / range_norm
            ny = local_y / range_norm
            jacobian = (nx, ny, -ny * local_x + nx * local_y)
            for row in range(3):
                for column in range(3):
                    info[row][column] += (
                        weight * jacobian[row] * jacobian[column]
                    )

        info_x = info[0][0]
        info_y = info[1][1]
        self._last_info = info
        ratio = info_x / max(info_y, 1.0e-9)
        sigma_x = 1.0 / math.sqrt(max(info_x, 1.0e-9))
        sigma_y = 1.0 / math.sqrt(max(info_y, 1.0e-9))
        warning = (
            ratio < float(self.get_parameter("degeneracy_ratio").value)
            or info_x < float(self.get_parameter("minimum_information_x").value)
        )
        diagnostic = String()
        if warning:
            diagnostic.data = "LOCALIZATION_DEGENERACY_WARNING"
        else:
            diagnostic.data = (
                f"LOCALIZATION_OK information_ratio={ratio:.6f} "
                f"sigma_x={sigma_x:.3f} sigma_y={sigma_y:.3f}"
            )
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9
        if self._last_log_time is None or stamp - self._last_log_time >= 2.0:
            if warning:
                self.get_logger().warning("LOCALIZATION_DEGENERACY_WARNING")
            else:
                self.get_logger().info(
                    f"SCAN DIAGNOSTICS | OK | pose=({px:.2f}, {py:.2f}) m | "
                    f"info_x={info_x:.2f} | info_y={info_y:.2f} | "
                    f"sigma_x={sigma_x:.3f} | sigma_y={sigma_y:.3f} | "
                    f"ratio={ratio:.6f}"
                )
            self._last_log_time = stamp
        self._diagnostic_pub.publish(diagnostic)
        self._last_warning = warning

        info_msg = Float64MultiArray()
        info_msg.data = [value for row in info for value in row]
        self._info_pub.publish(info_msg)
        self._publish_map(msg.header.stamp)
        self._writer.writerow(
            [
                stamp,
                info_x,
                info_y,
                info[2][2],
                ratio,
                sigma_x,
                sigma_y,
                int(warning),
                diagnostic.data if warning else "",
            ]
        )
        self._log.flush()

    def _publish_scan_marker(self, cloud_msg, scan_points):
        marker = Marker()
        marker.header = cloud_msg.header
        marker.ns = "lidar_scan_rays"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.008
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 0.45
        for local_x, local_y, local_z in scan_points[::2]:
            marker.points.append(Point())
            marker.points.append(
                Point(x=local_x, y=local_y, z=local_z)
            )
        marker.lifetime.sec = 0
        self._scan_marker_pub.publish(marker)

    def _publish_map(self, stamp):
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = self._map_frame
        grid.info.resolution = self._resolution
        grid.info.width = self._width
        grid.info.height = self._height
        grid.info.origin.position.x = self._origin_x
        grid.info.origin.position.y = self._origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = [
            100 if evidence >= 0.5
            else 50 if evidence <= -0.5
            else -1
            for evidence in self._map_evidence
        ]
        self._map_pub.publish(grid)
        self._last_map_stamp = stamp

    def destroy_node(self):
        self._log.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = MappingDiagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
