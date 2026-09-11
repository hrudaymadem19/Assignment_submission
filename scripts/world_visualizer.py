#!/usr/bin/env python3
"""Publish RViz"""

from rclpy.node import Node
import rclpy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray


class WorldVisualizer(Node):
    def __init__(self):
        super().__init__("slam_world_visualizer")
        self._wall_y = -6.0
        self._map = None
        self._publisher = self.create_publisher(
            MarkerArray, "/slam_robot/world_markers", 1
        )
        self.create_subscription(Float64, "/shifting_wall/y", self._wall_callback, 10)
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, 1)
        self.create_timer(0.2, self._publish_markers)

    def _wall_callback(self, message):
        self._wall_y = message.data

    def _map_callback(self, message):
        self._map = message

    def _box(self, marker_id, x, y, z, scale_x, scale_y, scale_z, color):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "gazebo_world"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime.sec = 0
        return marker

    def _publish_markers(self):
        markers = MarkerArray()
        markers.markers = [
            self._box(0, 10.0, 0.0, -0.05, 25.0, 12.0, 0.1, (0.35, 0.35, 0.35, 0.35)),
            self._box(1, 10.0, 3.15, 1.5, 20.0, 0.3, 3.0, (0.8, 0.8, 0.8, 0.8)),
            self._box(2, 10.0, -3.25, 1.5, 20.0, 0.5, 3.0, (0.8, 0.8, 0.8, 0.8)),
            self._box(3, 20.0, 0.0, 1.5, 0.3, 6.6, 3.0, (0.8, 0.8, 0.8, 0.8)),
            self._box(4, 10.0, self._wall_y, 1.5, 0.2, 6.0, 3.0, (0.9, 0.7, 0.2, 0.9)),
        ]
        self._publisher.publish(markers)


def main():
    rclpy.init()
    node = WorldVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
