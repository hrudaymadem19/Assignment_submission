#!/usr/bin/env python3
"""Oscillate the Gazebo sliding wall between its open and closed positions."""

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from std_msgs.msg import Float64, Int32


class ShiftingWallController(Node):
    def __init__(self):
        super().__init__("shifting_wall_controller")
        self.declare_parameter("pose_service", "/world/assignment_world/set_pose")
        self.declare_parameter("step_topic", "/shifting_wall/step")
        self.declare_parameter("interval_sec", 15.0)
        self.declare_parameter("shift_m", 1.5)
        self.declare_parameter("initial_y", -6.0)
        self.declare_parameter("min_y", -6.0)
        self.declare_parameter("max_y", 0.0)
        self.declare_parameter("wall_x", 10.0)
        self.declare_parameter("wall_z", 1.5)
        self._x = float(self.get_parameter("wall_x").value)
        self._y = float(self.get_parameter("initial_y").value)
        self._min_y = float(self.get_parameter("min_y").value)
        self._max_y = float(self.get_parameter("max_y").value)
        self._z = float(self.get_parameter("wall_z").value)
        self._shift = float(self.get_parameter("shift_m").value)
        self._direction = 1
        self._step = 0
        self._pose_client = self.create_client(
            SetEntityPose, str(self.get_parameter("pose_service").value)
        )
        self._step_pub = self.create_publisher(
            Int32, str(self.get_parameter("step_topic").value), 10
        )
        self._y_pub = self.create_publisher(Float64, "/shifting_wall/y", 10)
        self._timer = None
        self._initial_request_pending = False
        self._startup_timer = self.create_timer(0.1, self._start_when_ready)
        self.get_logger().info(
            "Wall controller started: y={:.2f} m, moving {:.2f} m every {:.2f} s".format(
                self._y,
                self._shift,
                float(self.get_parameter("interval_sec").value),
            )
        )

    def _start_when_ready(self):
        if not self._pose_client.service_is_ready():
            return
        if not self._initial_request_pending:
            self._initial_request_pending = True
            self._publish_pose()

    def _publish_pose(self):
        request = SetEntityPose.Request()
        request.entity.name = "shifting_wall"
        request.entity.type = Entity.MODEL
        request.pose.position.x = self._x
        request.pose.position.y = self._y
        request.pose.position.z = self._z
        request.pose.orientation.w = 1.0
        future = self._pose_client.call_async(request)
        future.add_done_callback(self._service_result)

    def _service_result(self, future):
        if future.exception() is not None:
            self.get_logger().error(
                f"Gazebo set_pose request failed: {future.exception()}"
            )
            self._initial_request_pending = False
            return
        response = future.result()
        if not response.success:
            self.get_logger().error("Gazebo rejected shifting wall pose request")
            self._initial_request_pending = False
            return

        step = Int32()
        step.data = self._step
        self._step_pub.publish(step)
        wall_y = Float64()
        wall_y.data = self._y
        self._y_pub.publish(wall_y)

        if self._timer is None:
            self._startup_timer.cancel()
            self._timer = self.create_timer(
                float(self.get_parameter("interval_sec").value), self._move_wall
            )

    def _move_wall(self):
        if self._timer is None or not self._pose_client.service_is_ready():
            self.get_logger().warning(
                "Gazebo set_pose service is not ready; keeping wall at its last pose"
            )
            return

        new_y = self._y + self._shift * self._direction
        if new_y >= self._max_y:
            new_y = self._max_y
            self._direction = -1
            self.get_logger().info("CORRIDOR CLOSED, opening...")
        elif new_y <= self._min_y:
            new_y = self._min_y
            self._direction = 1
            self.get_logger().info("CORRIDOR OPEN, closing...")

        self._y = new_y
        self._step += 1
        self._publish_pose()
        self.get_logger().info(f"Wall at y={self._y:.2f} m")


def main():
    rclpy.init()
    node = ShiftingWallController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
