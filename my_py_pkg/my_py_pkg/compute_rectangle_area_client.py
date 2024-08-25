#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from my_robot_interfaces.srv import ComputeRectangleArea


class ComputeRectangleAreaClientNode(Node):
    def __init__(self):
        super().__init__("compute_rectangle_area_client")
        self.call_compute_rectangle_server(12.0, 10.0)

    def call_compute_rectangle_server(self, length, width):
        client = self.create_client(
            ComputeRectangleArea, "compute_rectangle_area")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Compute Rectangle Area Server...")

        request = ComputeRectangleArea.Request()
        request.length = length
        request.width = width

        future = client.call_async(request)
        future.add_done_callback(partial(
            self.callback_call_compute_rectangle_server, length=length, width=width))

    def callback_call_compute_rectangle_server(self, future, length, width):
        try:
            response = future.result()
            self.get_logger().info(str(length) + " * " + str(width) + " = " + str(response.area))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = ComputeRectangleAreaClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
