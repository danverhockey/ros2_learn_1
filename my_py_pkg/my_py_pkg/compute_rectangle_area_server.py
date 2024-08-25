#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import ComputeRectangleArea


class ComputeRectangleAreaServerNode(Node):
    def __init__(self):
        super().__init__("compute_rectangle_area_server")
        self.compute_rectangle_area_server_ = self.create_service(
            ComputeRectangleArea, "compute_rectangle_area", self.callback_compute_rectangle_server)
        self.get_logger().info("Compute Rectangle Area Server has been started.")

    def callback_compute_rectangle_server(self, request, response):
        response.area = request.length * request.width
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ComputeRectangleAreaServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
