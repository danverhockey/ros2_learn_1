#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.counter_ = 0
        self.counter_publisher_ = self.create_publisher(
            Int64, "number_counter", 10)
        self.number_subscriber_ = self.create_subscription(
            Int64, "number", self.callback_number, 10)
        self.reset_counter_server_ = self.create_service(
            SetBool, "/reset_counter", self.callback_reset_counter)
        # self.counter_timer_ = self.create_timer(0.5, self.publish_counter)
        self.get_logger().info("Number Counter Node started.")

    def callback_number(self, msg):
        #self.get_logger().info(str(self.counter_))
        self.counter_ += msg.data
        new_msg = Int64()
        new_msg.data = self.counter_
        self.counter_publisher_.publish(new_msg)

    def callback_reset_counter(self, request, response):
        if request.data is True:
            self.counter_ = 0
            response.success = True
            response.message = "Counter reset to 0."
        else:
            response.success = False
            response.message = "Counter not reset to 0."
        return response 

    '''def publish_counter(self):
        msg = Int64()
        msg.data = self.counter_
        self.publisher_.publish(msg)'''


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
