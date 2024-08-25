#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import SetLed
from my_robot_interfaces.msg import LedStatus


class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")
        self.declare_parameter("led_states", [0, 0, 0])

        self.led_panel_state_ = self.get_parameter("led_states").value
        self.led_panel_state_publisher_ = self.create_publisher(
            LedStatus, "led_panel_state", 10)
        self.led_panel_state_timer_ = self.create_timer(
            4.0, self.publish_led_panel_state)
        self.set_led_server_ = self.create_service(
            SetLed, "/set_led", self.callback_set_led)
        self.get_logger().info("Led Panel Node Started.")

    def publish_led_panel_state(self):
        msg = LedStatus()
        msg.led_state = self.led_panel_state_
        self.led_panel_state_publisher_.publish(msg)

    def callback_set_led(self, request, response):
        if request.is_battery_empty is True:
            response.success = True
            response.state_message = "Led set to on."
            self.led_panel_state_ = [0, 0, 1]
        else:
            response.success = True
            response.state_message = "Led set to off."
            self.led_panel_state_ = [0, 0, 0]
        self.publish_led_panel_state()
        return response

    
def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
