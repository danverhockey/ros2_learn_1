#!/usr/bin/env python3
import rclpy
import time 
from rclpy.node import Node
from functools import partial

from my_robot_interfaces.srv import SetLed


class BatteryClientNode(Node):
    def __init__(self):
        super().__init__("battery_client")
        self.battery_indicator_ = False
        while True:
            self.count_ = 4
            print(f"Battery Status: {self.count_}/4")
            while self.count_ > 0:
                time.sleep(1)
                self.count_ -= 1
                print(f"Battery Status: {self.count_}/4")
            if self.count_ == 0:
                self.battery_indicator_ = True
                self.call_set_led_server(self.battery_indicator_)
                print("Led Set On")
            print(f"Battery Charge Status: {self.count_}/6")
            while self.count_ < 6:
                time.sleep(1)
                self.count_ += 1
                print(f"Battery Charge Status: {self.count_}/6")
            if self.count_ == 6:
                self.battery_indicator_ = False
                self.call_set_led_server(self.battery_indicator_)
                print("Led Set Off")

    def call_set_led_server(self, is_battery_empty):
        client = self.create_client(SetLed, "/set_led")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Led...")

        request = SetLed.Request()
        request.is_battery_empty = is_battery_empty

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_set_led_server, is_battery_empty=is_battery_empty))

    def callback_call_set_led_server(self, future, is_battery_empty):
        try:
            response = future.result()
            self.get_logger().info("Request sent: " + str(is_battery_empty) + ", Result: " +
                                   str(response.success) + ", " + str(response.state_message))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
