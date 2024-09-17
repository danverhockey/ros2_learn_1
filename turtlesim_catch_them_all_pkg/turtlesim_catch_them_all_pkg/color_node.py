#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from my_robot_interfaces.msg import Color
from my_robot_interfaces.msg import ColorArray
from my_robot_interfaces.srv import ChangePenColor
from turtlesim.srv import SetPen
 
class ColorNode(Node):
    def __init__(self):
        super().__init__("color_node")

        self.blue_ = Color(color = "blue", r = 0, g = 0, b = 255)
        self.red_ = Color(color = "red", r = 255, g = 0, b = 0)
        self.green_ = Color(color = "green", r = 0, g = 255, b = 0)
        self.purple_ = Color(color = "purple", r = 138, g = 3, b = 255)
        self.yellow_ = Color(color = "yellow", r = 255, g = 255, b = 3)
        self.pink_ = Color(color = "pink", r = 255, g = 3, b = 239)
        self.color_list_ = [self.blue_, self.red_, self.green_, self.purple_, self.yellow_, self.pink_]

        self.color_publisher_ = self.create_publisher(ColorArray, "/color_publish", 10)
        self.color_publisher_timer_ = self.create_timer(0.5, self.publish_colors)

        self.change_pen_color_server_ = self.create_service(ChangePenColor, "/change_pen_color", self.callback_change_pen_color)

        self.get_logger().info("Starting Color Node...")
    
    def publish_colors(self):
        msg = ColorArray()
        msg.colors = self.color_list_
        self.color_publisher_.publish(msg)
    
    def callback_change_pen_color(self, request, response):
        self.call_set_pen(request.color.r, request.color.g, request.color.b)
        response.message = "Changing pen color to " + request.color.color
        return response 

    def call_set_pen(self, r, g, b, width=5):
        client = self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server Set Pen...")
        
        request = SetPen.Request(
            r = r,
            g = g,
            b = b,
            width = width
        )

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_set_pen, r=r, g=g, b=b, width=width))
    
    def callback_call_set_pen(self, future, r, g, b, width):
        try:
            future.result()
            self.get_logger().info("Set Pen Parameters: " + "r: " + str(r) + " g: " + str(g) + " b: " + str(b) + " width: " + str(width))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
 
def main(args=None):
    rclpy.init(args=args)
    node = ColorNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()