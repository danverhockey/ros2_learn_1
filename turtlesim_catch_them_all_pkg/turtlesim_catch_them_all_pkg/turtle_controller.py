#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import pow, atan2, sqrt
import random

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from my_robot_interfaces.msg import Color
from my_robot_interfaces.msg import ColorArray
from my_robot_interfaces.srv import ChangePenColor

 
class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.declare_parameter("nearest_turtle", False)

        self.nearest_turtle_ = self.get_parameter("nearest_turtle").value 

        self.distance_tolerance_ = 0.01

        self.turtle1_x_, self.turtle1_y_, self.turtle1_theta_ = 0.0, 0.0, 0.0
        self.turtle_new_name_, self.turtle_new_x_, self.turtle_new_y_ = "", 0.0, 0.0

        self.move_turtle1_ = False

        self.pen_colors_ = []

        self.turtle1_cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.turtle1_pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_turtle1_pose, 10)

        self.new_turtle_subsriber_ = self.create_subscription(TurtleArray, "/turtles", self.callback_new_turtle_position, 10)
        self.turtle_move_timer_ = self.create_timer(0.01, self.move_to_new_turtle)

        self.pen_color_subsriber_ = self.create_subscription(ColorArray, "/color_publish", self.callback_pen_color, 10)

        self.get_logger().info("Turtle Controller Node Started.")
        
    def callback_turtle1_pose(self, msg):
        self.turtle1_x_ = msg.x
        self.turtle1_y_ = msg.y
        self.turtle1_theta_ = msg.theta

    def euclidean_distance(self): # distance between turtle1 coordinates and new turtle coordinates 
        return sqrt(pow((self.turtle_new_x_ - self.turtle1_x_), 2) + pow((self.turtle_new_y_ - self.turtle1_y_), 2))

    def linear_velocity(self): # linear velocity to be published 
        return 2.0 * self.euclidean_distance()

    def steering_angle(self): # steering angle 
        return atan2(self.turtle_new_y_ - self.turtle1_y_, self.turtle_new_x_ - self.turtle1_x_)

    def angular_velocity(self): # angular velocity to be published
        return 10.0 * (self.steering_angle() - self.turtle1_theta_)
    
    def move_to_new_turtle(self):
        msg = Twist()
        
        if self.move_turtle1_ is False:
            return

        elif self.euclidean_distance() <= self.distance_tolerance_:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0

            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            
            self.call_turtle_catcher(self.turtle_new_name_)
            self.call_change_pen_color()
        else:
            msg.linear.x = self.linear_velocity()
            msg.linear.y = 0.0
            msg.linear.z = 0.0

            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = self.angular_velocity()

        self.turtle1_cmd_vel_publisher_.publish(msg)
    
    def turtle_array_sort_func(self, turtle):
        return sqrt(pow((turtle.x - self.turtle1_x_), 2) + pow((turtle.y - self.turtle1_y_), 2))

    def callback_new_turtle_position(self, msg):
        if not msg.turtles:
            self.move_turtle1_ = False
            self.get_logger().info("No turtles to catch!")
        else:
            self.move_turtle1_ = True
            
            if self.nearest_turtle_ is True:
                msg.turtles.sort(key=self.turtle_array_sort_func)
               
            self.turtle_new_name_ = msg.turtles[0].name
            self.turtle_new_x_ = msg.turtles[0].x
            self.turtle_new_y_ = msg.turtles[0].y
    
    def callback_pen_color(self, msg):
        self.pen_colors_ = msg.colors
    
    def call_change_pen_color(self):
        client = self.create_client(ChangePenColor, "/change_pen_color")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Change Pen Color")
        
        color = random.choice(self.pen_colors_)

        request = ChangePenColor.Request(
            color = color 
        )

        future = client.call_async(request)
        future.add_done_callback(self.callback_call_change_pen_color)
    
    def callback_call_change_pen_color(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def call_turtle_catcher(self, turtle_name):
        client = self.create_client(CatchTurtle, "/catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Catch Turtle")
        
        request = CatchTurtle.Request(
            name = turtle_name
        )

        future = client.call_async(request)
        future.add_done_callback(self.callback_call_turtle_catcher)
    
    def callback_call_turtle_catcher(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    

 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()