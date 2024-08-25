#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random 

from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
 
class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")

        self.declare_parameter("spawn_time", 15.0)

        self.spawn_time_ = self.get_parameter("spawn_time").value

        self.turtle_array_ = []
        self.turtle_x_ = 0.0
        self.turtle_y_ = 0.0
       
        self.turtle_array_publisher_ = self.create_publisher(TurtleArray, "/turtles", 10)
        self.turtle_spawn_timer_ = self.create_timer(self.spawn_time_, self.call_spawn)
        self.catch_turtle_server_ = self.create_service(CatchTurtle, "/catch_turtle", self.callback_catch_turtle)

        self.get_logger().info("Turtle Spawner Node Started...")
        
    def publish_turtle_array(self):
        msg = TurtleArray()
        msg.turtles = self.turtle_array_
        self.turtle_array_publisher_.publish(msg)

    def call_spawn(self):
        client = self.create_client(Spawn, "/spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server Spawn...")
        
        request = Spawn.Request(
            x = random.uniform(0.0, 11.0),
            y = random.uniform(0.0, 11.0),
            theta = 0.0
        )

        self.turtle_x_ = request.x
        self.turtle_y_ = request.y

        future = client.call_async(request)
        future.add_done_callback(self.callback_call_spawn)

    def callback_call_spawn(self, future):
        try:
            response = future.result()
            self.get_logger().info(response.name + " has spawned!")

            turtle = Turtle(
                name = response.name,
                x = self.turtle_x_,
                y = self.turtle_y_
            )

            self.turtle_array_.append(turtle)
            self.publish_turtle_array()

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def call_kill(self, name):
        client = self.create_client(Kill, "/kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server Kill...")
        
        request = Kill.Request()
        request.name = name 

        future = client.call_async(request)
        future.add_done_callback(self.callback_call_kill)
    
    def callback_call_kill(self, future):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def callback_catch_turtle(self, request, response):
            turtle_exists = False
            for turtle in self.turtle_array_:
                if request.name == turtle.name:
                    turtle_exists = True
                    self.turtle_array_.remove(turtle)
                    self.publish_turtle_array()
                    break 
            if turtle_exists is True: 
                self.call_kill(request.name)
                response.success = True
                response.message = request.name + " was caught!"
                self.get_logger().info(request.name + " was caught!")
            else:
                response.success = False
                response.message = request.name + " does not exist"
                self.get_logger().info(request.name + " does not exist")
            return response 
 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()