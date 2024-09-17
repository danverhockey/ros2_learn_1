from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    turtle_controller_node = Node(
        package="turtlesim_catch_them_all_pkg",
        executable="turtle_controller",
        parameters=[
            {"nearest_turtle": True}
        ]
    )

    color_node = Node(
        package="turtlesim_catch_them_all_pkg",
        executable="color_node"
    )

    turtle_spawner_node = Node(
        package="turtlesim_catch_them_all_pkg",
        executable="turtle_spawner",
        parameters=[
            {"spawn_time": 2.0}
        ]
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_controller_node)
    ld.add_action(color_node)
    ld.add_action(turtle_spawner_node)

    return ld 