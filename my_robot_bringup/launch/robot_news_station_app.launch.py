from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_news_station_giskard_node = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station_giskard",
        parameters=[
            {"robot_name": "Giskard"}
        ]
    )

    robot_news_station_bb8_node = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station_bb8",
        parameters=[
            {"robot_name": "BB8"}
        ]
    )

    robot_news_station_daneel_node = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station_daneel",
        parameters=[
            {"robot_name": "Daneel"}
        ]
    )

    robot_news_station_lander_node = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station_lander",
        parameters=[
            {"robot_name": "Lander"}
        ]
    )

    robot_news_station_c3po_node = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station_c3po",
        parameters=[
            {"robot_name": "C3PO"}
        ]
    )

    smartphone_node = Node(
        package="my_py_pkg",
        executable="smartphone"
    )

    ld.add_action(robot_news_station_giskard_node)
    ld.add_action(robot_news_station_bb8_node)
    ld.add_action(robot_news_station_daneel_node)
    ld.add_action(robot_news_station_lander_node)
    ld.add_action(robot_news_station_c3po_node)
    ld.add_action(smartphone_node)    

    return ld