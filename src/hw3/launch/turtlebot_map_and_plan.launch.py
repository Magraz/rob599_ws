#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
)
from launch_ros.actions import Node
import yaml


def generate_launch_description():

    package_dir = get_package_share_directory("hw3")
    config_file = os.path.join(package_dir, "config", "turtlebot.yaml")
    rviz_config = os.path.join(package_dir, "config", "turtlebot.rviz")

    global_planner_node = Node(
        package="hw3",
        executable="global_planner",
        name="global_planner",
        parameters=[config_file],
        output="screen",
    )

    path_2_waypoints_node = Node(
        package="hw3",
        executable="path_2_waypoints",
        name="path_2_waypoints",
        parameters=[config_file],
        output="screen",
    )

    map_loader_node = Node(
        package="hw3",
        executable="map_loader",
        name="map_loader",
        parameters=[config_file],
        output="screen",
    )

    vfh_follower_node = Node(
        package="hw3",
        executable="vfh_follower_turtlebot",
        name="vfh_follower_turtlebot",
        parameters=[config_file],
        output="screen",
    )

    vfh_delayed = TimerAction(
        period=2.0,
        actions=[vfh_follower_node],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        [
            rviz_node,
            map_loader_node,
            global_planner_node,
            path_2_waypoints_node,
            vfh_delayed,
        ]
    )
