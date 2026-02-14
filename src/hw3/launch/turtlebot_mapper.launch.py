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

    bayesian_mapper_node = Node(
        package="hw3",
        executable="bayesian_mapper",
        name="bayesian_mapper",
        parameters=[config_file],
        output="screen",
    )

    map_saver_node = Node(
        package="hw3",
        executable="map_saver",
        name="map_saver",
        parameters=[config_file],
        output="screen",
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
            bayesian_mapper_node,
            map_saver_node,
        ]
    )
