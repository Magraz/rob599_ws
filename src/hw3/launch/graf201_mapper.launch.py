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
)
from launch_ros.actions import Node
import yaml


def generate_launch_description():

    # Get directories
    stage_ros2_dir = get_package_share_directory("stage_ros2")
    stage_launch_dir = os.path.join(stage_ros2_dir, "launch")

    package_dir = get_package_share_directory("hw3")
    config_file = os.path.join(package_dir, "config", "graf201.yaml")

    world_name = "graf201"

    # World can be overridden from command line, but defaults to YAML value
    world = LaunchConfiguration("world")
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=world_name,
        description="World to load in stage (from config or override)",
    )

    bayesian_mapper_node = Node(
        package="hw3",
        executable="bayesian_mapper",
        name="bayesian_mapper",
        parameters=[config_file],
        output="screen",
    )

    map_pub_node = Node(
        package="hw3",
        executable="map_publisher",
        name="map_publisher",
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

    # Include stage_ros2 demo launch file
    stage_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(stage_launch_dir, "demo.launch.py")),
        launch_arguments={
            "world": world,
            "use_stamped_velocity": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            declare_world_cmd,
            stage_demo,
            map_pub_node,
            map_saver_node,
            bayesian_mapper_node,
        ]
    )
