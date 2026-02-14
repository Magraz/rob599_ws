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
from launch_ros.actions import Node as LaunchNode
from launch_ros.actions import Node
import yaml


def generate_launch_description():

    # Get directories
    stage_ros2_dir = get_package_share_directory("stage_ros2")
    stage_launch_dir = os.path.join(stage_ros2_dir, "launch")

    package_dir = get_package_share_directory("hw3")
    config_file = os.path.join(package_dir, "config", "graf201.yaml")

    # Load YAML config to extract world_name
    world_name = "graf201"

    # World can be overridden from command line, but defaults to YAML value
    world = LaunchConfiguration("world")
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=world_name,
        description="World to load in stage (from config or override)",
    )

    map_pub_node = Node(
        package="hw3",
        executable="map_publisher",
        name="map_publisher",
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

    vfh_follower_node = Node(
        package="hw3",
        executable="vfh_follower",
        name="vfh_follower",
        parameters=[config_file],
        output="screen",
    )

    stage_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(stage_launch_dir, "demo.launch.py")),
        launch_arguments={
            "world": world,
            "use_stamped_velocity": "true",
        }.items(),
    )

    LaunchNode(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        name="world_to_odom_tf",
    ),

    return LaunchDescription(
        [
            declare_world_cmd,
            stage_demo,
            map_pub_node,
            map_loader_node,
            vfh_follower_node,
            global_planner_node,
            path_2_waypoints_node,
        ]
    )
