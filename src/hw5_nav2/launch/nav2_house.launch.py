#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Launch Stage simulation of the polkadot world with Nav2 + SLAM Toolbox.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


NODENAME = "hw5_nav2"


def generate_launch_description():

    pkg_dir = get_package_share_directory(NODENAME)
    map_yaml = os.path.join(pkg_dir, "maps", "house.yaml")
    nav2_params = os.path.join(pkg_dir, "config", "nav2_params.yaml")
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")

    # RViz with a single shared config for all worlds
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "rviz.launch.py")
        ),
        launch_arguments={
            "config": "hw5_slam",
        }.items(),
    )

    nav2_min_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "nav2_minimal.launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "params_file": nav2_params,
            "use_sim_time": "true",
        }.items(),
    )

    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, "launch", "turtlebot3_house.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "x_pose": "0.0",
            "y_pose": "-1.0",
        }.items(),
    )

    # Nav2 navigation: planner, controller, bt_navigator, behaviors, etc.
    return LaunchDescription(
        [
            rviz,
            nav2_min_launch,
            turtlebot3_gazebo,
        ]
    )
