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
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")

    slam_params = os.path.join(pkg_dir, "config", "slam_toolbox.yaml")

    # RViz with a single shared config for all worlds
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "rviz.launch.py")
        ),
        launch_arguments={
            "config": "hw5_slam",
        }.items(),
    )

    nav2_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "slam_launch.py")
        ),
        launch_arguments={
            "params_file": slam_params,
            "use_sim_time": "true",
            "autostart": "true",
        }.items(),
    )

    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, "launch", "turtlebot3_house.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    # Nav2 navigation: planner, controller, bt_navigator, behaviors, etc.
    return LaunchDescription(
        [
            rviz,
            nav2_slam,
            turtlebot3_gazebo,
        ]
    )
