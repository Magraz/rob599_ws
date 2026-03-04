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


NODENAME = "hw4_localization"


def generate_launch_description():

    pkg_dir = get_package_share_directory(NODENAME)
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    slam_dir = get_package_share_directory("slam_toolbox")

    slam_params = os.path.join(pkg_dir, "config", "slam_toolbox_turtlebot.yaml")

    # ── RViz with MCL config ────────────────────────────────────
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "rviz.launch.py")
        ),
        launch_arguments={
            "config": "mcl",
        }.items(),
    )

    nav2_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "slam_launch.py")
        ),
        launch_arguments={
            "params_file": slam_params,
            "use_sim_time": "false",
            "autostart": "true",
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "params_file": slam_params,
            "use_sim_time": "false",
            "autostart": "true",
        }.items(),
    )

    # Nav2 navigation: planner, controller, bt_navigator, behaviors, etc.
    return LaunchDescription(
        [
            rviz,
            slam,
        ]
    )
