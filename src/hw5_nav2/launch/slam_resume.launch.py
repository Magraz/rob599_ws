#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Resume SLAM mapping from a previously serialized pose graph.

Usage:
    ros2 launch hw5_nav2 slam_resume.launch.py map_file:=/absolute/path/to/saved_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


NODENAME = "hw5_nav2"


def generate_launch_description():

    pkg_dir = get_package_share_directory(NODENAME)
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")

    resume_params = os.path.join(pkg_dir, "config", "slam_toolbox_resume.yaml")

    # --- Launch arguments ------------------------------------------------
    map_file_arg = DeclareLaunchArgument(
        "map_file",
        description="Absolute path to the serialized pose graph (no extension)",
    )
    map_file = LaunchConfiguration("map_file")

    # Rewrite the resume config so map_file_name gets the user-supplied path.
    rewritten_params = RewrittenYaml(
        source_file=resume_params,
        root_key="",
        param_rewrites={"map_file_name": map_file},
        convert_types=True,
    )

    # --- Nodes / includes ------------------------------------------------
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
            "params_file": rewritten_params,
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

    return LaunchDescription(
        [
            map_file_arg,
            rviz,
            nav2_slam,
            turtlebot3_gazebo,
        ]
    )
