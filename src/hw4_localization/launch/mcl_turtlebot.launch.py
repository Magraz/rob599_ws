#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Launch Stage simulation with custom MCL (Monte Carlo Localization) node.

Launches:
  - Stage simulator + RViz  (via demo.launch.py)
  - nav2_map_server          (serves the static map on /map)
  - mcl node                  (custom particle filter localization)
  - Initial pose publisher    (seeds MCL particles after 7 s)
"""
import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


PACKAGE = "hw4_localization"


def generate_launch_description():
    pkg_dir = get_package_share_directory(PACKAGE)

    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    num_particles = LaunchConfiguration("num_particles")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")

    # ── RViz with MCL config ────────────────────────────────────
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "rviz.launch.py")
        ),
        launch_arguments={
            "config": "mcl",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # ── Map server (lifecycle-managed) ──────────────────────────
    map_yaml = [pkg_dir, "/world/bitmaps/", LaunchConfiguration("world"), ".yaml"]

    map_server_group = GroupAction(
        [
            SetParameter(name="use_sim_time", value=use_sim_time),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"yaml_filename": map_yaml}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[{"autostart": True, "node_names": ["map_server"]}],
            ),
        ]
    )

    # ── MCL node ────────────────────────────────────────────────
    mcl_node = Node(
        package=PACKAGE,
        executable="mcl",
        name="mcl",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "num_particles": num_particles,
                "scan_topic": scan_topic,
                "odom_topic": odom_topic,
                "alpha1": 0.2,
                "alpha2": 0.2,
                "alpha3": 0.2,
                "alpha4": 0.2,
                "sigma_hit": 0.5,
                "z_hit": 0.5,
                "z_rand": 0.5,
                "laser_max_range": 3.5,
                "max_beams": 60,
                "update_min_d": 0.2,
                "update_min_a": 0.2,
                "tf_broadcast": True,
                "global_frame_id": "map",
                "odom_frame_id": "odom",
                "base_frame_id": "base_link",
            }
        ],
        # prefix=["python3 -m debugpy --listen 5678 --wait-for-client"],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="office"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("num_particles", default_value="2000"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            rviz,
            map_server_group,
            mcl_node,
        ]
    )
