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
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


PACKAGE = "hw4_localization"
DEFAULT_WORLD = "graf201"


def generate_launch_description():
    pkg_dir = get_package_share_directory(PACKAGE)

    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    num_particles = LaunchConfiguration("num_particles")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")

    # ── Stage simulator (no RViz from demo — we launch our own) ─
    stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "demo.launch.py")
        ),
        launch_arguments={
            "world": world,
            "use_stamped_velocity": "true",
            "rviz": "false",
        }.items(),
    )

    # ── RViz with MCL config ────────────────────────────────────
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "rviz.launch.py")
        ),
        launch_arguments={
            "config": "mcl",
        }.items(),
    )

    # ── Map server (lifecycle-managed) ──────────────────────────
    # map_yaml = os.path.join(pkg_dir, "world", "bitmaps", f"{world}.yaml")
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
                "use_sim_time": True,
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
                "laser_max_range": 10.0,
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

    # ── Publish initial pose after 7 s ──────────────────────────
    yaw_deg = 45.0
    yaw_rad = math.radians(yaw_deg)
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    initial_pose_yaml = (
        "{"
        "header: {frame_id: map}, "
        "pose: {pose: {"
        f"position: {{x: -4.00, y: -6.00, z: 0.0}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz:.5f}, w: {qw:.5f}}}"
        "}, covariance: ["
        "4.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 4.0, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.0, 0.0, 0.0, 0.0, 0.5"
        "]}"
        "}"
    )

    initial_pose_pub = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/initialpose",
                    "geometry_msgs/msg/PoseWithCovarianceStamped",
                    initial_pose_yaml,
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=DEFAULT_WORLD),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("num_particles", default_value="2000"),
            DeclareLaunchArgument("scan_topic", default_value="/base_scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            stage,
            rviz,
            map_server_group,
            mcl_node,
            initial_pose_pub,
        ]
    )
