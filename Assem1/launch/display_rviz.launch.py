#!/usr/bin/env python3
"""
EN3563 - Robotics Mini Project: RViz2 Coordinate Frame & Model Visualizer Launch File
====================================================================================
Launches:
  1. robot_state_publisher (with either 'cad' or 'simplified' URDF)
  2. joint_state_publisher_gui (interactive joint sliders)
  3. rviz2 (with all link coordinate frames labeled and visible)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("Assem1")
    model_type = LaunchConfiguration("model_type").perform(context).strip().lower()

    if model_type == "simplified":
        urdf_file = os.path.join(pkg_share, "urdf", "Assem1_simplified.urdf")
    else:
        urdf_file = os.path.join(pkg_share, "urdf", "Assem1.urdf")

    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_description_content = f.read()

    rviz_config = os.path.join(pkg_share, "config", "gp7_frames.rviz")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description_content}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_type",
            default_value="cad",
            description="'cad' (Full SolidWorks CAD model) or 'simplified' (Simplified D-H model)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
