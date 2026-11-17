#!/usr/bin/env python3
"""
EN3563 - Robotics Mini Project: Gazebo Harmonic + ROS 2 Jazzy Launch File
=========================================================================
Launches:
  1. Gazebo Harmonic (gz sim) with industrial workcell world
  2. YASKAWA GP7 6-DoF Robot Arm at Origin (0, 0, 0)
  3. Box A (30 cm x 30 cm x 20 cm) and Box B (5 cm x 5 cm x 5 cm)
  4. ROS 2 <-> Gazebo Bridge (parameter_bridge)
  5. Robot State Publisher (TF frames for all links)
  6. GP7 Pick-and-Place Trajectory & Velocity Control Node
"""

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_box_a_sdf():
    """Generates SDF for Box A (30 cm x 30 cm x 20 cm)."""
    return """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="box_a">
    <static>true</static>
    <link name="box_a_link">
      <collision name="collision">
        <geometry><box><size>0.30 0.30 0.20</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.30 0.30 0.20</size></box></geometry>
        <material>
          <ambient>0.18 0.55 0.34 1</ambient>
          <diffuse>0.18 0.55 0.34 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
      <visual name="target_bullseye">
        <pose>0 0 0.1005 0 0 0</pose>
        <geometry><cylinder><radius>0.04</radius><length>0.001</length></cylinder></geometry>
        <material>
          <ambient>0.95 0.85 0.15 1</ambient>
          <diffuse>0.95 0.85 0.15 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def make_box_b_sdf():
    """Generates SDF for Box B (5 cm x 5 cm x 5 cm)."""
    return """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="box_b">
    <link name="box_b_link">
      <inertial>
        <mass>0.08</mass>
        <inertia>
          <ixx>3.33e-5</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>3.33e-5</iyy><iyz>0</iyz><izz>3.33e-5</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
        <material>
          <ambient>0.88 0.18 0.18 1</ambient>
          <diffuse>0.88 0.18 0.18 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>
      <visual name="top_up_marker">
        <pose>0 0 0.0255 0 0 0</pose>
        <geometry><box><size>0.03 0.03 0.001</size></box></geometry>
        <material>
          <ambient>0.98 0.92 0.20 1</ambient>
          <diffuse>0.98 0.92 0.20 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("Assem1")
    share_parent = os.path.dirname(pkg_share)

    # Add scripts directory to import random reachable sampler
    scripts_dir = os.path.join(pkg_share, "scripts")
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    from trajectory_planner import sample_random_reachable_box_b

    model_type = LaunchConfiguration("model_type").perform(context).strip().lower()
    velocity_profile = LaunchConfiguration("velocity_profile").perform(context).strip().lower()
    box_a_x = float(LaunchConfiguration("box_a_x").perform(context).strip())
    box_a_y = float(LaunchConfiguration("box_a_y").perform(context).strip())
    random_box_b_str = LaunchConfiguration("random_box_b").perform(context).strip().lower()
    box_b_x_str = LaunchConfiguration("box_b_x").perform(context).strip()
    box_b_y_str = LaunchConfiguration("box_b_y").perform(context).strip()
    headless_str = LaunchConfiguration("headless").perform(context).strip().lower()
    use_rviz_str = LaunchConfiguration("use_rviz").perform(context).strip().lower()
    speed_factor = float(LaunchConfiguration("speed_factor").perform(context).strip())

    # Select full SolidWorks CAD URDF or simplified D-H URDF
    if model_type == "simplified":
        urdf_file = os.path.join(pkg_share, "urdf", "Assem1_simplified.urdf")
    else:
        urdf_file = os.path.join(pkg_share, "urdf", "Assem1.urdf")

    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_description_content = f.read()

    # Determine Box B reachable ground coordinate
    if random_box_b_str in ("true", "1", "yes"):
        box_b_pos = sample_random_reachable_box_b(box_a_xy=(box_a_x, box_a_y), scale=1.0)
        box_b_x, box_b_y = float(box_b_pos[0]), float(box_b_pos[1])
    else:
        box_b_x = float(box_b_x_str)
        box_b_y = float(box_b_y_str)

    world_file = os.path.join(pkg_share, "worlds", "empty_workcell.sdf")
    gz_args = ["-s", "-r", world_file] if headless_str in ("true", "1", "yes") else ["-r", world_file]

    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    new_gz_path = f"{share_parent}:{pkg_share}:{existing_gz_path}" if existing_gz_path else f"{share_parent}:{pkg_share}"

    actions = [
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=new_gz_path),
        ExecuteProcess(
            cmd=["gz", "sim"] + gz_args,
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description_content}],
        ),
        # Spawn YASKAWA GP7 Robot Arm at Origin (0, 0, 0)
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_gp7_robot",
            output="screen",
            arguments=[
                "-name", "Assem1",
                "-string", robot_description_content,
                "-x", "0.0", "-y", "0.0", "-z", "0.0",
            ],
        ),
        # Spawn Box A (30 cm x 30 cm x 20 cm) on ground at (0.5 m, 0.5 m, 0) -> center z=0.10 m
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_box_a",
            output="screen",
            arguments=[
                "-name", "box_a",
                "-string", make_box_a_sdf(),
                "-x", str(box_a_x),
                "-y", str(box_a_y),
                "-z", "0.10",
            ],
        ),
        # Spawn Box B (5 cm x 5 cm x 5 cm) at random reachable ground location -> center z=0.025 m
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_box_b",
            output="screen",
            arguments=[
                "-name", "box_b",
                "-string", make_box_b_sdf(),
                "-x", str(box_b_x),
                "-y", str(box_b_y),
                "-z", "0.025",
            ],
        ),
        # Bridge ROS 2 and Gazebo Harmonic topics
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="screen",
            arguments=[
                "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                "/gp7/joint1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
                "/gp7/joint2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
                "/gp7/joint3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
                "/gp7/joint4/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
                "/gp7/joint5/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
                "/gp7/joint6/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
                "/gp7/gripper_left/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
                "/gp7/gripper_right/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
                "/gp7/gripper/attach@std_msgs/msg/Empty]gz.msgs.Empty",
                "/gp7/gripper/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            ],
        ),
        # Launch GP7 Pick-and-Place Trajectory & Velocity Control Node
        Node(
            package="Assem1",
            executable="pick_and_place_node.py",
            name="gp7_pick_and_place_controller",
            output="screen",
            parameters=[{
                "box_a_x": box_a_x,
                "box_a_y": box_a_y,
                "box_b_x": box_b_x,
                "box_b_y": box_b_y,
                "velocity_profile": velocity_profile,
                "scale": 1.0,
                "speed_factor": speed_factor,
            }],
        ),
    ]

    if use_rviz_str in ("true", "1", "yes"):
        rviz_config = os.path.join(pkg_share, "config", "gp7_frames.rviz")
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_type",
            default_value="cad",
            description="'cad' (Full SolidWorks STL model) or 'simplified' (Simplified D-H cylinder/box model)",
        ),
        DeclareLaunchArgument(
            "velocity_profile",
            default_value="s_curve",
            description="'s_curve' (Quintic S-Curve) or 'trapezoidal' (LSPB Trapezoidal Velocity Profile)",
        ),
        DeclareLaunchArgument(
            "box_a_x",
            default_value="0.50",
            description="X coordinate of Box A on ground (default 0.50 m)",
        ),
        DeclareLaunchArgument(
            "box_a_y",
            default_value="0.50",
            description="Y coordinate of Box A on ground (default 0.50 m)",
        ),
        DeclareLaunchArgument(
            "random_box_b",
            default_value="true",
            description="Sample a random reachable ground position for Box B on launch",
        ),
        DeclareLaunchArgument(
            "box_b_x",
            default_value="-0.35",
            description="Manual X coordinate for Box B if random_box_b:=false",
        ),
        DeclareLaunchArgument(
            "box_b_y",
            default_value="0.45",
            description="Manual Y coordinate for Box B if random_box_b:=false",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo server headless (-s) without GUI window",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz2 alongside Gazebo to inspect TF coordinate frames",
        ),
        DeclareLaunchArgument(
            "speed_factor",
            default_value="1.0",
            description="Playback speed multiplier (1.0 = normal real-time)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
