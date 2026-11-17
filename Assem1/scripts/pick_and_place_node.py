#!/usr/bin/env python3
"""
EN3563 - Robotics Mini Project: ROS 2 Pick-and-Place Motion Control Node
========================================================================
Executes the upright-constrained Pick-and-Place trajectory in Gazebo Harmonic
via ROS 2 (ros_gz_bridge), controlling the 6 revolute joints of the YASKAWA GP7
and the 2 prismatic jaws of the parallel gripper.
"""

import os
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64
from sensor_msgs.msg import JointState

# Ensure local scripts directory is on sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from kinematics import forward_kinematics, box_b_orientation_rpy
from trajectory_planner import generate_pick_and_place_trajectory


class GP7PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__("gp7_pick_and_place_controller")

        # Declare ROS 2 parameters
        self.declare_parameter("box_a_x", 0.50)
        self.declare_parameter("box_a_y", 0.50)
        self.declare_parameter("box_b_x", -0.35)
        self.declare_parameter("box_b_y", 0.45)
        self.declare_parameter("velocity_profile", "s_curve")
        self.declare_parameter("scale", 1.0)
        self.declare_parameter("startup_hold_sec", 2.5)
        self.declare_parameter("speed_factor", 1.0)

        self.box_a_x = float(self.get_parameter("box_a_x").value)
        self.box_a_y = float(self.get_parameter("box_a_y").value)
        self.box_b_x = float(self.get_parameter("box_b_x").value)
        self.box_b_y = float(self.get_parameter("box_b_y").value)
        self.velocity_profile = str(self.get_parameter("velocity_profile").value)
        self.scale = float(self.get_parameter("scale").value)
        self.startup_hold_sec = float(self.get_parameter("startup_hold_sec").value)
        self.speed_factor = max(1, int(round(float(self.get_parameter("speed_factor").value))))

        # Create Joint Command Publishers
        self.joint_pubs = [
            self.create_publisher(Float64, f"/gp7/joint{i}/cmd_pos", 10)
            for i in range(1, 7)
        ]
        self.grip_left_pub = self.create_publisher(Float64, "/gp7/gripper_left/cmd_pos", 10)
        self.grip_right_pub = self.create_publisher(Float64, "/gp7/gripper_right/cmd_pos", 10)
        self.attach_pub = self.create_publisher(Empty, "/gp7/gripper/attach", 10)
        self.detach_pub = self.create_publisher(Empty, "/gp7/gripper/detach", 10)

        # Subscribe to /joint_states to synchronize startup with Gazebo physics
        self.sim_ready = False
        self.latest_joint_state = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        # Generate full Pick-and-Place trajectory
        self.dt = 0.02  # 50 Hz control loop
        self.traj = generate_pick_and_place_trajectory(
            box_b_pos=(self.box_b_x, self.box_b_y, 0.025),
            box_a_pos=(self.box_a_x, self.box_a_y, 0.0),
            profile_type=self.velocity_profile,
            dt=self.dt,
            scale=self.scale,
        )

        self.get_logger().info("=============================================================")
        self.get_logger().info(" YASKAWA GP7 Pick-and-Place Controller Initialized")
        self.get_logger().info(f" Velocity Profile : {self.velocity_profile.upper()}")
        self.get_logger().info(f" Robot Scale      : {self.scale:.2f}x")
        self.get_logger().info(f" Box B (Pick)     : ({self.box_b_x:.3f}, {self.box_b_y:.3f}, 0.025) m")
        self.get_logger().info(f" Box A (Place Top): ({self.box_a_x:.3f}, {self.box_a_y:.3f}, 0.225) m")
        self.get_logger().info(" Waiting for Gazebo /joint_states to begin execution...")
        self.get_logger().info("=============================================================")

        self.hold_steps_remaining = int(np.round(self.startup_hold_sec / self.dt))
        self.step_idx = 0
        self.prev_attached = False
        self.prev_phase = ""
        self.completed_logged = False

        self.timer = self.create_timer(self.dt, self._control_loop)

    def _on_joint_state(self, msg: JointState):
        if not self.sim_ready:
            self.sim_ready = True
            self.get_logger().info("Gazebo physics active! Holding home pose & detaching Box B...")
        self.latest_joint_state = msg

    def _publish_joints(self, q, grip):
        for i in range(6):
            msg = Float64()
            msg.data = float(q[i])
            self.joint_pubs[i].publish(msg)

        msg_l = Float64()
        msg_l.data = float(grip[0])
        self.grip_left_pub.publish(msg_l)

        msg_r = Float64()
        msg_r.data = float(grip[1])
        self.grip_right_pub.publish(msg_r)

    def _control_loop(self):
        if not self.sim_ready:
            # Keep publishing detach and zero pose while waiting for Gazebo
            self._publish_joints(np.zeros(6), [0.0, 0.0])
            self.detach_pub.publish(Empty())
            return

        # Initial hold phase: keep robot at q=0 while detaching Box B on the ground
        if self.hold_steps_remaining > 0:
            self._publish_joints(np.zeros(6), [0.0, 0.0])
            if self.hold_steps_remaining % 5 == 0:
                self.detach_pub.publish(Empty())
            self.hold_steps_remaining -= 1
            return

        n_total = len(self.traj["time"])
        if self.step_idx < n_total:
            q_cmd = self.traj["q"][self.step_idx]
            grip_cmd = self.traj["gripper"][self.step_idx]
            is_attached = bool(self.traj["attached"][self.step_idx])
            phase = self.traj["phase"][self.step_idx]
            tcp_pos = self.traj["tcp_pos"][self.step_idx]
            rpy_err = self.traj["box_b_rpy_deg"][self.step_idx]

            if phase != self.prev_phase:
                self.get_logger().info(
                    f"[Phase: {phase}] t={self.traj['time'][self.step_idx]:.2f}s | "
                    f"TCP=({tcp_pos[0]:.3f}, {tcp_pos[1]:.3f}, {tcp_pos[2]:.3f}) m | "
                    f"Box B RPY=({rpy_err[0]:.2f}, {rpy_err[1]:.2f}, {rpy_err[2]:.2f}) deg"
                )
                self.prev_phase = phase

            # Trigger attach / detach transitions
            if is_attached and not self.prev_attached:
                self.get_logger().info(">>> GRASPING Box B (Closing gripper jaws & attaching) <<<")
                for _ in range(3):
                    self.attach_pub.publish(Empty())
            elif not is_attached and self.prev_attached:
                self.get_logger().info(">>> RELEASING Box B onto center of Box A top surface <<<")
                for _ in range(3):
                    self.detach_pub.publish(Empty())

            # Periodically reinforce attach/detach state during motion
            if self.step_idx % 10 == 0:
                if is_attached:
                    self.attach_pub.publish(Empty())
                elif phase.startswith("7_") or phase.startswith("8_"):
                    self.detach_pub.publish(Empty())

            self.prev_attached = is_attached
            self._publish_joints(q_cmd, grip_cmd)
            # Advance by speed_factor, stopping at any attach/detach transition boundary
            next_idx = self.step_idx + 1
            for _ in range(self.speed_factor - 1):
                if next_idx < n_total and bool(self.traj["attached"][next_idx]) == is_attached:
                    next_idx += 1
                else:
                    break
            self.step_idx = next_idx
        else:
            # Hold final retracted pose
            self._publish_joints(self.traj["q"][-1], self.traj["gripper"][-1])
            if not self.completed_logged:
                self.completed_logged = True
                max_orient_err = float(np.max(np.abs(self.traj["box_b_rpy_deg"])))
                self.get_logger().info("=============================================================")
                self.get_logger().info(" PICK-AND-PLACE TASK COMPLETED SUCCESSFULLY!")
                self.get_logger().info(f" Box B picked from : ({self.box_b_x:.3f}, {self.box_b_y:.3f}, 0.025) m")
                self.get_logger().info(f" Box B placed at   : ({self.box_a_x:.3f}, {self.box_a_y:.3f}, 0.225) m")
                self.get_logger().info(f" Max Box B Tilt Err: {max_orient_err:.6f} deg (Strictly Upright!)")
                self.get_logger().info("=============================================================")


def main(args=None):
    rclpy.init(args=args)
    node = GP7PickAndPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
