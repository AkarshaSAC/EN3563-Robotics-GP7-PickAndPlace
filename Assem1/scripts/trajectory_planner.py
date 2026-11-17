#!/usr/bin/env python3
"""
EN3563 - Robotics Mini Project: Trajectory Planning & Smooth Velocity Profiles
===============================================================================
Implements:
1. Smooth Velocity Profiles:
   - Trapezoidal Velocity Profile (LSPB: Linear Segments with Parabolic Blends)
   - S-Curve Velocity Profile (Quintic Polynomial with zero boundary acceleration)
2. Upright-Constrained Pick-and-Place Trajectory Planner:
   - Maintains Box B upright (Roll = 0 deg, Pitch = 0 deg, Yaw = 0 deg) during
     the entire motion from random reachable ground coordinate to top center of Box A.
   - Computes continuous joint positions q(t) via analytical IK and joint velocities
     q_dot(t) via the Geometric Manipulator Jacobian J(q).
"""

import numpy as np
from kinematics import (
    R_UPRIGHT_DEFAULT,
    box_b_orientation_rpy,
    compute_joint_velocities,
    forward_kinematics,
    inverse_kinematics,
)


# =====================================================================
# 1. SMOOTH VELOCITY PROFILE GENERATORS
# =====================================================================
def s_curve_profile(t, T):
    """
    Quintic S-Curve normalized motion profile s(t) in [0, 1] for t in [0, T].
    Guarantees continuous position, bell-shaped velocity, and zero endpoint
    acceleration: s(0)=0, s(T)=1, s_dot(0)=s_dot(T)=0, s_ddot(0)=s_ddot(T)=0.
    Returns: (s, s_dot, s_ddot, s_dddot)
    """
    if T <= 0.0:
        return 1.0, 0.0, 0.0, 0.0
    tau = np.clip(t / T, 0.0, 1.0)
    s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    s_dot = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / T
    s_ddot = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / (T**2)
    s_dddot = (60.0 - 360.0 * tau + 360.0 * tau**2) / (T**3)
    return float(s), float(s_dot), float(s_ddot), float(s_dddot)


def trapezoidal_profile(t, T, blend_fraction=0.25):
    """
    Symmetric Trapezoidal Velocity Profile (LSPB) s(t) in [0, 1] for t in [0, T].
    blend_fraction alpha_b in (0, 0.5) specifies the acceleration/deceleration
    duration t_b = alpha_b * T.
    Returns: (s, s_dot, s_ddot, s_dddot)
    """
    if T <= 0.0:
        return 1.0, 0.0, 0.0, 0.0
    alpha_b = np.clip(blend_fraction, 0.05, 0.45)
    t = np.clip(t, 0.0, T)
    t_b = alpha_b * T
    v_max = 1.0 / (T * (1.0 - alpha_b))
    a_max = v_max / t_b

    if t <= t_b:
        s = 0.5 * a_max * (t**2)
        s_dot = a_max * t
        s_ddot = a_max
    elif t <= T - t_b:
        s = 0.5 * a_max * (t_b**2) + v_max * (t - t_b)
        s_dot = v_max
        s_ddot = 0.0
    else:
        dt = T - t
        s = 1.0 - 0.5 * a_max * (dt**2)
        s_dot = a_max * dt
        s_ddot = -a_max

    return float(s), float(s_dot), float(s_ddot), 0.0


def evaluate_profile(t, T, profile_type="s_curve"):
    """Evaluates either 's_curve' or 'trapezoidal' velocity profile at time t."""
    if profile_type.lower().startswith("trap"):
        return trapezoidal_profile(t, T)
    return s_curve_profile(t, T)


# =====================================================================
# 2. RANDOM REACHABLE GROUND COORDINATE GENERATOR FOR BOX B
# =====================================================================
def sample_random_reachable_box_b(box_a_xy=(0.50, 0.50), scale=1.0, seed=None):
    """
    Samples a random reachable coordinate (x_B, y_B, z_B=0.025) on the ground
    for Box B (5 cm x 5 cm x 5 cm), ensuring:
      1. Radial distance r in [0.38 * scale, 0.68 * scale] (well inside GP7 reach)
      2. Safe clearance (>= 0.30 m) from Box A (30 cm x 30 cm x 20 cm)
      3. Exact IK feasibility at both pick height (z=0.025 m) and hover height
    """
    rng = np.random.default_rng(seed)
    r_min = 0.38 * scale
    r_max = 0.66 * scale

    for _ in range(200):
        r = rng.uniform(r_min, r_max)
        # Sample angle across the reachable workspace
        angle = rng.uniform(-0.75 * np.pi, 0.85 * np.pi)
        xb = float(np.round(r * np.cos(angle), 3))
        yb = float(np.round(r * np.sin(angle), 3))

        # Ensure Box B does not overlap Box A (30 cm x 30 cm footprint + margin)
        if np.hypot(xb - box_a_xy[0], yb - box_a_xy[1]) < 0.35 * scale:
            continue

        try:
            inverse_kinematics([xb, yb, 0.025], R_UPRIGHT_DEFAULT, scale=scale)
            inverse_kinematics([xb, yb, 0.30 * scale], R_UPRIGHT_DEFAULT, scale=scale)
            return np.array([xb, yb, 0.025], dtype=float)
        except ValueError:
            continue

    return np.array([-0.42 * scale, 0.32 * scale, 0.025], dtype=float)


# =====================================================================
# 3. FULL PICK-AND-PLACE TRAJECTORY PLANNER (UPRIGHT BOX B)
# =====================================================================
def generate_pick_and_place_trajectory(
    box_b_pos,
    box_a_pos=(0.50, 0.50, 0.0),
    box_a_size=(0.30, 0.30, 0.20),
    box_b_size=0.05,
    profile_type="s_curve",
    dt=0.02,
    scale=1.0,
):
    """
    Generates the complete time-parameterized Pick-and-Place trajectory.

    Key Task Geometry:
      - Robot base at Origin: (0, 0, 0)
      - Box A (0.30 m x 0.30 m x 0.20 m) on ground at (x_A, y_A, 0)
        -> Top surface center is at (x_A, y_A, 0.20 m)
      - Box B (0.05 m x 0.05 m x 0.05 m) on ground at (x_B, y_B, 0)
        -> Initial center of Box B is at z_pick = 0.025 m
        -> Final center of Box B on top of Box A is at z_place = 0.20 + 0.025 = 0.225 m
      - Orientation of Box B is maintained strictly upright (R_0^6 = R_UPRIGHT_DEFAULT,
        so Roll = 0 deg, Pitch = 0 deg, Yaw = 0 deg) during the entire motion.

    Returns a dictionary of time-series arrays for simulation and plotting.
    """
    xb, yb = float(box_b_pos[0]), float(box_b_pos[1])
    xa, ya = float(box_a_pos[0]), float(box_a_pos[1])
    z_pick = box_b_size / 2.0                           # 0.025 m
    z_place = box_a_size[2] + box_b_size / 2.0          # 0.20 + 0.025 = 0.225 m
    z_hover_pick = max(0.26 * scale, z_pick + 0.22)     # Safe hover above Box B
    z_cruise = max(0.34 * scale, z_place + 0.12)        # Safe cruise above Box A

    p_home = np.array([-0.42 * scale, 0.0, 0.42 * scale], dtype=float)
    p_hover_pick = np.array([xb, yb, z_hover_pick], dtype=float)
    p_pick = np.array([xb, yb, z_pick], dtype=float)
    p_lift = np.array([xb, yb, z_cruise], dtype=float)
    p_hover_place = np.array([xa, ya, z_cruise], dtype=float)
    p_place = np.array([xa, ya, z_place], dtype=float)
    p_retract = np.array([xa, ya, z_cruise], dtype=float)

    # Gripper jaw positions:
    #   Open:   q7 =  0.000 m, q8 =  0.000 m (7.0 cm jaw gap)
    #   Closed: q7 = -0.010 m, q8 = +0.010 m (5.0 cm jaw gap, flush on 5 cm Box B)
    GRIP_OPEN = (0.0, 0.0)
    GRIP_CLOSED = (-0.010, 0.010)

    # Define sequential motion phases:
    # (phase_name, duration_s, start_pt, end_pt, interp_mode, grip_start, grip_end, attached)
    phases = [
        ("1_Approach_Hover_Pick", 2.5, p_home,        p_hover_pick,  "cylindrical", GRIP_OPEN,   GRIP_OPEN,   False),
        ("2_Descend_To_Pick",     1.8, p_hover_pick,  p_pick,        "cartesian",   GRIP_OPEN,   GRIP_OPEN,   False),
        ("3_Grasp_Box_B",         1.0, p_pick,        p_pick,        "hold",        GRIP_OPEN,   GRIP_CLOSED, True),
        ("4_Vertical_Lift",       2.0, p_pick,        p_lift,        "cartesian",   GRIP_CLOSED, GRIP_CLOSED, True),
        ("5_Upright_Transit",     3.5, p_lift,        p_hover_place, "cylindrical", GRIP_CLOSED, GRIP_CLOSED, True),
        ("6_Descend_To_Box_A",    2.0, p_hover_place, p_place,       "cartesian",   GRIP_CLOSED, GRIP_CLOSED, True),
        ("7_Release_Box_B",       1.0, p_place,       p_place,       "hold",        GRIP_CLOSED, GRIP_OPEN,   False),
        ("8_Retract_Upward",      1.8, p_place,       p_retract,     "cartesian",   GRIP_OPEN,   GRIP_OPEN,   False),
    ]

    times = []
    phase_labels = []
    s_vals, s_dot_vals, s_ddot_vals = [], [], []
    tcp_positions = []
    tcp_velocities = []
    joint_positions = []
    joint_velocities = []
    gripper_positions = []
    attached_flags = []
    box_b_rpy_deg = []

    t_global = 0.0
    q_zero = np.zeros(6, dtype=float)
    q_home = inverse_kinematics(p_home, R_UPRIGHT_DEFAULT, q_prev=q_zero, scale=scale)

    # Phase 0: Smoothly transition from URDF zero pose q=[0,0,0,0,0,0] to Upright Home pose p_home
    init_duration = 2.0
    n_init = max(2, int(np.round(init_duration / dt)))
    for tl in np.linspace(0.0, init_duration, n_init, endpoint=True):
        s, s_dot, s_ddot, _ = evaluate_profile(tl, init_duration, profile_type=profile_type)
        q_sol = (1.0 - s) * q_zero + s * q_home
        q_dot = s_dot * (q_home - q_zero)
        T_fk = forward_kinematics(q_sol, scale=scale)
        times.append(t_global + tl)
        phase_labels.append("0_Init_Upright_Pose")
        s_vals.append(s)
        s_dot_vals.append(s_dot)
        s_ddot_vals.append(s_ddot)
        tcp_positions.append(T_fk[:3, 3])
        tcp_velocities.append(np.zeros(3))
        joint_positions.append(q_sol)
        joint_velocities.append(q_dot)
        gripper_positions.append([GRIP_OPEN[0], GRIP_OPEN[1]])
        attached_flags.append(False)
        box_b_rpy_deg.append(np.zeros(3))

    t_global += init_duration
    q_prev = q_home.copy()

    for name, duration, p0, p1, mode, g0, g1, is_attached in phases:
        n_steps = max(2, int(np.round(duration / dt)))
        t_local_arr = np.linspace(0.0, duration, n_steps, endpoint=True)

        # Cylindrical interpolation parameters (prevents passing through base column at r=0)
        r0, r1 = np.hypot(p0[0], p0[1]), np.hypot(p1[0], p1[1])
        ang0 = np.arctan2(p0[1], p0[0])
        ang1 = np.arctan2(p1[1], p1[0])
        d_ang = (ang1 - ang0 + np.pi) % (2.0 * np.pi) - np.pi

        for idx, tl in enumerate(t_local_arr):
            if idx == 0 and len(times) > 0:
                continue  # Avoid duplicate boundary timestamp

            s, s_dot, s_ddot, _ = evaluate_profile(tl, duration, profile_type=profile_type)

            if mode == "cartesian" or mode == "hold":
                pos = (1.0 - s) * p0 + s * p1
                v_lin = s_dot * (p1 - p0)
            else:
                # Cylindrical task-space path (smoothly arcs around origin while R_0^6 = const)
                r_s = (1.0 - s) * r0 + s * r1
                dr_dt = s_dot * (r1 - r0)
                ang_s = ang0 + s * d_ang
                dang_dt = s_dot * d_ang
                z_s = (1.0 - s) * p0[2] + s * p1[2]
                dz_dt = s_dot * (p1[2] - p0[2])

                pos = np.array([r_s * np.cos(ang_s), r_s * np.sin(ang_s), z_s], dtype=float)
                vx = dr_dt * np.cos(ang_s) - r_s * np.sin(ang_s) * dang_dt
                vy = dr_dt * np.sin(ang_s) + r_s * np.cos(ang_s) * dang_dt
                v_lin = np.array([vx, vy, dz_dt], dtype=float)

            # Solve analytical IK with upright orientation constraint R_UPRIGHT_DEFAULT
            q_sol = inverse_kinematics(pos, R_UPRIGHT_DEFAULT, q_prev=q_prev, scale=scale)
            q_prev = q_sol

            # Compute joint velocities via Geometric Jacobian with omega = [0, 0, 0]^T
            q_dot = compute_joint_velocities(q_sol, v_lin, np.zeros(3), scale=scale)

            # Interpolate gripper finger positions
            q7 = (1.0 - s) * g0[0] + s * g1[0]
            q8 = (1.0 - s) * g0[1] + s * g1[1]

            times.append(t_global + tl)
            phase_labels.append(name)
            s_vals.append(s)
            s_dot_vals.append(s_dot)
            s_ddot_vals.append(s_ddot)
            tcp_positions.append(pos)
            tcp_velocities.append(v_lin)
            joint_positions.append(q_sol)
            joint_velocities.append(q_dot)
            gripper_positions.append([q7, q8])
            attached_flags.append(bool(is_attached))
            box_b_rpy_deg.append(box_b_orientation_rpy(q_sol, scale=scale))

        t_global += duration

    return {
        "time": np.asarray(times, dtype=float),
        "phase": phase_labels,
        "s": np.asarray(s_vals, dtype=float),
        "s_dot": np.asarray(s_dot_vals, dtype=float),
        "s_ddot": np.asarray(s_ddot_vals, dtype=float),
        "tcp_pos": np.asarray(tcp_positions, dtype=float),
        "tcp_vel": np.asarray(tcp_velocities, dtype=float),
        "q": np.asarray(joint_positions, dtype=float),
        "q_dot": np.asarray(joint_velocities, dtype=float),
        "gripper": np.asarray(gripper_positions, dtype=float),
        "attached": np.asarray(attached_flags, dtype=bool),
        "box_b_rpy_deg": np.asarray(box_b_rpy_deg, dtype=float),
        "box_b_pos": np.asarray([xb, yb, z_pick], dtype=float),
        "box_a_pos": np.asarray([xa, ya, 0.0], dtype=float),
        "box_a_place_pos": np.asarray([xa, ya, z_place], dtype=float),
    }


if __name__ == "__main__":
    box_b = sample_random_reachable_box_b(seed=7)
    print(f"Sampled Random Reachable Box B Ground Position: {box_b}")
    traj = generate_pick_and_place_trajectory(box_b, profile_type="s_curve")
    max_rpy_err = np.max(np.abs(traj["box_b_rpy_deg"]))
    max_q_vel = np.max(np.abs(traj["q_dot"]))
    print(f"Total trajectory points: {len(traj['time'])}, Duration: {traj['time'][-1]:.2f} s")
    print(f"Max Box B Orientation Error (Roll/Pitch/Yaw): {max_rpy_err:.4e} deg")
    print(f"Max Joint Velocity across trajectory: {max_q_vel:.3f} rad/s")
