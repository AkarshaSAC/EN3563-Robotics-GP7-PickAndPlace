#!/usr/bin/env python3
"""
EN3563 - Robotics Mini Project: YASKAWA GP7 Kinematics Module
=============================================================
Implements:
1. Standard Denavit-Hartenberg (D-H) Parameter Table & Transformation Matrices
2. Symbolic and Numerical Forward Kinematics (FK) for all link frames
3. Closed-Form Analytical Inverse Kinematics (IK) via Kinematic Decoupling
   (Position IK for Joints 1-3 + Spherical Wrist Orientation IK for Joints 4-6)
   with Upright Box-B Constraint (Roll = 0, Pitch = 0, Yaw = const)
4. Geometric Manipulator Jacobian J(q) for Differential Kinematics & Velocity Control
"""

import numpy as np


# =====================================================================
# 1. DENAVIT-HARTENBERG (D-H) PARAMETERS FOR YASKAWA GP7 (Assem1.urdf)
# =====================================================================
# Nominal dimensions measured from SolidWorks CAD / Assem1.urdf:
#   d1 = 0.199 + 0.131 = 0.330 m  (Base to Shoulder L-axis height)
#   a1 = 0.040 m                  (Shoulder horizontal offset along -X0 at q1=0)
#   a2 = 0.445 m                  (Upper arm length: L-axis to U-axis)
#   a3 = 0.040 m                  (Elbow offset: U-axis to R-axis)
#   d4 = 0.442 m                  (Forearm length: U-axis to B-axis wrist center)
#   d6 = 0.0875 + 0.025 = 0.1125 m (Wrist center to Parallel Gripper TCP center)

GP7_DIMENSIONS = {
    "d1": 0.330,
    "a1": 0.040,
    "a2": 0.445,
    "a3": 0.040,
    "d4": 0.442,
    "d6": 0.1125,
}

# Upright Top-Down Grasp Rotation Matrix R_0^6:
#   X_6 = [1,  0,  0]^T  (Horizontal gripper normal)
#   Y_6 = [0, -1,  0]^T  (Gripper jaw opening/closing axis)
#   Z_6 = [0,  0, -1]^T  (Tool approach axis pointing vertically downward)
R_UPRIGHT_DEFAULT = np.array([
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0],
], dtype=float)


def get_dh_table(q, scale=1.0, d6=None):
    """
    Returns the 6x4 Standard Denavit-Hartenberg parameter table [theta_i, d_i, a_i, alpha_i]
    for given URDF joint angles q = [q1, q2, q3, q4, q5, q6] (in radians).

    Link | Joint   | theta_i        | d_i (m)       | a_i (m)       | alpha_i (rad)
    -----+---------+----------------+---------------+---------------+--------------
      1  | S-Axis  |  q1 + pi       | 0.330 * scale | 0.040 * scale | +pi/2
      2  | L-Axis  | -q2 + pi/2     | 0.000         | 0.445 * scale |  0
      3  | U-Axis  | -q3            | 0.000         | 0.040 * scale | +pi/2
      4  | R-Axis  |  q4            | 0.442 * scale | 0.000         | -pi/2
      5  | B-Axis  | -q5            | 0.000         | 0.000         | +pi/2
      6  | T-Axis  | -q6            | 0.1125        | 0.000         |  0
    """
    q1, q2, q3, q4, q5, q6 = q
    d1 = GP7_DIMENSIONS["d1"] * scale
    a1 = GP7_DIMENSIONS["a1"] * scale
    a2 = GP7_DIMENSIONS["a2"] * scale
    a3 = GP7_DIMENSIONS["a3"] * scale
    d4 = GP7_DIMENSIONS["d4"] * scale
    d6_val = GP7_DIMENSIONS["d6"] if d6 is None else d6

    return [
        (q1 + np.pi,      d1,    a1,  np.pi / 2.0),
        (-q2 + np.pi / 2, 0.0,   a2,  0.0),
        (-q3,             0.0,   a3,  np.pi / 2.0),
        (q4,              d4,   0.0, -np.pi / 2.0),
        (-q5,             0.0,  0.0,  np.pi / 2.0),
        (-q6,             d6_val, 0.0, 0.0),
    ]


def dh_transform(theta, d, a, alpha):
    """
    Computes the Standard D-H Homogeneous Transformation Matrix A_i:
        A_i = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ], dtype=float)


def forward_kinematics(q, scale=1.0, d6=None):
    """
    Computes the End-Effector (TCP) 4x4 homogeneous transformation matrix T_0^6(q).
    """
    T = np.eye(4, dtype=float)
    for theta, d, a, alpha in get_dh_table(q, scale=scale, d6=d6):
        T = T @ dh_transform(theta, d, a, alpha)
    return T


def forward_kinematics_all_frames(q, scale=1.0, d6=None):
    """
    Returns a dictionary of 4x4 transformation matrices for all coordinate frames
    from Base (Frame 0) to TCP (Frame 6) for visualization and Jacobian computation.
    """
    frames = {"frame_0_base": np.eye(4, dtype=float)}
    T = np.eye(4, dtype=float)
    names = [
        "frame_1_shoulder",
        "frame_2_upper_arm",
        "frame_3_elbow",
        "frame_4_forearm",
        "frame_5_wrist",
        "frame_6_tcp",
    ]
    dh_params = get_dh_table(q, scale=scale, d6=d6)
    for name, (theta, d, a, alpha) in zip(names, dh_params):
        T = T @ dh_transform(theta, d, a, alpha)
        frames[name] = T.copy()
    return frames


def rotation_matrix_to_rpy(R):
    """Extracts Roll-Pitch-Yaw (XYZ extrinsic / ZYX intrinsic) angles in radians."""
    sy = np.hypot(R[0, 0], R[1, 0])
    if sy > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return np.array([roll, pitch, yaw], dtype=float)


def box_b_orientation_rpy(q, scale=1.0):
    """
    Computes the world Roll, Pitch, Yaw (in degrees) of Box B when held by the gripper.
    At the upright reference R_UPRIGHT_DEFAULT, Box B has Roll=0, Pitch=0, Yaw=0.
    """
    T_tcp = forward_kinematics(q, scale=scale)
    R_box = T_tcp[:3, :3] @ R_UPRIGHT_DEFAULT.T
    return np.degrees(rotation_matrix_to_rpy(R_box))


# =====================================================================
# 2. CLOSED-FORM ANALYTICAL INVERSE KINEMATICS (KINEMATIC DECOUPLING)
# =====================================================================
def inverse_kinematics(target_pos, target_rot=None, q_prev=None, scale=1.0, d6=None):
    """
    Closed-form analytical Inverse Kinematics for the 6-DoF YASKAWA GP7 arm.
    Uses kinematic decoupling at the spherical wrist center p_wc:
        p_wc = p_tcp - d6 * R_target[:, 2]

    For maintaining Box B upright (target_rot = R_UPRIGHT_DEFAULT):
        - q4 = 0
        - q5 = pi/2 - (q2 + q3)   (keeps gripper vertical: Roll = 0, Pitch = 0)
        - q6 = q1 - psi_box       (cancels base rotation so Box B Yaw = const)
    """
    if target_rot is None:
        target_rot = R_UPRIGHT_DEFAULT

    d1 = GP7_DIMENSIONS["d1"] * scale
    a1 = GP7_DIMENSIONS["a1"] * scale
    a2 = GP7_DIMENSIONS["a2"] * scale
    a3 = GP7_DIMENSIONS["a3"] * scale
    d4 = GP7_DIMENSIONS["d4"] * scale
    d6_val = GP7_DIMENSIONS["d6"] if d6 is None else d6

    # Step 1: Wrist Center Position (p_wc)
    target_pos = np.asarray(target_pos, dtype=float)
    p_wc = target_pos - d6_val * target_rot[:, 2]
    xw, yw, zw = p_wc

    # Step 2: Base Joint Angle q1
    theta1 = np.arctan2(yw, xw)
    q1 = (theta1 - np.pi + np.pi) % (2.0 * np.pi) - np.pi
    if q_prev is not None:
        q1 = q_prev[0] + ((q1 - q_prev[0] + np.pi) % (2.0 * np.pi) - np.pi)
        if q1 > 2.0 * np.pi:
            q1 -= 2.0 * np.pi
        elif q1 < -2.0 * np.pi:
            q1 += 2.0 * np.pi

    # Step 3: Planar Coordinates (r, s) from Shoulder Joint (L-axis)
    r = np.hypot(xw, yw) - a1
    s = zw - d1

    # Effective forearm length L34 and structural offset angle psi
    L34 = np.hypot(a3, d4)
    psi = np.arctan2(d4, a3)

    D_sq = r**2 + s**2
    cos_gamma = (D_sq - a2**2 - L34**2) / (2.0 * a2 * L34)
    if abs(cos_gamma) > 1.0 + 1e-5:
        raise ValueError(
            f"Target position {target_pos.round(4)} is outside GP7 workspace "
            f"(r={r:.4f} m, s={s:.4f} m, max reach={a1 + a2 + L34:.4f} m)."
        )
    cos_gamma = np.clip(cos_gamma, -1.0, 1.0)

    # Elbow-up configuration (sin_gamma <= 0)
    sin_gamma = -np.sqrt(max(0.0, 1.0 - cos_gamma**2))
    gamma = np.arctan2(sin_gamma, cos_gamma)

    # Step 4: Elbow Joint Angle q3 and Shoulder Joint Angle q2
    theta3 = gamma + psi
    q3 = -theta3

    beta = np.arctan2(s, r)
    phi2 = np.arctan2(L34 * np.sin(gamma), a2 + L34 * np.cos(gamma))
    theta2 = beta - phi2
    q2 = np.pi / 2.0 - theta2

    # Step 5: Spherical Wrist Orientation IK (q4, q5, q6)
    T01 = dh_transform(q1 + np.pi, d1, a1, np.pi / 2.0)
    T12 = dh_transform(-q2 + np.pi / 2.0, 0.0, a2, 0.0)
    T23 = dh_transform(-q3, 0.0, a3, np.pi / 2.0)
    R03 = (T01 @ T12 @ T23)[:3, :3]
    R36 = R03.T @ target_rot

    # Extract spherical wrist angles using the q4=0 non-flipped wrist branch
    c5 = np.clip(R36[2, 2], -1.0, 1.0)
    s5 = -np.hypot(R36[0, 2], R36[1, 2])
    if abs(s5) > 1e-6:
        theta5 = np.arctan2(s5, c5)
        theta4 = np.arctan2(-R36[1, 2], -R36[0, 2])
        theta6 = np.arctan2(-R36[2, 1], R36[2, 0])
    else:
        theta5 = 0.0 if c5 > 0 else np.pi
        theta4 = 0.0
        theta6 = np.arctan2(-R36[0, 1], R36[0, 0])

    q4 = theta4
    q5 = -theta5
    q6 = -theta6

    # Wrap wrist angles and unwrap q6 relative to q_prev
    q4 = (q4 + np.pi) % (2.0 * np.pi) - np.pi
    q5 = (q5 + np.pi) % (2.0 * np.pi) - np.pi
    q6 = (q6 + np.pi) % (2.0 * np.pi) - np.pi
    if q_prev is not None:
        q6 = q_prev[5] + ((q6 - q_prev[5] + np.pi) % (2.0 * np.pi) - np.pi)
        if q6 > 2.0 * np.pi:
            q6 -= 2.0 * np.pi
        elif q6 < -2.0 * np.pi:
            q6 += 2.0 * np.pi

    return np.array([q1, q2, q3, q4, q5, q6], dtype=float)


# =====================================================================
# 3. GEOMETRIC JACOBIAN & DIFFERENTIAL KINEMATICS (VELOCITY CONTROL)
# =====================================================================
def geometric_jacobian(q, scale=1.0, d6=None):
    """
    Computes the 6x6 Geometric Jacobian J(q) mapping joint velocities q_dot
    to spatial end-effector twist V = [v_x, v_y, v_z, omega_x, omega_y, omega_z]^T:
        J_v_i = s_i * (z_{i-1} x (p_6 - p_{i-1}))
        J_w_i = s_i * z_{i-1}
    where s_i in {+1, -1} accounts for the URDF joint axis sign relative to DH theta_i.
    """
    frames = forward_kinematics_all_frames(q, scale=scale, d6=d6)
    frame_list = [
        frames["frame_0_base"],
        frames["frame_1_shoulder"],
        frames["frame_2_upper_arm"],
        frames["frame_3_elbow"],
        frames["frame_4_forearm"],
        frames["frame_5_wrist"],
        frames["frame_6_tcp"],
    ]
    # d(theta_i)/d(q_i) signs from the DH table: [+1, -1, -1, +1, -1, -1]
    signs = [1.0, -1.0, -1.0, 1.0, -1.0, -1.0]
    p_e = frame_list[6][:3, 3]

    J = np.zeros((6, 6), dtype=float)
    for i in range(6):
        z_prev = frame_list[i][:3, 2] * signs[i]
        p_prev = frame_list[i][:3, 3]
        J[:3, i] = np.cross(z_prev, p_e - p_prev)
        J[3:, i] = z_prev
    return J


def compute_joint_velocities(q, v_linear, omega_angular=None, scale=1.0, damping=1e-3):
    """
    Computes joint velocities q_dot from desired Cartesian linear velocity v_linear
    and angular velocity omega_angular using Damped Least-Squares Jacobian Inverse:
        q_dot = J^T (J J^T + lambda^2 I)^{-1} V_ee
    For upright motion, omega_angular = [0, 0, 0]^T so Box B never rotates.
    """
    if omega_angular is None:
        omega_angular = np.zeros(3, dtype=float)
    twist = np.concatenate([np.asarray(v_linear, dtype=float), np.asarray(omega_angular, dtype=float)])
    J = geometric_jacobian(q, scale=scale)
    JJT = J @ J.T + (damping ** 2) * np.eye(6)
    return J.T @ np.linalg.solve(JJT, twist)


def print_symbolic_kinematics():
    """Prints symbolic D-H matrices using SymPy for verification and report generation."""
    import sympy as sp
    q1, q2, q3, q4, q5, q6 = sp.symbols("q1 q2 q3 q4 q5 q6", real=True)
    d1, a1, a2, a3, d4, d6 = sp.symbols("d1 a1 a2 a3 d4 d6", real=True)

    dh_sym = [
        (q1 + sp.pi,      d1, a1,  sp.pi / 2),
        (-q2 + sp.pi / 2,  0, a2,  0),
        (-q3,              0, a3,  sp.pi / 2),
        (q4,              d4,  0, -sp.pi / 2),
        (-q5,              0,  0,  sp.pi / 2),
        (-q6,             d6,  0,  0),
    ]

    for idx, (th, d, a, al) in enumerate(dh_sym, start=1):
        ct, st = sp.cos(th), sp.sin(th)
        ca, sa = sp.cos(al), sp.sin(al)
        Ai = sp.Matrix([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,        sa,       ca,      d],
            [0,         0,        0,      1],
        ])
        print(f"\n--- A_{idx} (Frame {idx-1} -> Frame {idx}) ---")
        sp.pprint(sp.simplify(Ai))


if __name__ == "__main__":
    print("=================================================================")
    print(" YASKAWA GP7 Kinematics Verification (FK, IK, Upright Constraint)")
    print("=================================================================")
    print_symbolic_kinematics()

    print("\n--- Numerical IK/FK Verification Across Pick-and-Place Waypoints ---")
    waypoints = [
        ("Pick Box B (Ground)",  [0.50, -0.25, 0.025]),
        ("Lift Box B",           [0.50, -0.25, 0.320]),
        ("Hover Above Box A",    [0.50,  0.50, 0.350]),
        ("Place on Top Box A",   [0.50,  0.50, 0.225]),
    ]
    q_prev = None
    for label, pt in waypoints:
        q_sol = inverse_kinematics(pt, R_UPRIGHT_DEFAULT, q_prev=q_prev)
        q_prev = q_sol
        T_fk = forward_kinematics(q_sol)
        pos_err = np.linalg.norm(T_fk[:3, 3] - pt)
        rpy_box = box_b_orientation_rpy(q_sol)
        print(
            f"{label:22s} | Target={pt} | "
            f"q(deg)={np.round(np.degrees(q_sol), 2)} | "
            f"PosErr={pos_err:.2e} m | Box B RPY(deg)={np.round(rpy_box, 4)}"
        )
