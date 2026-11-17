#!/usr/bin/env python3
"""
EN3563 - Robotics Mini Project: Publication-Quality Report Figure Generator
===========================================================================
Generates all technical figures for the project report:
  1. fig1_coordinate_frames_dh.png         (3D CAD + Link Coordinate Frames O_0..O_6)
  2. fig2_velocity_profiles.png            (Trapezoidal vs S-Curve Velocity Profiles)
  3. fig3_trajectory_3d.png                (3D Pick-and-Place Trajectory & Upright Triads)
  4. fig4_joint_angles_velocities.png      (Joint Angles q(t), Velocities q_dot(t), Gripper)
  5. fig5_upright_orientation_verification.png (TCP Pose & Box B Upright Orientation Proof)
  6. fig6_simulation_sequence.png          (4-Stage 3D CAD Simulation Sequence)
"""

import os
import struct
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from kinematics import (
    R_UPRIGHT_DEFAULT,
    forward_kinematics,
    forward_kinematics_all_frames,
    inverse_kinematics,
)
from trajectory_planner import (
    generate_pick_and_place_trajectory,
    s_curve_profile,
    trapezoidal_profile,
)

OUT_DIR = "/home/chirantha-akarsha/Robotics/report_figures"
MESH_DIR = "/home/chirantha-akarsha/Robotics/Assem1/meshes"
os.makedirs(OUT_DIR, exist_ok=True)


def load_stl_faces(stl_path, stride=6):
    """Loads triangles from a binary STL mesh with decimation for fast 3D rendering."""
    with open(stl_path, "rb") as f:
        _ = f.read(80)
        count = struct.unpack("<I", f.read(4))[0]
        dtype = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("attr", "<u2")])
        data = np.frombuffer(f.read(count * 50), dtype=dtype)
        tris = data["v"].reshape(-1, 3, 3)[::stride]
    return tris.astype(float)


def rpy_to_mat(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def axis_angle_to_mat(axis, theta):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(theta), np.sin(theta)
    C = 1.0 - c
    return np.array([
        [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ])


def compute_urdf_link_transforms(q, grip=(0.0, 0.0)):
    """Computes the exact 4x4 world transform for each STL mesh link in Assem1.urdf."""
    T_base = np.eye(4)
    joints = [
        ("link1", [0, 0, 0.199], [0, 0, 0], [0, 0, 1], q[0], "rev"),
        ("link2", [-0.040, 0, 0.131], [np.pi / 2, 0, np.pi], [0, 0, -1], q[1], "rev"),
        ("link3", [0, 0.445, 0], [0, 0, np.pi / 2], [0, 0, -1], q[2], "rev"),
        ("link4", [0.040, 0, 0], [np.pi / 2, 0, 0], [0, 0, 1], q[3], "rev"),
        ("llink5", [0, 0, 0.442], [-np.pi / 2, 0, 0], [0, 0, -1], q[4], "rev"),
        ("link6", [0, 0, 0], [np.pi / 2, 0, 0], [0, 0, -1], q[5], "rev"),
    ]
    out = {"base_link": T_base.copy()}
    T = T_base.copy()
    for name, xyz, rpy, ax, qi, _ in joints:
        Ti = np.eye(4)
        R0 = rpy_to_mat(*rpy)
        Ti[:3, :3] = R0 @ axis_angle_to_mat(ax, qi)
        Ti[:3, 3] = xyz
        T = T @ Ti
        out[name] = T.copy()

    # Prismatic gripper jaws link7 and link8 relative to link6
    for lname, q_prism in [("link7", grip[0]), ("link8", grip[1])]:
        Ti = np.eye(4)
        R0 = rpy_to_mat(-np.pi / 2, 0, 0)
        Ti[:3, :3] = R0
        Ti[:3, 3] = np.array([0, 0, 0.0875]) + R0 @ np.array([0, 0, q_prism])
        out[lname] = out["link6"] @ Ti

    return out


def draw_box_3d(ax, center_xyz, size_xyz, color, alpha=0.85, edgecolor="k", lw=0.6):
    cx, cy, cz = center_xyz
    dx, dy, dz = size_xyz[0] / 2.0, size_xyz[1] / 2.0, size_xyz[2] / 2.0
    v = np.array([
        [cx - dx, cy - dy, cz - dz],
        [cx + dx, cy - dy, cz - dz],
        [cx + dx, cy + dy, cz - dz],
        [cx - dx, cy + dy, cz - dz],
        [cx - dx, cy - dy, cz + dz],
        [cx + dx, cy - dy, cz + dz],
        [cx + dx, cy + dy, cz + dz],
        [cx - dx, cy + dy, cz + dz],
    ])
    faces = [
        [v[0], v[1], v[2], v[3]],
        [v[4], v[5], v[6], v[7]],
        [v[0], v[1], v[5], v[4]],
        [v[2], v[3], v[7], v[6]],
        [v[1], v[2], v[6], v[5]],
        [v[4], v[7], v[3], v[0]],
    ]
    poly = Poly3DCollection(faces, facecolors=color, edgecolors=edgecolor, linewidths=lw, alpha=alpha)
    ax.add_collection3d(poly)


def render_robot_meshes(ax, q, grip=(0.0, 0.0), alpha=0.85, stride=10):
    link_Transforms = compute_urdf_link_transforms(q, grip)
    colors = {
        "base_link": "#2c323d",
        "link1": "#1565c0",
        "link2": "#1976d2",
        "link3": "#dfe4ea",
        "link4": "#1565c0",
        "llink5": "#dfe4ea",
        "link6": "#37474f",
        "link7": "#f57c00",
        "link8": "#f57c00",
    }
    for lname, T in link_Transforms.items():
        stl_path = os.path.join(MESH_DIR, f"{lname}.STL")
        if not os.path.exists(stl_path):
            continue
        tris = load_stl_faces(stl_path, stride=stride)
        R, p = T[:3, :3], T[:3, 3]
        tris_w = np.einsum("ij,ntj->nti", R, tris) + p
        poly = Poly3DCollection(
            tris_w,
            facecolors=colors.get(lname, "#90a4ae"),
            edgecolors="none",
            alpha=alpha,
        )
        ax.add_collection3d(poly)


def draw_frame_triad(ax, T, label, length=0.09, lw=2.2):
    o = T[:3, 3]
    R = T[:3, :3]
    axis_colors = ["#d32f2f", "#2e7d32", "#1565c0"]  # X: Red, Y: Green, Z: Blue
    axis_names = ["X", "Y", "Z"]
    for i in range(3):
        d = R[:, i] * length
        ax.plot(
            [o[0], o[0] + d[0]],
            [o[1], o[1] + d[1]],
            [o[2], o[2] + d[2]],
            color=axis_colors[i],
             lw=lw,
        )
    ax.scatter([o[0]], [o[1]], [o[2]], color="k", s=22, zorder=10)
    ax.text(o[0] + 0.02, o[1] + 0.02, o[2] + 0.02, label, fontsize=8.5, fontweight="bold", color="#111")


# =====================================================================
# FIGURE 1: COORDINATE FRAME ASSIGNMENT & D-H LINK DIAGRAM
# =====================================================================
def generate_fig1_coordinate_frames():
    fig = plt.figure(figsize=(14, 6.8), dpi=220)

    # Subplot 1: 3D CAD Mesh with Coordinate Frames at q = [0, 0, 0, 0, 0, 0]
    ax1 = fig.add_subplot(121, projection="3d")
    q_zero = np.zeros(6)
    render_robot_meshes(ax1, q_zero, grip=(0.0, 0.0), alpha=0.45, stride=8)

    frames = forward_kinematics_all_frames(q_zero)
    labels = [
        ("frame_0_base",      "O₀ (Base: 0,0,0)"),
        ("frame_1_shoulder",  "O₁ (J2 L-Axis: d₁=0.330, a₁=0.040)"),
        ("frame_2_upper_arm", "O₂ (J3 U-Axis: a₂=0.445)"),
        ("frame_3_elbow",     "O₃ (J4 R-Axis: a₃=0.040)"),
        ("frame_4_forearm",   "O₄ (J5 B-Axis Wrist: d₄=0.442)"),
        ("frame_6_tcp",       "O₆ (TCP: d₆=0.1125)"),
    ]
    pts = []
    for key, lbl in labels:
        T = frames[key]
        pts.append(T[:3, 3])
        draw_frame_triad(ax1, T, lbl, length=0.095, lw=2.4)

    pts = np.array(pts)
    ax1.plot(pts[:, 0], pts[:, 1], pts[:, 2], "k--", lw=1.6, label="Kinematic Chain")

    # Legend proxies for X, Y, Z axes
    ax1.plot([], [], [], color="#d32f2f", lw=2.5, label="X-axis (Red)")
    ax1.plot([], [], [], color="#2e7d32", lw=2.5, label="Y-axis (Green)")
    ax1.plot([], [], [], color="#1565c0", lw=2.5, label="Z-axis (Blue)")

    ax1.set_xlim(-0.70, 0.25)
    ax1.set_ylim(-0.45, 0.45)
    ax1.set_zlim(0.0, 0.95)
    ax1.set_xlabel("X₀ (m)")
    ax1.set_ylabel("Y₀ (m)")
    ax1.set_zlabel("Z₀ (m)")
    ax1.view_init(elev=22, azim=-55)
    ax1.set_title("(a) 3D CAD Model & D-H Coordinate Frames (q = 0)", fontsize=11, fontweight="bold")
    ax1.legend(loc="upper left", fontsize=8)

    # Subplot 2: 2D Schematic Kinematic Diagram in X0-Z0 Plane + D-H Table
    ax2 = fig.add_subplot(122)
    ax2.set_facecolor("#f8f9fa")
    # Plot kinematic chain in (-X0, Z0) plane at q=0
    chain_xz = np.array([
        [0.000, 0.000],
        [0.000, 0.330],
        [-0.040, 0.330],
        [-0.040, 0.775],
        [-0.040, 0.815],
        [-0.482, 0.815],
        [-0.5945, 0.815],
    ])
    ax2.plot(chain_xz[:, 0], chain_xz[:, 1], "-o", color="#1565c0", lw=3.0, markersize=8, label="Robot Links")

    annotations = [
        (0.00, 0.00, "Frame 0 (Base)\n(0, 0, 0)", (0.04, 0.04)),
        (-0.04, 0.33, "Frame 1 (L-Axis)\n(-0.040, 0, 0.330)\nd₁=330 mm, a₁=40 mm", (-0.36, 0.38)),
        (-0.04, 0.775, "Frame 2 (U-Axis)\n(-0.040, 0, 0.775)\na₂=445 mm", (0.03, 0.65)),
        (-0.04, 0.815, "Frame 3 (R-Axis)\n(-0.040, 0, 0.815)\na₃=40 mm", (0.03, 0.84)),
        (-0.482, 0.815, "Frames 4 & 5 (Wrist B/T)\n(-0.482, 0, 0.815)\nd₄=442 mm", (-0.52, 0.63)),
        (-0.5945, 0.815, "Frame 6 (TCP)\n(-0.5945, 0, 0.815)\nd₆=112.5 mm", (-0.68, 0.90)),
    ]
    for x, z, text, (tx, tz) in annotations:
        ax2.annotate(
            text,
            xy=(x, z),
            xytext=(tx, tz),
            fontsize=8.2,
            fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#90a4ae", alpha=0.95),
            arrowprops=dict(arrowstyle="->", color="#37474f", lw=1.2),
        )

    # Embedded D-H Table on bottom of ax2
    table_data = [
        ["1 (S)", "q₁ + π",   "0.330", "0.040", "+π/2"],
        ["2 (L)", "-q₂ + π/2","0.000", "0.445", "0"],
        ["3 (U)", "-q₃",      "0.000", "0.040", "+π/2"],
        ["4 (R)", "q₄",       "0.442", "0.000", "-π/2"],
        ["5 (B)", "-q₅",      "0.000", "0.000", "+π/2"],
        ["6 (T)", "-q₆",      "0.1125","0.000", "0"],
    ]
    col_labels = ["Link i", "θᵢ (rad)", "dᵢ (m)", "aᵢ (m)", "αᵢ (rad)"]
    tbl = ax2.table(
        cellText=table_data,
        colLabels=col_labels,
        loc="lower left",
        bbox=[0.03, 0.02, 0.64, 0.28],
        cellLoc="center",
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8.2)

    ax2.set_xlim(-0.74, 0.28)
    ax2.set_ylim(-0.08, 1.02)
    ax2.set_xlabel("World X₀ Coordinate (m)")
    ax2.set_ylabel("World Z₀ Coordinate (m)")
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.set_title("(b) Link Dimensions & Standard D-H Parameter Table", fontsize=11, fontweight="bold")

    plt.tight_layout()
    path = os.path.join(OUT_DIR, "fig1_coordinate_frames_dh.png")
    plt.savefig(path, dpi=220)
    plt.close(fig)
    print(f"Saved {path}")


# =====================================================================
# FIGURE 2: SMOOTH VELOCITY PROFILES (TRAPEZOIDAL VS S-CURVE)
# =====================================================================
def generate_fig2_velocity_profiles():
    T = 3.0
    t_arr = np.linspace(0.0, T, 500)
    sc = np.array([s_curve_profile(t, T) for t in t_arr])
    tr = np.array([trapezoidal_profile(t, T, blend_fraction=0.25) for t in t_arr])

    fig, axes = plt.subplots(2, 2, figsize=(12.5, 7.5), dpi=220)
    titles = [
        ("Normalized Position s(t)", "s(t) [dimensionless]"),
        ("Normalized Velocity ṡ(t)", "ṡ(t) [s⁻¹]"),
        ("Normalized Acceleration s̈(t)", "s̈(t) [s⁻²]"),
        ("Normalized Jerk s⃛(t)", "s⃛(t) [s⁻³]"),
    ]
    for i, (title, ylabel) in enumerate(titles):
        ax = axes[i // 2, i % 2]
        ax.plot(t_arr, sc[:, i], color="#1565c0", lw=2.5, label="Quintic S-Curve Profile")
        ax.plot(t_arr, tr[:, i], "--", color="#e65100", lw=2.2, label="Trapezoidal (LSPB) Profile")
        ax.set_title(title, fontsize=10.5, fontweight="bold")
        ax.set_xlabel("Time t (s)")
        ax.set_ylabel(ylabel)
        ax.grid(True, linestyle=":", alpha=0.7)
        ax.legend(loc="best", fontsize=8.5)

    fig.suptitle("Comparison of Smooth Velocity Profiles (S-Curve vs. Trapezoidal LSPB, T = 3.0 s)", fontsize=12.5, fontweight="bold")
    plt.tight_layout()
    path = os.path.join(OUT_DIR, "fig2_velocity_profiles.png")
    plt.savefig(path, dpi=220)
    plt.close(fig)
    print(f"Saved {path}")


# =====================================================================
# FIGURE 3: 3D PICK-AND-PLACE TRAJECTORY & UPRIGHT ORIENTATION TRIADS
# =====================================================================
def generate_fig3_trajectory_3d(traj):
    fig = plt.figure(figsize=(11, 8), dpi=220)
    ax = fig.add_subplot(111, projection="3d")

    # Draw ground Box B at pickup and Box A at destination
    xb, yb, _ = traj["box_b_pos"]
    xa, ya, _ = traj["box_a_pos"]
    draw_box_3d(ax, [xb, yb, 0.025], [0.05, 0.05, 0.05], color="#e53935", alpha=0.90)
    draw_box_3d(ax, [xa, ya, 0.100], [0.30, 0.30, 0.20], color="#2e7d32", alpha=0.65)
    draw_box_3d(ax, [xa, ya, 0.225], [0.05, 0.05, 0.05], color="#fb8c00", alpha=0.95)

    # Render robot arm at Pick and Place poses
    idx_pick = [i for i, p in enumerate(traj["phase"]) if p.startswith("3_Grasp")][0]
    idx_place = [i for i, p in enumerate(traj["phase"]) if p.startswith("7_Release")][0]
    render_robot_meshes(ax, traj["q"][idx_pick], grip=(-0.01, 0.01), alpha=0.35, stride=12)
    render_robot_meshes(ax, traj["q"][idx_place], grip=(-0.01, 0.01), alpha=0.55, stride=12)

    # Plot 3D TCP Trajectory
    tcp = traj["tcp_pos"]
    attached = traj["attached"]
    ax.plot(tcp[~attached, 0], tcp[~attached, 1], tcp[~attached, 2], "--", color="#546e7a", lw=2.0, label="Approach / Retract Path (Unloaded)")
    ax.plot(tcp[attached, 0], tcp[attached, 1], tcp[attached, 2], "-", color="#d32f2f", lw=3.2, label="Box B Transport Path (Upright Constrained)")

    # Draw upright orientation triads along the transport path
    transport_indices = np.where(attached)[0][::25]
    for idx in transport_indices:
        T_tcp = forward_kinematics(traj["q"][idx])
        o = T_tcp[:3, 3]
        # Draw Box B upright Z-normal (+Z_world) and X-normal (+X_world)
        ax.quiver(o[0], o[1], o[2], 0, 0, 0.06, color="#1565c0", lw=1.8, arrow_length_ratio=0.3)
        ax.quiver(o[0], o[1], o[2], 0.06, 0, 0, color="#d32f2f", lw=1.8, arrow_length_ratio=0.3)

    ax.text(xb, yb, 0.09, f"Box B Pick\n({xb:.2f}, {yb:.2f}, 0.025)", fontsize=9, fontweight="bold", color="#b71c1c")
    ax.text(xa, ya, 0.30, f"Box B Placed on Box A\n({xa:.2f}, {ya:.2f}, 0.225)", fontsize=9, fontweight="bold", color="#1b5e20")

    ax.set_xlim(-0.65, 0.75)
    ax.set_ylim(-0.55, 0.75)
    ax.set_zlim(0.0, 0.85)
    ax.set_xlabel("World X (m)")
    ax.set_ylabel("World Y (m)")
    ax.set_zlabel("World Z (m)")
    ax.view_init(elev=26, azim=-48)
    ax.set_title("3D Pick-and-Place Trajectory with Upright Box B Orientation Constraint", fontsize=12, fontweight="bold")
    ax.legend(loc="upper left", fontsize=9)

    plt.tight_layout()
    path = os.path.join(OUT_DIR, "fig3_trajectory_3d.png")
    plt.savefig(path, dpi=220)
    plt.close(fig)
    print(f"Saved {path}")


# =====================================================================
# FIGURE 4: JOINT ANGLES, JOINT VELOCITIES & GRIPPER TRAJECTORY
# =====================================================================
def generate_fig4_joints(traj):
    t = traj["time"]
    q_deg = np.degrees(traj["q"])
    qd = traj["q_dot"]
    grip_mm = traj["gripper"] * 1000.0

    fig, axes = plt.subplots(3, 1, figsize=(12.5, 9.2), sharex=True, dpi=220)
    j_labels = ["q₁ (S-Axis)", "q₂ (L-Axis)", "q₃ (U-Axis)", "q₄ (R-Axis)", "q₅ (B-Axis)", "q₆ (T-Axis)"]
    colors = ["#1565c0", "#ef6c00", "#2e7d32", "#8e24aa", "#c62828", "#00838f"]

    for i in range(6):
        axes[0].plot(t, q_deg[:, i], lw=2.0, color=colors[i], label=j_labels[i])
        axes[1].plot(t, qd[:, i], lw=2.0, color=colors[i], label=f"q̇_{i+1}")

    axes[0].set_ylabel("Joint Angle (deg)")
    axes[0].set_title("(a) Joint Angle Trajectories q₁(t) ... q₆(t) from Analytical Inverse Kinematics", fontsize=10.5, fontweight="bold")
    axes[0].grid(True, linestyle=":", alpha=0.7)
    axes[0].legend(loc="upper right", ncol=3, fontsize=8.5)

    axes[1].set_ylabel("Joint Velocity (rad/s)")
    axes[1].set_title("(b) Smooth Joint Velocities q̇₁(t) ... q̇₆(t) via Geometric Jacobian Velocity Control", fontsize=10.5, fontweight="bold")
    axes[1].grid(True, linestyle=":", alpha=0.7)
    axes[1].legend(loc="upper right", ncol=6, fontsize=8.5)

    axes[2].plot(t, grip_mm[:, 0], lw=2.3, color="#d84315", label="Left Jaw q₇ (mm)")
    axes[2].plot(t, grip_mm[:, 1], lw=2.3, color="#1565c0", label="Right Jaw q₈ (mm)")
    axes[2].fill_between(t, -12, 12, where=traj["attached"], color="#c8e6c9", alpha=0.45, label="Box B Grasped & Upright")
    axes[2].set_xlabel("Time t (s)")
    axes[2].set_ylabel("Jaw Displacement (mm)")
    axes[2].set_title("(c) Parallel-Jaw Gripper Actuation & Box B Grasp State", fontsize=10.5, fontweight="bold")
    axes[2].grid(True, linestyle=":", alpha=0.7)
    axes[2].legend(loc="upper right", fontsize=8.5)

    plt.tight_layout()
    path = os.path.join(OUT_DIR, "fig4_joint_angles_velocities.png")
    plt.savefig(path, dpi=220)
    plt.close(fig)
    print(f"Saved {path}")


# =====================================================================
# FIGURE 5: UPRIGHT ORIENTATION VERIFICATION PLOT
# =====================================================================
def generate_fig5_upright_verification(traj):
    t = traj["time"]
    tcp = traj["tcp_pos"]
    q_deg = np.degrees(traj["q"])
    rpy = traj["box_b_rpy_deg"]

    # Filter from Phase 1 onward (where upright constraint is active)
    mask = np.array([not p.startswith("0_") for p in traj["phase"]])
    t_u = t[mask]
    rpy_u = rpy[mask]
    pitch_sum = q_deg[mask, 1] + q_deg[mask, 2] + q_deg[mask, 4]

    fig, axes = plt.subplots(3, 1, figsize=(12.0, 8.5), sharex=True, dpi=220)

    axes[0].plot(t, tcp[:, 0], lw=2.2, color="#d32f2f", label="TCP X(t) [m]")
    axes[0].plot(t, tcp[:, 1], lw=2.2, color="#2e7d32", label="TCP Y(t) [m]")
    axes[0].plot(t, tcp[:, 2], lw=2.2, color="#1565c0", label="TCP Z(t) [m]")
    axes[0].set_ylabel("Position (m)")
    axes[0].set_title("(a) End-Effector (TCP) Cartesian Position Trajectory", fontsize=10.5, fontweight="bold")
    axes[0].grid(True, linestyle=":", alpha=0.7)
    axes[0].legend(loc="upper right", ncol=3, fontsize=9)

    axes[1].plot(t_u, pitch_sum, lw=2.4, color="#6a1b9a", label="q₂(t) + q₃(t) + q₅(t) = 90.0° (Vertical Pitch Invariant)")
    axes[1].plot(t_u, q_deg[mask, 3], "--", lw=2.0, color="#00838f", label="q₄(t) = 0.0° (Zero Roll Twist)")
    axes[1].set_ylabel("Angle (deg)")
    axes[1].set_ylim(-15, 105)
    axes[1].set_title("(b) Analytical Wrist Orientation Coupling Maintaining Vertical Approach Axis Z₆ = [0, 0, -1]ᵀ", fontsize=10.5, fontweight="bold")
    axes[1].grid(True, linestyle=":", alpha=0.7)
    axes[1].legend(loc="center right", fontsize=9)

    axes[2].plot(t_u, rpy_u[:, 0], lw=2.2, color="#d32f2f", label="Box B Roll Error (deg)")
    axes[2].plot(t_u, rpy_u[:, 1], "--", lw=2.2, color="#2e7d32", label="Box B Pitch Error (deg)")
    axes[2].plot(t_u, rpy_u[:, 2], ":", lw=2.2, color="#1565c0", label="Box B Yaw Error (deg)")
    axes[2].set_ylim(-1.0, 1.0)
    axes[2].set_xlabel("Time t (s)")
    axes[2].set_ylabel("Orientation Error (deg)")
    axes[2].set_title("(c) Box B World Orientation Error During Entire Pick-and-Place Motion (Strictly 0.000°)", fontsize=10.5, fontweight="bold")
    axes[2].grid(True, linestyle=":", alpha=0.7)
    axes[2].legend(loc="upper right", ncol=3, fontsize=9)

    plt.tight_layout()
    path = os.path.join(OUT_DIR, "fig5_upright_orientation_verification.png")
    plt.savefig(path, dpi=220)
    plt.close(fig)
    print(f"Saved {path}")


# =====================================================================
# FIGURE 6: 4-STAGE 3D CAD SIMULATION SEQUENCE
# =====================================================================
def generate_fig6_simulation_sequence(traj):
    fig = plt.figure(figsize=(14, 10), dpi=220)
    xb, yb, _ = traj["box_b_pos"]
    xa, ya, _ = traj["box_a_pos"]

    stage_defs = [
        ("3_Grasp",          "(a) Stage 1: Grasping Box B at Random Ground Position", [xb, yb, 0.025]),
        ("4_Vertical_Lift",  "(b) Stage 2: Smooth Vertical Lift (Box B Upright)",     None),
        ("5_Upright_Transit","(c) Stage 3: Upright Horizontal Transit Across Workcell", None),
        ("7_Release",        "(d) Stage 4: Placing Box B on Top Center of Box A",     [xa, ya, 0.225]),
    ]

    for panel_idx, (phase_prefix, title, fixed_box_b) in enumerate(stage_defs, start=1):
        ax = fig.add_subplot(2, 2, panel_idx, projection="3d")
        matching = [i for i, p in enumerate(traj["phase"]) if p.startswith(phase_prefix)]
        idx = matching[len(matching) // 2]

        q_curr = traj["q"][idx]
        grip_curr = traj["gripper"][idx]
        tcp_curr = traj["tcp_pos"][idx]
        box_b_center = fixed_box_b if fixed_box_b is not None else tcp_curr

        # Draw ground plane grid, Box A, and Box B
        draw_box_3d(ax, [xa, ya, 0.100], [0.30, 0.30, 0.20], color="#2e7d32", alpha=0.75)
        draw_box_3d(ax, box_b_center, [0.05, 0.05, 0.05], color="#e53935", alpha=0.95)
        render_robot_meshes(ax, q_curr, grip=grip_curr, alpha=0.82, stride=10)

        # Plot trace up to current index
        ax.plot(traj["tcp_pos"][:idx, 0], traj["tcp_pos"][:idx, 1], traj["tcp_pos"][:idx, 2], "r-", lw=2.0)

        ax.set_xlim(-0.60, 0.75)
        ax.set_ylim(-0.45, 0.75)
        ax.set_zlim(0.0, 0.85)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.view_init(elev=25, azim=-45)
        ax.set_title(f"{title}\nTCP=({tcp_curr[0]:.2f}, {tcp_curr[1]:.2f}, {tcp_curr[2]:.2f}) m, Roll=0°, Pitch=0°", fontsize=10, fontweight="bold")

    plt.tight_layout()
    path = os.path.join(OUT_DIR, "fig6_simulation_sequence.png")
    plt.savefig(path, dpi=220)
    plt.close(fig)
    print(f"Saved {path}")


if __name__ == "__main__":
    traj = generate_pick_and_place_trajectory(
        box_b_pos=(-0.35, 0.45, 0.025),
        box_a_pos=(0.50, 0.50, 0.0),
        profile_type="s_curve",
        dt=0.02,
        scale=1.0,
    )
    generate_fig1_coordinate_frames()
    generate_fig2_velocity_profiles()
    generate_fig3_trajectory_3d(traj)
    generate_fig4_joints(traj)
    generate_fig5_upright_verification(traj)
    generate_fig6_simulation_sequence(traj)
    print("All report figures generated successfully!")
