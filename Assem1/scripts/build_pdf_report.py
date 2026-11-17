#!/usr/bin/env python3
"""Generates a multi-page A4 PDF report (Project_Report.pdf) with equations, tables, code snippets, and all 6 figures."""

import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.image as mpimg

FIG_DIR = "/home/chirantha-akarsha/Robotics/report_figures"
PDF_PATH = "/home/chirantha-akarsha/Robotics/Project_Report.pdf"


def add_title_and_kinematics_page(pdf):
    fig = plt.figure(figsize=(8.27, 11.69), dpi=200)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.axis("off")

    y = 0.95
    ax.text(0.5, y, "EN3563 — Robotics Mini Project Report", ha="center", va="top", fontsize=16, fontweight="bold", color="#0d47a1")
    y -= 0.03
    ax.text(0.5, y, "Implementation of a 6-DoF YASKAWA GP7 Robot Arm for Upright Pick-and-Place", ha="center", va="top", fontsize=11.5, fontweight="bold")
    y -= 0.025
    ax.text(0.5, y, "Group Members:   [Member 1 - Index No.]    |    [Member 2 - Index No.]    |    [Member 3 - Index No.]", ha="center", va="top", fontsize=9.5, color="#333")

    y -= 0.035
    ax.text(0.06, y, "1. Coordinate Frame Assignment & Denavit–Hartenberg (D-H) Parameter Table", fontsize=11, fontweight="bold", color="#0d47a1")
    y -= 0.022
    intro_txt = (
        "The YASKAWA GP7 6-DoF manipulator is placed at the origin (0, 0, 0). Box A (30 cm × 30 cm × 20 cm) is placed at\n"
        "(0.5 m, 0.5 m, 0) and Box B (5 cm × 5 cm × 5 cm) is placed at a random reachable ground coordinate (x_B, y_B, 0).\n"
        "Dimensions measured from the SolidWorks CAD / URDF model: d₁ = 0.330 m, a₁ = 0.040 m, a₂ = 0.445 m, a₃ = 0.040 m,\n"
        "d₄ = 0.442 m, and d₆ = 0.1125 m (wrist center to parallel-jaw gripper Tool Center Point)."
    )
    ax.text(0.06, y, intro_txt, va="top", fontsize=8.8, linespacing=1.35)

    # D-H Table
    table_data = [
        ["1 (S-Axis)", "q₁ + π",     "0.330", "0.040", "+π/2", "Revolute"],
        ["2 (L-Axis)", "-q₂ + π/2",  "0.000", "0.445", "0",    "Revolute"],
        ["3 (U-Axis)", "-q₃",        "0.000", "0.040", "+π/2", "Revolute"],
        ["4 (R-Axis)", "q₄",         "0.442", "0.000", "-π/2", "Revolute"],
        ["5 (B-Axis)", "-q₅",        "0.000", "0.000", "+π/2", "Revolute"],
        ["6 (T-Axis)", "-q₆",        "0.1125","0.000", "0",    "Revolute (TCP)"],
    ]
    cols = ["Link i", "θᵢ (rad)", "dᵢ (m)", "aᵢ (m)", "αᵢ (rad)", "Joint Type"]
    tbl_ax = fig.add_axes([0.08, 0.65, 0.84, 0.14])
    tbl_ax.axis("off")
    tbl = tbl_ax.table(cellText=table_data, colLabels=cols, loc="center", cellLoc="center", bbox=[0, 0, 1, 1])
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8.5)

    # Embed Fig 1
    img1 = mpimg.imread(os.path.join(FIG_DIR, "fig1_coordinate_frames_dh.png"))
    img_ax = fig.add_axes([0.05, 0.31, 0.90, 0.32])
    img_ax.imshow(img1)
    img_ax.axis("off")

    y = 0.29
    ax.text(0.06, y, "2. Forward & Inverse Kinematics Summary", fontsize=11, fontweight="bold", color="#0d47a1")
    y -= 0.022
    fk_ik_txt = (
        "• Forward Kinematics:  T₀⁶(q) = A₁(q₁) A₂(q₂) A₃(q₃) A₄(q₄) A₅(q₅) A₆(q₆), where Aᵢ = Rot_z(θᵢ) Trans_z(dᵢ) Trans_x(aᵢ) Rot_x(αᵢ).\n"
        "• Kinematic Decoupling (Wrist Center):  p_wc = p_TCP - d₆ R₀⁶ ẑ₆ = [x_w, y_w, z_w]ᵀ.\n"
        "• Position IK (Joints 1–3):  q₁ = atan2(y_w, x_w) - π ;   r = √(x_w² + y_w²) - a₁ ,   s = z_w - d₁ ;\n"
        "  L₃₄ = √(a₃² + d₄²) = 0.44381 m,  ψ = atan2(d₄, a₃) = 84.829° ;  cos(γ) = (r² + s² - a₂² - L₃₄²) / (2 a₂ L₃₄),\n"
        "  γ = atan2(-√(1 - cos²γ), cos γ) ;  q₃ = -(γ + ψ) ;  q₂ = π/2 - [atan2(s, r) - atan2(L₃₄ sin γ, a₂ + L₃₄ cos γ)].\n"
        "• Spherical Wrist IK (Joints 4–6) & Upright Box B Invariant:  R₃⁶ = (R₀³)ᵀ R₀⁶.\n"
        "  For upright Box B orientation R₀⁶ = diag(1, -1, -1) (Roll = 0°, Pitch = 0°, Yaw = 0°):\n"
        "      q₄(t) = 0°,     q₅(t) = 90° - (q₂(t) + q₃(t)),     q₆(t) = q₁(t) ± 180°.\n"
        "• Differential Kinematics (Velocity Control):  q̇(t) = Jᵀ(q) [J(q) Jᵀ(q) + λ² I]⁻¹ [v_TCP(t)ᵀ, 0, 0, 0]ᵀ."
    )
    ax.text(0.06, y, fk_ik_txt, va="top", fontsize=8.4, linespacing=1.38, family="monospace")

    pdf.savefig(fig)
    plt.close(fig)


def add_figures_page(pdf, fig_files, page_title):
    fig = plt.figure(figsize=(8.27, 11.69), dpi=200)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.axis("off")
    ax.text(0.5, 0.965, page_title, ha="center", va="top", fontsize=13, fontweight="bold", color="#0d47a1")

    n = len(fig_files)
    top = 0.93
    height = 0.86 / n
    for i, fname in enumerate(fig_files):
        img = mpimg.imread(os.path.join(FIG_DIR, fname))
        sub = fig.add_axes([0.05, top - (i + 1) * height + 0.01, 0.90, height - 0.02])
        sub.imshow(img)
        sub.axis("off")

    pdf.savefig(fig)
    plt.close(fig)


if __name__ == "__main__":
    with PdfPages(PDF_PATH) as pdf:
        add_title_and_kinematics_page(pdf)
        add_figures_page(
            pdf,
            ["fig2_velocity_profiles.png", "fig3_trajectory_3d.png"],
            "3. Smooth Velocity Profiles & 3D Upright Pick-and-Place Trajectory",
        )
        add_figures_page(
            pdf,
            ["fig4_joint_angles_velocities.png", "fig5_upright_orientation_verification.png"],
            "4. Joint Angles, Jacobian Velocity Control & Upright Orientation Verification",
        )
        add_figures_page(
            pdf,
            ["fig6_simulation_sequence.png"],
            "5. 3D CAD Pick-and-Place Simulation Sequence (Box A at (0.5 m, 0.5 m, 0))",
        )
    print(f"Saved compiled PDF report to {PDF_PATH}")
