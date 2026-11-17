# EN3563 - Robotics Mini Project: YASKAWA GP7 Pick-and-Place (ROS 2 Jazzy & Gazebo Harmonic)

This repository contains the complete ROS 2 Jazzy & Gazebo Harmonic (`gz-sim8`) implementation for the **EN3563 Robotics Mini Project**:
- **Robot Arm:** 6-DoF YASKAWA GP7 (`Assem1`) with a 2-jaw prismatic parallel gripper (`link7`, `link8`) placed at the origin `(0, 0, 0)`.
- **Box A ($30\text{ cm} \times 30\text{ cm} \times 20\text{ cm}$):** Placed at `(0.5 m, 0.5 m, 0)` (top surface center at `(0.5 m, 0.5 m, 0.20 m)`).
- **Box B ($5\text{ cm} \times 5\text{ cm} \times 5\text{ cm}$):** Placed at a random reachable coordinate on the ground (`z = 0`).
- **Motion Task:** Picks up Box B from the ground and places it at the **center of the top surface of Box A** (`(0.5 m, 0.5 m, 0.225 m)`) while keeping **Box B strictly upright** ($\text{Roll} = 0^\circ, \text{Pitch} = 0^\circ, \text{Yaw} = 0^\circ$) with a smooth **S-Curve** or **Trapezoidal** velocity profile.

---

## Quick-Start Instructions

### 1. Build the ROS 2 Workspace
Open a terminal in `/home/chirantha-akarsha/Robotics` and run:
```bash
cd /home/chirantha-akarsha/Robotics
source /opt/ros/jazzy/setup.bash
colcon build --packages-select Assem1
source install/setup.bash
```

### 2. Run the Full Pick-and-Place Simulation in Gazebo
To launch Gazebo Harmonic (`gz sim`) with the **Full SolidWorks CAD Model**, **Box A** at `(0.5 m, 0.5 m, 0)`, and **Box B** at a random reachable ground coordinate:
```bash
source /opt/ros/jazzy/setup.bash
source /home/chirantha-akarsha/Robotics/install/setup.bash
ros2 launch Assem1 gazebo_pick_and_place.launch.py
```

#### Optional Launch Parameters:
- **Use Simplified D-H Geometric Model** (per Implementation Notes):
  ```bash
  ros2 launch Assem1 gazebo_pick_and_place.launch.py model_type:=simplified
  ```
- **Switch Velocity Profile (`s_curve` or `trapezoidal`):**
  ```bash
  ros2 launch Assem1 gazebo_pick_and_place.launch.py velocity_profile:=trapezoidal
  ```
- **Specify a Fixed Ground Coordinate for Box B:**
  ```bash
  ros2 launch Assem1 gazebo_pick_and_place.launch.py random_box_b:=false box_b_x:=-0.35 box_b_y:=0.45
  ```
- **Open RViz2 Alongside Gazebo (to see live Coordinate Frames / TF):**
  ```bash
  ros2 launch Assem1 gazebo_pick_and_place.launch.py use_rviz:=true
  ```

### 3. Visualize Link Coordinate Frames Interactively in RViz2
To open RViz2 with all link coordinate frames (`world`, `base_link`, `link1`..`link6`, `tcp_link`, `link7`, `link8`) and interactive joint sliders:
```bash
source /opt/ros/jazzy/setup.bash
source /home/chirantha-akarsha/Robotics/install/setup.bash
ros2 launch Assem1 display_rviz.launch.py
```

### 4. Run Standalone Kinematics & Trajectory Verification Scripts
- **Verify Symbolic D-H Matrices & Analytical Inverse Kinematics:**
  ```bash
  python3 /home/chirantha-akarsha/Robotics/Assem1/scripts/kinematics.py
  ```
- **Verify Trajectory Planner & Upright Orientation Error:**
  ```bash
  python3 /home/chirantha-akarsha/Robotics/Assem1/scripts/trajectory_planner.py
  ```
- **Regenerate All Report Figures & PDF Report:**
  ```bash
  python3 /home/chirantha-akarsha/Robotics/Assem1/scripts/generate_report_figures.py
  python3 /home/chirantha-akarsha/Robotics/Assem1/scripts/build_pdf_report.py
  ```

---

## Project Files & Deliverables
- **Report (PDF & Markdown):**
  - `Project_Report.pdf`
  - `Project_Report.md`
  - `report_figures/fig1_coordinate_frames_dh.png` ... `fig6_simulation_sequence.png`
- **URDF Models:**
  - `Assem1/urdf/Assem1.urdf` (Full SolidWorks CAD Model with Gazebo plugins)
  - `Assem1/urdf/Assem1_simplified.urdf` (Simplified Geometric D-H Model)
- **Kinematics, Trajectory Planning & Control Code:**
  - `Assem1/scripts/kinematics.py`
  - `Assem1/scripts/trajectory_planner.py`
  - `Assem1/scripts/pick_and_place_node.py`
- **ROS 2 Launch Files:**
  - `Assem1/launch/gazebo_pick_and_place.launch.py`
  - `Assem1/launch/display_rviz.launch.py`
