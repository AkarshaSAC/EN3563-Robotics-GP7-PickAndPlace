cmake_minimum_required(VERSION 3.8)
project(Assem1)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes textures urdf worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/pick_and_place_node.py
    scripts/kinematics.py
    scripts/trajectory_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
