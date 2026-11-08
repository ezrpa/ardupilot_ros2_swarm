cmake_minimum_required(VERSION 3.5)
project(swarm_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# The importable package: swarm_control.{core,mission,perception,tak,nodes}
ament_python_install_package(${PROJECT_NAME})

# Executables: thin entry-point wrappers, installed as lib/swarm_control/<name>.py
# so launch files, compose and `ros2 run swarm_control <name>.py` keep working.
file(GLOB SCRIPT_FILES_SCRIPTS
  "scripts/*.py"
)

install(PROGRAMS
  ${SCRIPT_FILES_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  parameters
  DESTINATION share/${PROJECT_NAME}
)

ament_package()