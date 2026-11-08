cmake_minimum_required(VERSION 3.5)
project(swarm_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python packages
ament_python_install_package(scripts)

# Install Python scripts (exclude __pycache__ directories)
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