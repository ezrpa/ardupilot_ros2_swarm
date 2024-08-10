cmake_minimum_required(VERSION 2.8.3)
project(robot_pose_publisher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp tf)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}1 src/robot_pose_publisher1.cpp)
add_executable(${PROJECT_NAME}2 src/robot_pose_publisher2.cpp)
add_executable(${PROJECT_NAME}3 src/robot_pose_publisher3.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}1
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}2
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}3
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}1
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}2
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}3
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
