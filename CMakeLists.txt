cmake_minimum_required(VERSION 3.0.2)
project(simple_ur5_controller)

# Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_service_files(
  FILES
  MoveToPose.srv
  PoseService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# Declare a catkin package
catkin_package()

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

