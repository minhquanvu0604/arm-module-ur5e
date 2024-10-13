cmake_minimum_required(VERSION 3.0.2)
project(arm_module_ur5e_controller)

# Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(PythonLibs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  geometry_msgs
  std_srvs
  message_generation
)

# Add service files
add_service_files(
  FILES
  MoveToPose.srv
  PoseService.srv
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs std_srvs
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY python/src/
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
        FILES_MATCHING PATTERN "*.py")

file(GLOB_RECURSE SCRIPT_FILES python/scripts/*.py)
catkin_install_python(PROGRAMS ${SCRIPT_FILES}
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

