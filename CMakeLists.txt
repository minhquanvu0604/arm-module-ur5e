cmake_minimum_required(VERSION 3.8)
project(simple-UR5-planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME})



  
# robot_model_and_robot_state ------------------------------------------------------

# add_executable(robot_model_and_robot_state src/robot_model_and_robot_state_tutorial.cpp)
# target_include_directories(robot_model_and_robot_state PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(robot_model_and_robot_state PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   robot_model_and_robot_state
#   "moveit_ros_planning_interface"
#   "rclcpp"
# )
add_executable(robot_model_and_robot_state_tutorial
    src/robot_model_and_robot_state_tutorial.cpp)
target_include_directories(robot_model_and_robot_state_tutorial
    PUBLIC include)
ament_target_dependencies(robot_model_and_robot_state_tutorial
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS robot_model_and_robot_state_tutorial
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
