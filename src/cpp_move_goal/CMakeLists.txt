cmake_minimum_required(VERSION 3.8)
project(cpp_move_goal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(/opt/ros/humble/include/graph_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(Boost REQUIRED)
find_package(moveit_common)
moveit_package()

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

find_package(control_msgs)
find_package(Eigen3)
find_package(geometric_shapes)
find_package(graph_msgs REQUIRED)
find_package(interactive_markers)
find_package(moveit_core)
find_package(moveit)
find_package(moveit_msgs)
find_package(moveit_ros_planning)
find_package(moveit_ros_planning_interface)
find_package(moveit_ros_move_group)
find_package(moveit_servo)
find_package(moveit_task_constructor_core)
find_package(moveit_visual_tools)
find_package(pluginlib)
find_package(rclcpp)
find_package(rclcpp_action)
find_package(rviz_visual_tools)
find_package(tf2_geometry_msgs)
find_package(tf2_ros)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  pluginlib
  moveit_core
  moveit_ros_planning
  Boost
  rclcpp
  rclcpp_action
  moveit_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
)
  

add_executable(move_trial
    src/moveit_trial.cpp)
target_include_directories(move_trial
  PUBLIC include)
ament_target_dependencies(move_trial
  ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS move_trial
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(hello_moveit
src/tutorial.cpp)
target_include_directories(hello_moveit
PUBLIC include)
install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME}
)
ament_target_dependencies(hello_moveit
  ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
