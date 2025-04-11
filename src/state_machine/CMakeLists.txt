cmake_minimum_required(VERSION 3.8)
project(state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)  # Critical for MoveGroupInterface
find_package(geometry_msgs REQUIRED)


# Add executable with the new name
add_executable(robot_move_node src/robot_move_node.cpp)

ament_target_dependencies(robot_move_node 
  rclcpp 
  moveit_ros_planning_interface 
  geometry_msgs
)

install(TARGETS
  robot_move_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
