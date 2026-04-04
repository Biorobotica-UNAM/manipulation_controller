cmake_minimum_required(VERSION 3.8)
project(manipulation_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(arms_interfaces REQUIRED)
find_package(torso_interfaces REQUIRED)
find_package(manipulation_controller_interfaces REQUIRED)

add_executable(manipulation_controller_node
  src/manipulation_controller_node.cpp
)

ament_target_dependencies(manipulation_controller_node
  rclcpp
  geometry_msgs
  arms_interfaces
  torso_interfaces
  manipulation_controller_interfaces
)

install(TARGETS manipulation_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
