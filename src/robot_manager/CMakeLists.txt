cmake_minimum_required(VERSION 3.8)
project(robot_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_library(manager_node SHARED
  src/manager_node.cpp
)
target_include_directories(manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(manager_node
  rclcpp
  rclcpp_components
  robot_interfaces
  lifecycle_msgs
)

rclcpp_components_register_node(manager_node
  PLUGIN robot_manager::ManagerNode
  EXECUTABLE manager_component_node
)

install(TARGETS manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
