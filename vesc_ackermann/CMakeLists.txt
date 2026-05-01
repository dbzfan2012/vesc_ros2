cmake_minimum_required(VERSION 3.8)
project(vesc_ackermann)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)

include_directories(
  include
)

# node executables
add_executable(vesc_to_odom_node src/vesc_to_odom_node.cpp
                                 src/vesc_to_odom.cpp)
ament_target_dependencies(vesc_to_odom_node
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  vesc_msgs
)

add_executable(ackermann_to_vesc_node src/ackermann_to_vesc_node.cpp
                                      src/ackermann_to_vesc.cpp)
ament_target_dependencies(ackermann_to_vesc_node
  rclcpp
  std_msgs
  ackermann_msgs
  vesc_msgs
)

# component library
add_library(vesc_ackermann_component SHARED
  src/ackermann_to_vesc.cpp
  src/vesc_to_odom.cpp
)
ament_target_dependencies(vesc_ackermann_component
  rclcpp
  rclcpp_components
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ackermann_msgs
  vesc_msgs
)
rclcpp_components_register_nodes(vesc_ackermann_component "vesc_ackermann::AckermannToVesc")
rclcpp_components_register_nodes(vesc_ackermann_component "vesc_ackermann::VescToOdom")

#############
## Install ##
#############

install(TARGETS vesc_to_odom_node ackermann_to_vesc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS vesc_ackermann_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()