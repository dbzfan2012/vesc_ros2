cmake_minimum_required(VERSION 3.8)
project(vesc_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(io_context REQUIRED)
find_package(serial_driver REQUIRED)

include_directories(
  include
)

# node executable
add_executable(vesc_driver_node src/vesc_driver_node.cpp
                                src/vesc_driver.cpp
                                src/vesc_interface.cpp
                                src/vesc_packet.cpp
                                src/vesc_packet_factory.cpp)
ament_target_dependencies(vesc_driver_node
  io_context
  rclcpp
  serial_driver
  std_msgs
  vesc_msgs
)

# component library
add_library(vesc_driver_component SHARED
  src/vesc_driver.cpp
  src/vesc_interface.cpp
  src/vesc_packet.cpp
  src/vesc_packet_factory.cpp
)
ament_target_dependencies(vesc_driver_component
  io_context
  rclcpp
  rclcpp_components
  serial_driver
  std_msgs
  vesc_msgs
)
rclcpp_components_register_nodes(vesc_driver_component "vesc_driver::VescDriver")

#############
## Install ##
#############

install(TARGETS vesc_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS vesc_driver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(PROGRAMS
  src/throttle_interpolator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
