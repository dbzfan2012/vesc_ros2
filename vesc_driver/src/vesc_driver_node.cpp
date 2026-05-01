#include <rclcpp/rclcpp.hpp>

#include "vesc_driver/vesc_driver.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vesc_driver::VescDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
