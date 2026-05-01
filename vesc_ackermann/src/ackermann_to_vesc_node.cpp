#include <rclcpp/rclcpp.hpp>

#include "vesc_ackermann/ackermann_to_vesc.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vesc_ackermann::AckermannToVesc>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
