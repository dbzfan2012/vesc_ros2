#include <rclcpp/rclcpp.hpp>

#include "vesc_ackermann/vesc_to_odom.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vesc_ackermann::VescToOdom>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
