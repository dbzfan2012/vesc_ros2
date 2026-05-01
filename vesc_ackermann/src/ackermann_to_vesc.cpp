// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.h"

#include <cmath>
#include <sstream>

#include <std_msgs/msg/float64.hpp>

namespace vesc_ackermann
{

AckermannToVesc::AckermannToVesc() : Node("ackermann_to_vesc")
{
  // get conversion parameters
  speed_to_erpm_gain_ = this->declare_parameter<double>("speed_to_erpm_gain", 0.0);
  speed_to_erpm_offset_ = this->declare_parameter<double>("speed_to_erpm_offset", 0.0);
  steering_to_servo_gain_ = this->declare_parameter<double>("steering_angle_to_servo_gain", 0.0);
  steering_to_servo_offset_ = this->declare_parameter<double>("steering_angle_to_servo_offset", 0.0);

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("commands/motor/speed", 10);
  servo_pub_ = this->create_publisher<std_msgs::msg::Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, std::placeholders::_1));
}

void AckermannToVesc::ackermannCmdCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr cmd)
{
  // calc vesc electric RPM (speed)
  std_msgs::msg::Float64 erpm_msg;
  erpm_msg.data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // calc steering angle (servo)
  std_msgs::msg::Float64 servo_msg;
  servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  // publish
  erpm_pub_->publish(erpm_msg);
  servo_pub_->publish(servo_msg);
}

} // namespace vesc_ackermann
