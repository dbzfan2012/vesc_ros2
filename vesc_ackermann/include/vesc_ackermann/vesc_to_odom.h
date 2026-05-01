// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_ACKERMANN_VESC_TO_ODOM_H_
#define VESC_ACKERMANN_VESC_TO_ODOM_H_

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

class VescToOdom : public rclcpp::Node
{
public:

  VescToOdom();

private:
  // ROS parameters
  std::string odom_frame_;
  std::string base_frame_;
  /** State message does not report servo position, so use the command instead */
  bool use_servo_cmd_;
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;
  double wheelbase_;
  bool publish_tf_;

  // odometry state
  double x_, y_, yaw_;
  std_msgs::msg::Float64::SharedPtr last_servo_cmd_; ///< Last servo position commanded value
  vesc_msgs::msg::VescStateStamped::SharedPtr last_state_; ///< Last received state message

  // ROS services
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

  // ROS callbacks
  void vescStateCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr state);
  void servoCmdCallback(const std_msgs::msg::Float64::SharedPtr servo);
};

} // namespace vesc_ackermann

#endif // VESC_ACKERMANN_VESC_TO_ODOM_H_
