// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_DRIVER_VESC_DRIVER_H_
#define VESC_DRIVER_VESC_DRIVER_H_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <boost/optional.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"

namespace vesc_driver
{

class VescDriver : public rclcpp::Node
{
public:

  VescDriver();

private:
  // interface to the VESC
  VescInterface vesc_;
  void vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet);
  void vescErrorCallback(const std::string& error);

  // limits on VESC commands
  struct CommandLimit
  {
    CommandLimit(rclcpp::Node* node, const std::string& str,
                 const boost::optional<double>& min_lower = boost::optional<double>(),
                 const boost::optional<double>& max_upper = boost::optional<double>());
    double clip(double value);
    rclcpp::Node* node;
    std::string name;
    boost::optional<double> lower;
    boost::optional<double> upper;
  };
  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit position_limit_;
  CommandLimit servo_limit_;

  // ROS services
  rclcpp::Publisher<vesc_msgs::msg::VescStateStamped>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_sensor_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr duty_cycle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr brake_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // driver modes (possible states)
  typedef enum {
    MODE_INITIALIZING,
    MODE_OPERATING
  } driver_mode_t;

  // other variables
  driver_mode_t driver_mode_;           ///< driver state machine mode (state)
  int fw_version_major_;                ///< firmware major version reported by vesc
  int fw_version_minor_;                ///< firmware minor version reported by vesc

  // ROS callbacks
  void timerCallback();
  void dutyCycleCallback(const std_msgs::msg::Float64::SharedPtr duty_cycle);
  void currentCallback(const std_msgs::msg::Float64::SharedPtr current);
  void brakeCallback(const std_msgs::msg::Float64::SharedPtr brake);
  void speedCallback(const std_msgs::msg::Float64::SharedPtr speed);
  void positionCallback(const std_msgs::msg::Float64::SharedPtr position);
  void servoCallback(const std_msgs::msg::Float64::SharedPtr servo);
};

} // namespace vesc_driver

#endif // VESC_DRIVER_VESC_DRIVER_H_
