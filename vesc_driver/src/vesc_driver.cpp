// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_driver.h"

#include <cassert>
#include <cmath>
#include <sstream>

#include <boost/bind.hpp>

namespace vesc_driver
{

VescDriver::VescDriver() :
  Node("vesc_driver"),
  vesc_(std::string(),
        boost::bind(&VescDriver::vescPacketCallback, this, _1),
        boost::bind(&VescDriver::vescErrorCallback, this, _1)),
  duty_cycle_limit_(this, "duty_cycle", -1.0, 1.0), current_limit_(this, "current"),
  brake_limit_(this, "brake"), speed_limit_(this, "speed"),
  position_limit_(this, "position"), servo_limit_(this, "servo", 0.0, 1.0),
  driver_mode_(MODE_INITIALIZING), fw_version_major_(-1), fw_version_minor_(-1)
{
  // get vesc serial port address
  std::string port = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");

  // attempt to connect to the serial port
  try {
    vesc_.connect(port);
  }
  catch (const SerialException & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to connect to the VESC, %s.", e.what());
    rclcpp::shutdown();
    return;
  }

  // create vesc state (telemetry) publisher
  state_pub_ = this->create_publisher<vesc_msgs::msg::VescStateStamped>("sensors/core", 10);

  // since vesc state does not include the servo position, publish the commanded
  // servo position as a "sensor"
  servo_sensor_pub_ = this->create_publisher<std_msgs::msg::Float64>("sensors/servo_position_command", 10);

  // subscribe to motor and servo command topics
  duty_cycle_sub_ = this->create_subscription<std_msgs::msg::Float64>("commands/motor/duty_cycle", 10,
                                 std::bind(&VescDriver::dutyCycleCallback, this, std::placeholders::_1));
  current_sub_ = this->create_subscription<std_msgs::msg::Float64>("commands/motor/current", 10, std::bind(&VescDriver::currentCallback, this, std::placeholders::_1));
  brake_sub_ = this->create_subscription<std_msgs::msg::Float64>("commands/motor/brake", 10, std::bind(&VescDriver::brakeCallback, this, std::placeholders::_1));
  speed_sub_ = this->create_subscription<std_msgs::msg::Float64>("commands/motor/speed", 10, std::bind(&VescDriver::speedCallback, this, std::placeholders::_1));
  position_sub_ = this->create_subscription<std_msgs::msg::Float64>("commands/motor/position", 10, std::bind(&VescDriver::positionCallback, this, std::placeholders::_1));
  servo_sub_ = this->create_subscription<std_msgs::msg::Float64>("commands/servo/position", 10, std::bind(&VescDriver::servoCallback, this, std::placeholders::_1));

  // create a 50Hz timer, used for state machine & polling VESC telemetry
  timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&VescDriver::timerCallback, this));
}

  /* TODO or TO-THINKABOUT LIST
    - what should we do on startup? send brake or zero command?
    - what to do if the vesc interface gives an error?
    - check version number against know compatable?
    - should we wait until we receive telemetry before sending commands?
    - should we track the last motor command
    - what to do if no motor command received recently?
    - what to do if no servo command received recently?
    - what is the motor safe off state (0 current?)
    - what to do if a command parameter is out of range, ignore?
    - try to predict vesc bounds (from vesc config) and command detect bounds errors
  */

void VescDriver::timerCallback()
{
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected()) {
    RCLCPP_FATAL(this->get_logger(), "Unexpectedly disconnected from serial port.");
    timer_->cancel();
    rclcpp::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      RCLCPP_INFO(this->get_logger(), "Connected to VESC with firmware version %d.%d",
               fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  }
  else if (driver_mode_ == MODE_OPERATING) {
    // poll for vesc state (telemetry)
    vesc_.requestState();
  }
  else {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void VescDriver::vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet)
{
  if (packet->name() == "Values") {
    boost::shared_ptr<VescPacketValues const> values =
      boost::dynamic_pointer_cast<VescPacketValues const>(packet);

    vesc_msgs::msg::VescStateStamped state_msg;
    state_msg.header.stamp = this->now();
    state_msg.state.voltage_input = values->v_in();
    state_msg.state.temperature_pcb = values->temp_pcb();
    state_msg.state.current_motor = values->current_motor();
    state_msg.state.current_input = values->current_in();
    state_msg.state.speed = values->rpm();
    state_msg.state.duty_cycle = values->duty_now();
    state_msg.state.charge_drawn = values->amp_hours();
    state_msg.state.charge_regen = values->amp_hours_charged();
    state_msg.state.energy_drawn = values->watt_hours();
    state_msg.state.energy_regen = values->watt_hours_charged();
    state_msg.state.displacement = values->tachometer();
    state_msg.state.distance_traveled = values->tachometer_abs();
    state_msg.state.fault_code = values->fault_code();

    state_pub_->publish(state_msg);
  }
  else if (packet->name() == "FWVersion") {
    boost::shared_ptr<VescPacketFWVersion const> fw_version =
      boost::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  }
}

void VescDriver::vescErrorCallback(const std::string& error)
{
  RCLCPP_ERROR(this->get_logger(), "%s", error.c_str());
}

/**
 * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
 *                   note that the VESC may impose a more restrictive bounds on the range depending
 *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
 */
void VescDriver::dutyCycleCallback(const std_msgs::msg::Float64::SharedPtr duty_cycle)
{
  if (driver_mode_ == MODE_OPERATING) {
    vesc_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data));
  }
}

/**
 * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
 *                note that the VESC may impose a more restrictive bounds on the range depending on
 *                its configuration.
 */
void VescDriver::currentCallback(const std_msgs::msg::Float64::SharedPtr current)
{
  if (driver_mode_ == MODE_OPERATING) {
    vesc_.setCurrent(current_limit_.clip(current->data));
  }
}

/**
 * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
 *              However, note that the VESC may impose a more restrictive bounds on the range
 *              depending on its configuration.
 */
void VescDriver::brakeCallback(const std_msgs::msg::Float64::SharedPtr brake)
{
  if (driver_mode_ == MODE_OPERATING) {
    vesc_.setBrake(brake_limit_.clip(brake->data));
  }
}

/**
 * @param speed Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
 *              multiplied by the number of motor poles. Any value is accepted by this
 *              driver. However, note that the VESC may impose a more restrictive bounds on the
 *              range depending on its configuration.
 */
void VescDriver::speedCallback(const std_msgs::msg::Float64::SharedPtr speed)
{
  if (driver_mode_ == MODE_OPERATING) {
    double clipped_speed = speed_limit_.clip(speed->data);
    if (std::abs(clipped_speed) < 900 && clipped_speed != 0.0) // min erpm
        clipped_speed = clipped_speed < 0 ? -920 : 920;
    vesc_.setSpeed(clipped_speed);
  }
}

/**
 * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
 *                 Note that the VESC must be in encoder mode for this command to have an effect.
 */
void VescDriver::positionCallback(const std_msgs::msg::Float64::SharedPtr position)
{
  if (driver_mode_ == MODE_OPERATING) {
    // ROS uses radians but VESC seems to use degrees. Convert to degrees.
    double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
    vesc_.setPosition(position_deg);
  }
}

/**
 * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
 */
void VescDriver::servoCallback(const std_msgs::msg::Float64::SharedPtr servo)
{
  if (driver_mode_ == MODE_OPERATING) {
    double servo_clipped(servo_limit_.clip(servo->data));
    vesc_.setServo(servo_clipped);
    // publish clipped servo value as a "sensor"
    std_msgs::msg::Float64 servo_sensor_msg;
    servo_sensor_msg.data = servo_clipped;
    servo_sensor_pub_->publish(servo_sensor_msg);
  }
}

VescDriver::CommandLimit::CommandLimit(rclcpp::Node* node, const std::string& str,
                                       const boost::optional<double>& min_lower,
                                       const boost::optional<double>& max_upper) :
  node(node),
  name(str)
{
  // check if user's minimum value is outside of the range min_lower to max_upper
  double param_min;
  if (node->get_parameter(name + "_min", param_min)) {
    if (min_lower && param_min < *min_lower) {
      lower = *min_lower;
      RCLCPP_WARN_STREAM(node->get_logger(), "Parameter " << name << "_min (" << param_min <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_min > *max_upper) {
      lower = *max_upper;
      RCLCPP_WARN_STREAM(node->get_logger(), "Parameter " << name << "_min (" << param_min <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else {
      lower = param_min;
    }
  }
  else if (min_lower) {
    lower = *min_lower;
  }

  // check if the uers' maximum value is outside of the range min_lower to max_upper
  double param_max;
  if (node->get_parameter(name + "_max", param_max)) {
    if (min_lower && param_max < *min_lower) {
      upper = *min_lower;
      RCLCPP_WARN_STREAM(node->get_logger(), "Parameter " << name << "_max (" << param_max <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_max > *max_upper) {
      upper = *max_upper;
      RCLCPP_WARN_STREAM(node->get_logger(), "Parameter " << name << "_max (" << param_max <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else {
      upper = param_max;
    }
  }
  else if (max_upper) {
    upper = *max_upper;
  }

  // check for min > max
  if (upper && lower && *lower > *upper) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Parameter " << name << "_max (" << *upper
                    << ") is less than parameter " << name << "_min (" << *lower << ").");
    double temp(*lower);
    lower = *upper;
    upper = temp;
  }

  std::ostringstream oss;
  oss << "  " << name << " limit: ";
  if (lower) oss << *lower << " "; else oss << "(none) ";
  if (upper) oss << *upper; else oss << "(none)";
  RCLCPP_DEBUG_STREAM(node->get_logger(), oss.str());
}

double VescDriver::CommandLimit::clip(double value)
{
  if (lower && value < lower) {
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 10000, "%s command value (%f) below minimum limit (%f), clipping.",
                      name.c_str(), value, *lower);
    return *lower;
  }
  if (upper && value > upper) {
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 10000, "%s command value (%f) above maximum limit (%f), clipping.",
                      name.c_str(), value, *upper);
    return *upper;
  }
  return value;
}


} // namespace vesc_driver
