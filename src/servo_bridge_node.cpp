#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "px4_msgs/msg/actuator_servos.hpp"
#include "px4_msgs/msg/manual_control_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ServoBridgeNode : public rclcpp::Node
{
public:
  ServoBridgeNode()
  : Node("servo_bridge_node")
  {
    declare_parameter<bool>("arm", false);
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&ServoBridgeNode::parameter_callback, this, std::placeholders::_1));

    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto mode_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    servo_pub_ = create_publisher<px4_msgs::msg::ActuatorServos>(
      "/fmu/in/actuator_servos", px4_qos);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", px4_qos);
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", px4_qos,
      std::bind(&ServoBridgeNode::vehicle_status_callback, this, std::placeholders::_1));
    manual_control_sub_ = create_subscription<px4_msgs::msg::ManualControlSetpoint>(
      "/fmu/out/manual_control_setpoint", px4_qos,
      std::bind(&ServoBridgeNode::manual_control_callback, this, std::placeholders::_1));
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/servo/mode", mode_qos,
      std::bind(&ServoBridgeNode::mode_callback, this, std::placeholders::_1));
    control_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/servo/control", 10,
      std::bind(&ServoBridgeNode::control_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(50ms, std::bind(&ServoBridgeNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Servo Bridge Node Started");
    RCLCPP_INFO(get_logger(), "RC: AUX1, R2: /servo/control, output: actuator_servos control[0]");
  }

private:
  static double clamp(double value)
  {
    return std::clamp(value, -1.0, 1.0);
  }

  uint64_t now_us()
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  double now_sec()
  {
    return static_cast<double>(get_clock()->now().nanoseconds()) / 1.0e9;
  }

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & parameter : parameters) {
      if (parameter.get_name() != "arm") {
        continue;
      }
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "arm parameter must be bool";
        return result;
      }

      arm_parameter_value_ = parameter.as_bool();
      if (arm_parameter_value_) {
        arm();
        RCLCPP_INFO(get_logger(), "ROS 2 parameter: ARM requested");
      } else {
        disarm();
        RCLCPP_INFO(get_logger(), "ROS 2 parameter: DISARM requested");
      }
    }
    return result;
  }

  void publish_vehicle_command(uint16_t command, float param1)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = now_us();
    msg.param1 = param1;
    msg.param2 = 0.0F;
    msg.param3 = 0.0F;
    msg.param4 = 0.0F;
    msg.param5 = 0.0F;
    msg.param6 = 0.0F;
    msg.param7 = 0.0F;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void arm()
  {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
    RCLCPP_INFO(get_logger(), "Sent PX4 ARM command");
  }

  void disarm()
  {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F);
    RCLCPP_INFO(get_logger(), "Sent PX4 DISARM command");
  }

  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    nav_state_ = msg->nav_state;
    arming_state_ = msg->arming_state;
  }

  void manual_control_callback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg)
  {
    if (!msg->valid || !std::isfinite(msg->aux1)) {
      rc_valid_ = false;
      return;
    }
    rc_aux1_ = clamp(msg->aux1);
    rc_valid_ = true;
    last_rc_time_ = now_sec();
  }

  void mode_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    auto requested_mode = msg->data;
    std::transform(
      requested_mode.begin(), requested_mode.end(), requested_mode.begin(),
      [](unsigned char value) {return static_cast<char>(std::toupper(value));});
    if (requested_mode != "RC" && requested_mode != "R2") {
      RCLCPP_WARN(get_logger(), "Invalid servo mode: %s", requested_mode.c_str());
      return;
    }
    if (requested_mode != control_mode_) {
      RCLCPP_INFO(
        get_logger(), "Servo Mode Changed: %s -> %s",
        control_mode_.c_str(), requested_mode.c_str());
    }
    control_mode_ = requested_mode;
    mode_received_ = true;
  }

  void control_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      r2_valid_ = false;
      return;
    }
    r2_control_ = clamp(msg->data);
    r2_valid_ = true;
    last_r2_time_ = now_sec();
  }

  bool is_rc_active()
  {
    return rc_valid_ && (now_sec() - last_rc_time_) <= rc_timeout_;
  }

  bool is_r2_active()
  {
    return r2_valid_ && (now_sec() - last_r2_time_) <= r2_timeout_;
  }

  void select_control_source()
  {
    if (!mode_received_) {
      control_source_ = "WAIT_MODE";
      target_servo_ = current_servo_;
    } else if (control_mode_ == "RC") {
      if (is_rc_active()) {
        control_source_ = "RC";
        target_servo_ = rc_aux1_;
      } else {
        control_source_ = "RC_HOLD";
        target_servo_ = current_servo_;
      }
    } else if (control_mode_ == "R2") {
      if (is_r2_active()) {
        control_source_ = "R2";
        target_servo_ = r2_control_;
      } else {
        control_source_ = "R2_HOLD";
        target_servo_ = current_servo_;
      }
    } else {
      control_source_ = "UNKNOWN_HOLD";
      target_servo_ = current_servo_;
    }
  }

  void update_smooth_servo()
  {
    const double error = target_servo_ - current_servo_;
    if (std::abs(error) <= max_step_) {
      current_servo_ = target_servo_;
    } else {
      current_servo_ += error > 0.0 ? max_step_ : -max_step_;
    }
    current_servo_ = clamp(current_servo_);
  }

  void publish_servo_command()
  {
    update_smooth_servo();
    px4_msgs::msg::ActuatorServos msg;
    const auto timestamp = now_us();
    msg.timestamp = timestamp;
    msg.timestamp_sample = timestamp;
    msg.control.fill(std::numeric_limits<float>::quiet_NaN());
    msg.control[0] = static_cast<float>(current_servo_);
    servo_pub_->publish(msg);
  }

  void timer_callback()
  {
    select_control_source();
    publish_servo_command();

    if (log_counter_ % 20 == 0) {
      const bool armed = arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
      RCLCPP_INFO(
        get_logger(),
        "mode=%s, source=%s, nav=%d, arming=%d, armed=%s, arm_param=%s, "
        "rc_active=%s, rc=%.3f, r2_active=%s, r2=%.3f, target=%.3f, servo=%.3f",
        control_mode_.c_str(), control_source_.c_str(), nav_state_, arming_state_,
        armed ? "true" : "false", arm_parameter_value_ ? "true" : "false",
        is_rc_active() ? "true" : "false", rc_aux1_,
        is_r2_active() ? "true" : "false", r2_control_, target_servo_, current_servo_);
    }
    ++log_counter_;
  }

  bool arm_parameter_value_{false};
  int nav_state_{-1};
  int arming_state_{-1};
  std::string control_mode_{"HOLD"};
  bool mode_received_{false};
  bool rc_valid_{false};
  double rc_aux1_{0.0};
  double last_rc_time_{0.0};
  const double rc_timeout_{0.5};
  bool r2_valid_{false};
  double r2_control_{0.0};
  double last_r2_time_{0.0};
  const double r2_timeout_{0.5};
  double current_servo_{0.0};
  double target_servo_{0.0};
  std::string control_source_{"HOLD"};
  const double max_step_{0.03};
  std::size_t log_counter_{0};

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr servo_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr control_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
