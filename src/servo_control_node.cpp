#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ServoControlNode : public rclcpp::Node
{
public:
  ServoControlNode()
  : Node("servo_control_node")
  {
    declare_parameter<std::string>("control_mode", "RC");

    auto mode_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    angle_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/servo/angle_cmd", 10,
      std::bind(&ServoControlNode::angle_callback, this, std::placeholders::_1));
    mode_pub_ = create_publisher<std_msgs::msg::String>("/servo/mode", mode_qos);
    control_pub_ = create_publisher<std_msgs::msg::Float32>("/servo/control", 10);
    timer_ = create_wall_timer(50ms, std::bind(&ServoControlNode::timer_callback, this));

    update_mode();
    RCLCPP_INFO(get_logger(), "Servo Control Node Started (mode=%s)", current_mode_.c_str());
    RCLCPP_INFO(get_logger(), "Angle input: /servo/angle_cmd [-90, 0] deg");
  }

private:
  static double clamp(double value, double minimum, double maximum)
  {
    return std::clamp(value, minimum, maximum);
  }

  void update_mode()
  {
    auto requested_mode = get_parameter("control_mode").as_string();
    std::transform(
      requested_mode.begin(), requested_mode.end(), requested_mode.begin(),
      [](unsigned char value) {return static_cast<char>(std::toupper(value));});

    if (requested_mode != "RC" && requested_mode != "R2") {
      RCLCPP_WARN(get_logger(), "Invalid control_mode: %s. Use RC or R2.", requested_mode.c_str());
      return;
    }
    if (requested_mode != current_mode_) {
      RCLCPP_INFO(
        get_logger(), "Servo Mode Changed: %s -> %s",
        current_mode_.c_str(), requested_mode.c_str());
    }
    current_mode_ = requested_mode;
  }

  double angle_to_normalized(double angle_deg) const
  {
    angle_deg = clamp(angle_deg, angle_min_, angle_max_);
    const double normalized =
      2.0 * (angle_deg - angle_min_) / (angle_max_ - angle_min_) - 1.0;
    return clamp(normalized, -1.0, 1.0);
  }

  void angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      RCLCPP_WARN(get_logger(), "Invalid servo angle command");
      return;
    }
    angle_deg_ = clamp(msg->data, angle_min_, angle_max_);
    normalized_control_ = angle_to_normalized(angle_deg_);
    angle_command_received_ = true;
    RCLCPP_INFO(
      get_logger(), "Angle: %.2f deg -> Control: %.3f", angle_deg_, normalized_control_);
  }

  void publish_mode()
  {
    std_msgs::msg::String msg;
    msg.data = current_mode_;
    mode_pub_->publish(msg);
  }

  void publish_control()
  {
    if (!angle_command_received_) {
      return;
    }
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(normalized_control_);
    control_pub_->publish(msg);
  }

  void timer_callback()
  {
    update_mode();
    publish_mode();
    publish_control();

    if (log_counter_ % 20 == 0) {
      RCLCPP_INFO(
        get_logger(), "mode=%s, angle_received=%s, angle=%.2f, control=%.3f",
        current_mode_.c_str(), angle_command_received_ ? "true" : "false",
        angle_deg_, normalized_control_);
    }
    ++log_counter_;
  }

  static constexpr double angle_min_ = -90.0;
  static constexpr double angle_max_ = 0.0;
  std::string current_mode_{"RC"};
  double angle_deg_{0.0};
  double normalized_control_{0.0};
  bool angle_command_received_{false};
  std::size_t log_counter_{0};

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr control_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoControlNode>());
  rclcpp::shutdown();
  return 0;
}
