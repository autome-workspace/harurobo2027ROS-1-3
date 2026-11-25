#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "harurobo_teleop/manual_input.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

namespace harurobo_teleop
{
class TeleopNode final : public rclcpp::Node
{
public:
  TeleopNode() : Node("harurobo_teleop")
  {
    axis_x_ = index("axis_vx", 1);
    axis_y_ = index("axis_vy", 0);
    axis_yaw_ = index("axis_wz", 2);
    button_ = index("deadman_button", 9);
    sign_x_ = sign("sign_vx");
    sign_y_ = sign("sign_vy");
    sign_yaw_ = sign("sign_wz");
    ManualInput::Config config;
    config.deadzone = parameter("deadzone", config.deadzone);
    config.yaw_deadzone = parameter("yaw_deadzone", config.yaw_deadzone);
    config.max_linear = parameter("max_linear_mps", config.max_linear);
    config.max_angular = parameter("max_angular_radps", config.max_angular);
    const int timeout_ms = parameter("joy_timeout_ms", 150);
    if (timeout_ms < 20 || timeout_ms > 240) {
      throw std::invalid_argument("joy_timeout_ms must be in [20, 240]");
    }
    config.timeout = std::chrono::milliseconds(timeout_ms);
    input_.emplace(config);
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel/manual", 1);
    armed_pub_ = create_publisher<std_msgs::msg::Bool>("/manual/armed", 1);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&TeleopNode::on_joy, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] {publish();});
  }

private:
  template<class T>
  T parameter(const std::string & name, T fallback)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, fallback, descriptor);
  }

  std::size_t index(const std::string & name, int fallback)
  {
    const int value = parameter(name, fallback);
    if (value < 0) {
      throw std::invalid_argument(name + " must not be negative");
    }
    return static_cast<std::size_t>(value);
  }

  double sign(const std::string & name)
  {
    const int value = parameter(name, 1);
    if (value != -1 && value != 1) {
      throw std::invalid_argument(name + " must be -1 or +1");
    }
    return value;
  }

  void on_joy(const sensor_msgs::msg::Joy & msg)
  {
    if (axis_x_ >= msg.axes.size() || axis_y_ >= msg.axes.size() ||
      axis_yaw_ >= msg.axes.size() || button_ >= msg.buttons.size() ||
      (msg.buttons[button_] != 0 && msg.buttons[button_] != 1))
    {
      input_->invalidate();
      return;
    }
    input_->update({sign_x_ * msg.axes[axis_x_], sign_y_ * msg.axes[axis_y_],
      sign_yaw_ * msg.axes[axis_yaw_], msg.buttons[button_] == 1}, ManualInput::Clock::now());
  }

  void publish()
  {
    const auto command = input_->command(ManualInput::Clock::now());
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = now();
    twist.header.frame_id = "base_link";
    twist.twist.linear.x = command.vx;
    twist.twist.linear.y = command.vy;
    twist.twist.angular.z = command.wz;
    cmd_pub_->publish(twist);
    // Display only. A future motor receiver needs a freshness-checked, atomic command/permit.
    std_msgs::msg::Bool armed;
    armed.data = command.armed;
    armed_pub_->publish(armed);
  }

  std::size_t axis_x_, axis_y_, axis_yaw_, button_;
  double sign_x_, sign_y_, sign_yaw_;
  std::optional<ManualInput> input_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace harurobo_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<harurobo_teleop::TeleopNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("harurobo_teleop"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
