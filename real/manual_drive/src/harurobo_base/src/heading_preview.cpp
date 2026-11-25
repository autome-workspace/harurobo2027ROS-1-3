#include "harurobo_base/heading_hold.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace harurobo_base
{
using namespace std::chrono_literals;
class HeadingPreview final : public rclcpp::Node
{
public:
  HeadingPreview() : Node("harurobo_heading_preview")
  {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    const double kp = declare_parameter("kp", 0.5, fixed);
    const double kd = declare_parameter("kd", 0.0, fixed);
    const double limit = declare_parameter("max_correction_radps", 0.2, fixed);
    controller_.emplace(kp, kd, limit);
    output_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel/manual/heading_preview", 1);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);
    imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS().keep_last(1),
      [this](const sensor_msgs::msg::Imu & message) {
        const auto & q = message.orientation;
        const auto & w = message.angular_velocity;
        const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
        if (message.header.frame_id != "imu_link" || message.orientation_covariance[0] < 0 ||
          message.angular_velocity_covariance[0] < 0 || !std::isfinite(norm) ||
          std::abs(norm - 1.0) > 1e-3 || !std::isfinite(w.x) || !std::isfinite(w.y) ||
          !std::isfinite(w.z) || !stamp_ok(rclcpp::Time(message.header.stamp), 100ms) ||
          (last_imu_stamp_ && rclcpp::Time(message.header.stamp) <= *last_imu_stamp_))
        {
          valid_imu_ = false; latched_ = true; controller_->reset(); return;
        }
        yaw_ = std::atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z));
        gyro_z_ = w.z;
        last_imu_stamp_ = rclcpp::Time(message.header.stamp);
        imu_received_ = Clock::now(); valid_imu_ = true;
      });
    cmd_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel/manual", rclcpp::QoS(1), [this](const geometry_msgs::msg::TwistStamped & message) {
        if (message.header.frame_id != "base_link" ||
          !std::isfinite(message.twist.linear.x) || !std::isfinite(message.twist.linear.y) ||
          !std::isfinite(message.twist.angular.z) || !stamp_ok(rclcpp::Time(message.header.stamp), 150ms) ||
          (last_cmd_stamp_ && rclcpp::Time(message.header.stamp) <= *last_cmd_stamp_))
        {
          valid_command_ = false; latched_ = true; controller_->reset(); return;
        }
        command_ = message;
        command_received_ = Clock::now(); valid_command_ = true;
        last_cmd_stamp_ = rclcpp::Time(message.header.stamp);
      });
    timer_ = create_wall_timer(10ms, [this] {tick();});
    diagnostic_timer_ = create_wall_timer(100ms, [this] {diagnose();});
  }
private:
  using Clock = HeadingHold::Clock;
  bool stamp_ok(const rclcpp::Time & stamp, std::chrono::milliseconds limit)
  {
    const auto age = (now() - stamp).nanoseconds();
    return age >= -50000000 && age < std::chrono::duration_cast<std::chrono::nanoseconds>(limit).count();
  }
  void tick()
  {
    const auto now = Clock::now();
    const bool fresh_imu = valid_imu_ && imu_received_ && now - *imu_received_ < 100ms &&
      last_imu_stamp_ && stamp_ok(*last_imu_stamp_, 100ms);
    const bool fresh_cmd = valid_command_ && command_received_ && now - *command_received_ < 150ms &&
      last_cmd_stamp_ && stamp_ok(*last_cmd_stamp_, 150ms);
    if (!fresh_imu || !fresh_cmd) {latched_ = true; controller_->reset();}
    const bool neutral = std::hypot(command_.twist.linear.x, command_.twist.linear.y) < 1e-6 &&
      std::abs(command_.twist.angular.z) < 1e-6;
    // Preview only: clearing a preview fault is NOT motor re-authorization.
    if (latched_ && fresh_imu && fresh_cmd && neutral) {latched_ = false;}
    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = this->now(); message.header.frame_id = "base_link";
    holding_ = false;
    if (!latched_) {
      const auto result = controller_->update(true, !neutral &&
        std::hypot(command_.twist.linear.x, command_.twist.linear.y) >= 1e-6,
        command_.twist.angular.z, yaw_, gyro_z_, true, now);
      if (!result.healthy) {latched_ = true; controller_->reset();}
      else {
        message.twist.linear.x = command_.twist.linear.x;
        message.twist.linear.y = command_.twist.linear.y;
        message.twist.angular.z = result.wz; holding_ = result.holding;
      }
    }
    output_->publish(message);
  }
  void diagnose()
  {
    using Status = diagnostic_msgs::msg::DiagnosticStatus;
    Status status;
    status.name = "harurobo_base: heading preview (NO CAN)";
    status.hardware_id = "preview_only";
    status.level = latched_ ? Status::WARN : Status::OK;
    status.message = latched_ ? "Input fault; fresh inputs and neutral required" :
      (holding_ ? "Heading correction preview" : "Manual/neutral preview");
    diagnostic_msgs::msg::KeyValue value;
    value.key = "drive_output"; value.value = "disabled; no motor permission";
    status.values.push_back(value);
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now(); message.status.push_back(status); diagnostics_->publish(message);
  }
  std::optional<HeadingHold> controller_;
  std::optional<Clock::time_point> imu_received_, command_received_;
  std::optional<rclcpp::Time> last_imu_stamp_, last_cmd_stamp_;
  bool valid_imu_{}, valid_command_{}, latched_{true}, holding_{};
  double yaw_{}, gyro_z_{};
  geometry_msgs::msg::TwistStamped command_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_, diagnostic_timer_;
};
}  // namespace harurobo_base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<harurobo_base::HeadingPreview>());}
  catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("heading_preview"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
