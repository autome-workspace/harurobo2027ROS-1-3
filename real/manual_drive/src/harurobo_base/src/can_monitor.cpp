#include "harurobo_base/socketcan_receiver.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace harurobo_base
{
using namespace std::chrono_literals;
using Status = diagnostic_msgs::msg::DiagnosticStatus;

class CanMonitor final : public rclcpp::Node
{
public:
  CanMonitor() : Node("harurobo_can_monitor")
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    interface_ = declare_parameter<std::string>("interface", "can0", descriptor);
    enable_fd_ = declare_parameter<bool>("enable_fd", false, descriptor);
    const auto timeout = declare_parameter<int>("traffic_timeout_ms", 100, descriptor);
    if (timeout <= 0 || timeout > 10000) {
      throw std::invalid_argument("traffic_timeout_ms must be in [1, 10000]");
    }
    timeout_ = std::chrono::milliseconds(timeout);
    pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);
    read_timer_ = create_wall_timer(10ms, [this] {read();});
    diagnostic_timer_ = create_wall_timer(100ms, [this] {publish();});
  }

private:
  using Clock = std::chrono::steady_clock;

  void read()
  {
    const auto now = Clock::now();
    if (!receiver_.is_open()) {
      if (now < retry_at_) {
        return;
      }
      retry_at_ = now + 1s;
      try {
        receiver_.open(interface_, enable_fd_);
        link_error_.clear();
      } catch (const std::exception & e) {
        link_error_ = e.what();
        return;
      }
    }
    // Bound work per timer invocation; never drive control from this aggregate stream.
    for (unsigned i = 0; i < 128; ++i) {
      const auto result = receiver_.receive();
      using State = SocketCanReceiver::ReadResult::State;
      if (result.state == State::empty) {
        return;
      }
      if (result.state == State::interrupted) {
        continue;
      }
      if (result.state == State::failed) {
        link_error_ = std::error_code(result.error, std::generic_category()).message();
        if (latched_fault_.empty()) {
          latched_fault_ = "SocketCAN receive failed: " + link_error_;
        }
        last_data_.reset();
        last_id_.reset();
        receiver_.close();
        retry_at_ = now + 1s;
        return;
      }
      switch (classify_frame(result.frame, result.size)) {
        case FrameKind::classic:
          ++classic_count_;
          observe_data(result.frame, now);
          break;
        case FrameKind::fd:
          ++fd_count_;
          observe_data(result.frame, now);
          break;
        case FrameKind::error:
          ++error_count_;
          last_error_mask_ = result.frame.can_id & CAN_ERR_MASK;
          last_error_ = now;
          if ((last_error_mask_ & CAN_ERR_BUSOFF) && latched_fault_.empty()) {
            latched_fault_ = "SocketCAN bus-off observed (latched)";
          }
          break;
        case FrameKind::remote_request:
          ++rtr_count_;
          break;
        case FrameKind::invalid:
          ++invalid_count_;
          break;
      }
    }
    ++read_budget_count_;
  }

  void observe_data(const canfd_frame & frame, Clock::time_point now)
  {
    last_data_ = now;
    last_id_ = frame.can_id & ((frame.can_id & CAN_EFF_FLAG) ? CAN_EFF_MASK : CAN_SFF_MASK);
    last_extended_ = frame.can_id & CAN_EFF_FLAG;
  }

  static void add(Status & status, const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue field;
    field.key = key;
    field.value = value;
    status.values.push_back(field);
  }

  void publish()
  {
    const auto now = Clock::now();
    Status status;
    status.name = "harurobo_base: SocketCAN monitor";
    status.hardware_id = interface_;
    if (!latched_fault_.empty()) {
      status.level = Status::ERROR;
      status.message = latched_fault_;
    } else if (!receiver_.is_open()) {
      status.level = Status::ERROR;
      status.message = "SocketCAN unavailable";
    } else if (last_error_ && now - *last_error_ < 1s) {
      status.level = Status::WARN;
      status.message = "Recent CAN error frame";
    } else if (!last_data_ || now - *last_data_ >= timeout_) {
      status.level = Status::STALE;
      status.message = "No recent CAN data frames";
    } else {
      status.level = Status::OK;
      status.message = "Raw CAN data present; motor feedback not validated";
    }
    add(status, "application_frames_tx", "disabled (receive only)");
    add(status, "can_fd_rx", enable_fd_ ? "true" : "false");
    add(status, "socket_open", receiver_.is_open() ? "true" : "false");
    add(status, "link_error", link_error_);
    add(status, "classic_rx", std::to_string(classic_count_));
    add(status, "fd_rx", std::to_string(fd_count_));
    add(status, "remote_request_rx", std::to_string(rtr_count_));
    add(status, "invalid_rx", std::to_string(invalid_count_));
    add(status, "error_rx", std::to_string(error_count_));
    add(status, "last_error_mask_decimal", std::to_string(last_error_mask_));
    add(status, "rx_budget_hits", std::to_string(read_budget_count_));
    if (last_data_) {
      add(status, "last_data_age_ms", std::to_string(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - *last_data_).count()));
    }
    if (last_id_) {
      add(status, "last_data_id_decimal", std::to_string(*last_id_));
      add(status, "last_data_extended_id", last_extended_ ? "true" : "false");
    }
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = this->now();
    message.status.push_back(status);
    pub_->publish(message);
  }

  SocketCanReceiver receiver_;
  std::string interface_;
  bool enable_fd_{};
  bool last_extended_{};
  std::chrono::milliseconds timeout_{};
  Clock::time_point retry_at_{Clock::time_point::min()};
  std::optional<Clock::time_point> last_data_, last_error_;
  std::optional<canid_t> last_id_;
  can_err_mask_t last_error_mask_{};
  std::uint64_t classic_count_{}, fd_count_{}, error_count_{}, rtr_count_{}, invalid_count_{};
  std::uint64_t read_budget_count_{};
  std::string link_error_, latched_fault_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr read_timer_, diagnostic_timer_;
};
}  // namespace harurobo_base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<harurobo_base::CanMonitor>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("harurobo_can_monitor"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
