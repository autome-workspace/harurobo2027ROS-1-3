#include "harurobo_imu/protocol.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <string>
#include <system_error>
#include <stdexcept>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

namespace harurobo_imu
{
using namespace std::chrono_literals;
class Receiver final : public rclcpp::Node
{
public:
  Receiver() : Node("harurobo_imu"), assembler_(1)
  {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    port_ = declare_parameter<std::string>("port", "", fixed);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link", fixed);
    if (port_.empty() || frame_id_.empty()) {throw std::invalid_argument("port and frame_id required");}
    pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS().keep_last(1));
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);
    timer_ = create_wall_timer(2ms, [this] {receive();});
    diagnostic_timer_ = create_wall_timer(100ms, [this] {diagnose();});
  }
  ~Receiver() override {disconnect();}
private:
  using Clock = Assembler::Clock;
  void disconnect()
  {
    if (fd_ >= 0) {::close(fd_); fd_ = -1;}
    parser_.reset(); assembler_.reset(); last_sample_.reset();
  }
  static void check(int result, const char * operation)
  {
    if (result < 0) {throw std::system_error(errno, std::generic_category(), operation);}
  }
  void receive()
  {
    const auto now = Clock::now();
    if (fd_ < 0) {
      if (now < retry_at_) {return;}
      retry_at_ = now + 1s;
      try {
        fd_ = ::open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        check(fd_, "open IMU");
        check(::ioctl(fd_, TIOCEXCL), "exclusive IMU");
        termios options{};
        check(::tcgetattr(fd_, &options), "tcgetattr");
        ::cfmakeraw(&options);
        options.c_cflag |= CLOCAL | CREAD;
        options.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS | CSIZE);
        options.c_cflag |= CS8;
        check(::cfsetispeed(&options, B921600), "input baud");
        check(::cfsetospeed(&options, B921600), "output baud");
        check(::tcsetattr(fd_, TCSANOW, &options), "tcsetattr");
        check(::tcflush(fd_, TCIFLUSH), "flush IMU");
        parser_.reset(); assembler_.reset(); error_.clear();
      } catch (const std::exception & e) {error_ = e.what(); disconnect(); return;}
    }
    if (last_sample_ && now - *last_sample_ >= 100ms) {
      assembler_.reset();
    }
    int pending = 0;
    if (::ioctl(fd_, FIONREAD, &pending) < 0) {fail("IMU ioctl failed"); return;}
    if (pending > 4096) {
      ++overruns_; ::tcflush(fd_, TCIFLUSH); parser_.reset(); assembler_.reset();
      last_sample_.reset(); error_ = "discarded serial backlog"; return;
    }
    std::uint8_t bytes[4096];
    const auto count = ::read(fd_, bytes, sizeof(bytes));
    if (count < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {fail("IMU read failed");}
      return;
    }
    for (ssize_t i = 0; i < count; ++i) {
      const auto invalid = parser_.invalid_frames;
      auto frame = parser_.feed(bytes[i]);
      if (parser_.invalid_frames != invalid) {assembler_.reset();}
      if (!frame) {continue;}
      auto sample = assembler_.accept(*frame, now);
      if (!sample) {continue;}
      sensor_msgs::msg::Imu message;
      message.header.stamp = this->now();
      message.header.frame_id = frame_id_;
      const auto q = quaternion(sample->euler_degrees);
      message.orientation.x = q[0]; message.orientation.y = q[1];
      message.orientation.z = q[2]; message.orientation.w = q[3];
      message.angular_velocity.x = sample->gyro[0];
      message.angular_velocity.y = sample->gyro[1];
      message.angular_velocity.z = sample->gyro[2];
      message.linear_acceleration.x = sample->accel[0];
      message.linear_acceleration.y = sample->accel[1];
      message.linear_acceleration.z = sample->accel[2];
      pub_->publish(message);
      ++samples_; last_sample_ = now; error_.clear();
    }
  }
  void fail(const std::string & reason) {error_ = reason; disconnect(); retry_at_ = Clock::now() + 1s;}
  void diagnose()
  {
    using Status = diagnostic_msgs::msg::DiagnosticStatus;
    Status status;
    status.name = "harurobo_imu: receive-only";
    status.hardware_id = port_;
    status.level = fd_ < 0 ? Status::ERROR :
      (!last_sample_ || Clock::now() - *last_sample_ >= 100ms ? Status::STALE : Status::OK);
    status.message = status.level == Status::OK ? "CRC-checked samples; axes need hardware verification" :
      (error_.empty() ? "No fresh complete IMU sample" : error_);
    auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key; pair.value = value; status.values.push_back(pair);
      };
    add("protocol_writes", "disabled");
    add("valid_frames", std::to_string(parser_.valid_frames));
    add("invalid_frames", std::to_string(parser_.invalid_frames));
    add("skipped_quaternions", std::to_string(parser_.skipped_quaternions));
    add("samples", std::to_string(samples_)); add("backlog_drops", std::to_string(overruns_));
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = this->now(); message.status.push_back(status); diagnostics_->publish(message);
  }
  int fd_{-1};
  Parser parser_;
  Assembler assembler_;
  Clock::time_point retry_at_{Clock::time_point::min()};
  std::optional<Clock::time_point> last_sample_;
  std::string port_, frame_id_, error_;
  std::uint64_t samples_{}, overruns_{};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_, diagnostic_timer_;
};
}  // namespace harurobo_imu

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<harurobo_imu::Receiver>());}
  catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("harurobo_imu"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
