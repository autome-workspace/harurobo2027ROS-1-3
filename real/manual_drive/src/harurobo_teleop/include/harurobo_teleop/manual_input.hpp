#ifndef HARUROBO_TELEOP__MANUAL_INPUT_HPP_
#define HARUROBO_TELEOP__MANUAL_INPUT_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace harurobo_teleop
{
struct Input
{
  double x{};
  double y{};
  double yaw{};
  bool pressed{};
};

struct Command
{
  double vx{};
  double vy{};
  double wz{};
  bool armed{};
};

class ManualInput
{
public:
  using Clock = std::chrono::steady_clock;
  struct Config
  {
    double deadzone{0.08};
    double max_linear{0.25};
    double max_angular{0.5};
    std::chrono::milliseconds timeout{150};
    double yaw_deadzone{0.08};
  };

  explicit ManualInput(Config config) : config_(config)
  {
    if (!(config_.deadzone >= 0.0 && config_.deadzone < 1.0) ||
      !(config_.yaw_deadzone >= 0.0 && config_.yaw_deadzone < 1.0) ||
      !std::isfinite(config_.max_linear) || config_.max_linear <= 0.0 ||
      !std::isfinite(config_.max_angular) || config_.max_angular <= 0.0 ||
      config_.timeout.count() <= 0)
    {
      throw std::invalid_argument("Invalid manual input limits or timeout");
    }
  }

  void invalidate()
  {
    state_ = State::waiting_for_release;
    last_input_.reset();
    command_ = {};
  }

  void update(Input input, Clock::time_point now)
  {
    expire(now);  // Latch gaps even if a new sample arrives before the timer runs.
    if (!valid_axis(input.x) || !valid_axis(input.y) || !valid_axis(input.yaw)) {
      invalidate();
      return;
    }
    last_input_ = now;
    const double magnitude = std::hypot(input.x, input.y);
    const double rotation = std::abs(input.yaw);
    const bool neutral = magnitude <= config_.deadzone && rotation <= config_.yaw_deadzone;
    if (!input.pressed) {
      state_ = neutral ? State::ready : State::waiting_for_release;
      command_ = {};
      return;
    }
    if (state_ == State::ready) {
      state_ = neutral ? State::armed : State::waiting_for_release;
    }
    if (state_ != State::armed) {
      return;
    }
    const double scale = magnitude > config_.deadzone ?
      config_.max_linear * shape(magnitude, config_.deadzone) / magnitude : 0.0;
    command_ = {input.x * scale, input.y * scale,
      std::copysign(config_.max_angular * shape(rotation, config_.yaw_deadzone), input.yaw), true};
  }

  Command command(Clock::time_point now)
  {
    expire(now);
    return command_;
  }

private:
  enum class State { waiting_for_release, ready, armed };

  static bool valid_axis(double value)
  {
    return std::isfinite(value) && std::abs(value) <= 1.0;
  }

  static double shape(double magnitude, double deadzone)
  {
    return std::clamp((magnitude - deadzone) / (1.0 - deadzone), 0.0, 1.0);
  }

  void expire(Clock::time_point now)
  {
    if (last_input_ && now - *last_input_ >= config_.timeout) {
      invalidate();
    }
  }

  Config config_;
  State state_{State::waiting_for_release};
  std::optional<Clock::time_point> last_input_;
  Command command_{};
};
}  // namespace harurobo_teleop

#endif  // HARUROBO_TELEOP__MANUAL_INPUT_HPP_
