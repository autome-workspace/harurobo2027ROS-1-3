#ifndef HARUROBO_BASE__SINGLE_WHEEL_INTERLOCK_HPP_
#define HARUROBO_BASE__SINGLE_WHEEL_INTERLOCK_HPP_

#include <chrono>
#include <optional>

namespace harurobo_base
{
// Offline policy only. This class has no connection to CAN or a ROS command topic.
class SingleWheelInterlock
{
public:
  using Clock = std::chrono::steady_clock;

  explicit SingleWheelInterlock(bool independent_stop_verified = false)
  : independent_stop_verified_(independent_stop_verified) {}

  void feedback(bool healthy, Clock::time_point now)
  {
    if (armed_ && (!last_feedback_ || now - *last_feedback_ >= feedback_timeout_)) {
      stop();
    }
    if (!healthy) {
      fault();
      return;
    }
    last_feedback_ = now;
  }

  void fault()
  {
    stop();
    last_feedback_.reset();
  }

  bool rearm(bool neutral, Clock::time_point now)
  {
    if (armed_ || !neutral || !independent_stop_verified_ || !fresh_feedback(now)) {
      return false;
    }
    armed_ = true;
    last_command_ = now;
    return true;
  }

  bool accept_command(Clock::time_point now)
  {
    if (!allowed(now)) {
      return false;
    }
    last_command_ = now;
    return true;
  }

  bool allowed(Clock::time_point now)
  {
    if (armed_ && (!fresh_feedback(now) || !last_command_ ||
      now - *last_command_ >= command_timeout_))
    {
      stop();
    }
    return armed_;
  }

  void stop()
  {
    armed_ = false;
    last_command_.reset();
  }

private:
  bool fresh_feedback(Clock::time_point now) const
  {
    return last_feedback_ && now - *last_feedback_ < feedback_timeout_;
  }

  static constexpr std::chrono::milliseconds feedback_timeout_{100};
  static constexpr std::chrono::milliseconds command_timeout_{150};
  const bool independent_stop_verified_;
  bool armed_{};
  std::optional<Clock::time_point> last_feedback_, last_command_;
};
}  // namespace harurobo_base

#endif  // HARUROBO_BASE__SINGLE_WHEEL_INTERLOCK_HPP_
