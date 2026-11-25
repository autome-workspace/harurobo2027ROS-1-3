#ifndef HARUROBO_BASE__HEADING_HOLD_HPP_
#define HARUROBO_BASE__HEADING_HOLD_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace harurobo_base
{
// Offline controller. Caller must latch IMU/CAN/input faults before motor permission.
class HeadingHold
{
public:
  using Clock = std::chrono::steady_clock;
  struct Result {double wz; bool healthy; bool holding;};

  HeadingHold(double kp = 1.0, double kd = 0.0, double limit = 0.2)
  : kp_(kp), kd_(kd), limit_(limit)
  {
    if (!std::isfinite(kp) || !std::isfinite(kd) || !std::isfinite(limit) ||
      kp < 0 || kd < 0 || limit <= 0)
    {
      throw std::invalid_argument("invalid heading gains/limit");
    }
  }

  void reset() {target_.reset(); settled_since_.reset();}

  Result update(
    bool permitted, bool translating, double manual_wz, double yaw, double gyro_z,
    bool imu_fresh, Clock::time_point now)
  {
    if (!permitted) {
      reset();
      return {0, true, false};
    }
    if (!imu_fresh || !std::isfinite(yaw) || !std::isfinite(gyro_z) ||
      !std::isfinite(manual_wz))
    {
      reset();
      return {0, false, false};
    }
    if (manual_wz != 0) {
      reset();
      return {manual_wz, true, false};
    }
    if (!translating) {
      reset();
      return {0, true, false};
    }
    if (!target_) {
      // Do not capture residual rotation. Require 200 ms below 0.05 rad/s.
      if (std::abs(gyro_z) > 0.05) {
        settled_since_.reset();
        return {0, true, false};
      }
      if (!settled_since_) {settled_since_ = now;}
      if (now - *settled_since_ < std::chrono::milliseconds(200)) {
        return {0, true, false};
      }
      target_ = yaw;
    }
    constexpr double tau = 6.28318530717958647692;
    const double error = std::remainder(*target_ - yaw, tau);
    const double correction = kp_ * error - kd_ * gyro_z;
    if (!std::isfinite(correction)) {
      reset();
      return {0, false, false};
    }
    return {std::clamp(correction, -limit_, limit_), true, true};
  }

private:
  double kp_, kd_, limit_;
  std::optional<double> target_;
  std::optional<Clock::time_point> settled_since_;
};
}  // namespace harurobo_base

#endif  // HARUROBO_BASE__HEADING_HOLD_HPP_
