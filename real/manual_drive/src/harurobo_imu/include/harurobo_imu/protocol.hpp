#ifndef HARUROBO_IMU__PROTOCOL_HPP_
#define HARUROBO_IMU__PROTOCOL_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <limits>
#include <vector>

namespace harurobo_imu
{
inline std::uint16_t crc16(const std::uint8_t * bytes, std::size_t count)
{
  std::uint16_t crc = 0xffff;
  for (std::size_t i = 0; i < count; ++i) {
    crc ^= static_cast<std::uint16_t>(bytes[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = static_cast<std::uint16_t>((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
  }
  return crc;
}

struct Frame {std::uint8_t slave, rid; std::array<float, 3> values;};

class Parser
{
public:
  std::optional<Frame> feed(std::uint8_t byte)
  {
    buffer_.push_back(byte);
    while (buffer_.size() >= 2 && (buffer_[0] != 0x55 || buffer_[1] != 0xaa)) {
      buffer_.erase(buffer_.begin());
    }
    if (buffer_.size() < 4) {return std::nullopt;}
    const std::size_t size = buffer_[3] == 4 ? 23 : 19;
    if (buffer_.size() < size) {return std::nullopt;}
    const auto expected = static_cast<std::uint16_t>(
      buffer_[size-3] | (buffer_[size-2] << 8));
    if (buffer_[size-1] != 0x0a || buffer_[3] < 1 || buffer_[3] > 4 ||
      crc16(buffer_.data(), size-3) != expected)
    {
      ++invalid_frames;
      buffer_.erase(buffer_.begin());
      return std::nullopt;
    }
    if (buffer_[3] == 4) {
      // Quaternion output is optional; validate its framing/CRC, but use Euler.
      ++skipped_quaternions; buffer_.clear(); return std::nullopt;
    }
    Frame frame{buffer_[2], buffer_[3], {}};
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
    for (std::size_t i = 0; i < 3; ++i) {
      std::uint32_t bits = 0;
      for (int j = 0; j < 4; ++j) {bits |= std::uint32_t(buffer_[4 + i * 4 + j]) << (8 * j);}
      std::memcpy(&frame.values[i], &bits, 4);
    }
    buffer_.clear();
    for (float value : frame.values) {
      if (!std::isfinite(value)) {++invalid_frames; return std::nullopt;}
    }
    ++valid_frames;
    return frame;
  }
  void reset() {buffer_.clear();}
  std::uint64_t valid_frames{}, invalid_frames{}, skipped_quaternions{};
private:
  std::vector<std::uint8_t> buffer_;
};

struct Sample {std::array<float, 3> accel, gyro, euler_degrees;};

// Require a fresh ordered 1/2/3 set. Any missing RID cannot reuse an old value.
class Assembler
{
public:
  using Clock = std::chrono::steady_clock;
  explicit Assembler(std::uint8_t slave) : slave_(slave) {}
  void reset() {accel_.reset(); gyro_.reset();}
  std::optional<Sample> accept(const Frame & frame, Clock::time_point now)
  {
    if (frame.slave != slave_) {return std::nullopt;}
    if (accel_ && now - start_ > std::chrono::milliseconds(20)) {reset();}
    if (frame.rid == 1) {
      reset(); accel_ = frame.values; start_ = now;
    } else if (frame.rid == 2 && accel_) {
      gyro_ = frame.values;
    } else if (frame.rid == 3 && accel_ && gyro_) {
      Sample result{*accel_, *gyro_, frame.values}; reset(); return result;
    } else {
      reset();
    }
    return std::nullopt;
  }
private:
  std::uint8_t slave_;
  Clock::time_point start_{};
  std::optional<std::array<float, 3>> accel_, gyro_;
};

inline std::array<double, 4> quaternion(const std::array<float, 3> & degrees)
{
  constexpr double half_degree = 3.14159265358979323846 / 360.;
  const double cr = std::cos(degrees[0] * half_degree), sr = std::sin(degrees[0] * half_degree);
  const double cp = std::cos(degrees[1] * half_degree), sp = std::sin(degrees[1] * half_degree);
  const double cy = std::cos(degrees[2] * half_degree), sy = std::sin(degrees[2] * half_degree);
  std::array<double, 4> q{sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
    cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy};
  double norm = 0;
  for (double value : q) {norm = std::hypot(norm, value);}
  for (auto & value : q) {value /= norm;}
  return q;
}
}  // namespace harurobo_imu

#endif  // HARUROBO_IMU__PROTOCOL_HPP_
