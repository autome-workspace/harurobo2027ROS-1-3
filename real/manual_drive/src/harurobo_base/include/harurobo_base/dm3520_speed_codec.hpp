#ifndef HARUROBO_BASE__DM3520_SPEED_CODEC_HPP_
#define HARUROBO_BASE__DM3520_SPEED_CODEC_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>

namespace harurobo_base
{
// Offline codec only: no transport, enable, stop, or firmware-specific policy.
// Reference: DM-S3519-1EC / DM3520-1EC manual, "Control Frame in Speed Mode".
struct Dm3520SpeedFrame
{
  std::uint16_t standard_id;
  std::array<std::uint8_t, 4> payload;
};

inline std::optional<Dm3520SpeedFrame> encode_dm3520_speed(
  std::uint16_t motor_id, float output_rad_s)
{
  static_assert(sizeof(float) == sizeof(std::uint32_t) && std::numeric_limits<float>::is_iec559);
  // The manual describes matching motor addresses by the low eight bits.
  if (motor_id > 0xff || !std::isfinite(output_rad_s)) {
    return std::nullopt;
  }
  std::uint32_t bits{};
  std::memcpy(&bits, &output_rad_s, sizeof(bits));
  Dm3520SpeedFrame frame{static_cast<std::uint16_t>(0x200 + motor_id), {}};
  for (auto & byte : frame.payload) {
    byte = static_cast<std::uint8_t>(bits & 0xffu);
    bits >>= 8;
  }
  return frame;
}
// DLC 8 variant observed in the existing bench script; still offline only.
struct Dm3520PaddedSpeedFrame
{
  std::uint16_t standard_id;
  std::array<std::uint8_t, 8> payload;
};

inline std::optional<Dm3520PaddedSpeedFrame> encode_dm3520_padded_speed(
  std::uint16_t motor_id, float output_rad_s)
{
  const auto compact = encode_dm3520_speed(motor_id, output_rad_s);
  if (!compact) {
    return std::nullopt;
  }
  return Dm3520PaddedSpeedFrame{compact->standard_id, {
    compact->payload[0], compact->payload[1], compact->payload[2], compact->payload[3]}};
}

}  // namespace harurobo_base

#endif  // HARUROBO_BASE__DM3520_SPEED_CODEC_HPP_
