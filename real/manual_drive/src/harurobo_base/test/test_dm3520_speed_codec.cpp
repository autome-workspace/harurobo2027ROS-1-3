#include <limits>

#include "gtest/gtest.h"
#include "harurobo_base/dm3520_speed_codec.hpp"

namespace
{
using harurobo_base::encode_dm3520_speed;
using Payload = std::array<std::uint8_t, 4>;

TEST(Dm3520SpeedCodec, MatchesManualVectorWithoutApplyingGearRatio)
{
  const auto frame = encode_dm3520_speed(3, 6.283f);
  ASSERT_TRUE(frame);
  EXPECT_EQ(frame->standard_id, 0x203);
  EXPECT_EQ(frame->payload, (Payload{0x56, 0x0e, 0xc9, 0x40}));
}

TEST(Dm3520SpeedCodec, KeepsNegativeSignAndSerializesZero)
{
  const auto negative = encode_dm3520_speed(3, -1.0f);
  const auto zero = encode_dm3520_speed(3, 0.0f);
  ASSERT_TRUE(negative);
  ASSERT_TRUE(zero);
  EXPECT_EQ(negative->payload, (Payload{0, 0, 0x80, 0xbf}));
  EXPECT_EQ(zero->payload, (Payload{0, 0, 0, 0}));
}

TEST(Dm3520SpeedCodec, RejectsAddressesOutsideLowByteAndNonfiniteSpeeds)
{
  EXPECT_FALSE(encode_dm3520_speed(0x100, 0.0f));
  EXPECT_FALSE(encode_dm3520_speed(3, std::numeric_limits<float>::infinity()));
  EXPECT_FALSE(encode_dm3520_speed(3, std::numeric_limits<float>::quiet_NaN()));
}
TEST(Dm3520SpeedCodec, MatchesExistingBenchDlc8Padding)
{
  const auto frame = harurobo_base::encode_dm3520_padded_speed(1, 1.0f);
  ASSERT_TRUE(frame);
  EXPECT_EQ(frame->standard_id, 0x201);
  EXPECT_EQ(frame->payload, (std::array<std::uint8_t, 8>{0, 0, 0x80, 0x3f, 0, 0, 0, 0}));
  EXPECT_FALSE(harurobo_base::encode_dm3520_padded_speed(0x100, 1.0f));
}
}  // namespace
