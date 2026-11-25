#include "gtest/gtest.h"
#include "harurobo_imu/protocol.hpp"

namespace
{
using namespace std::chrono_literals;
using harurobo_imu::Parser;
using harurobo_imu::Assembler;
using harurobo_imu::Frame;
std::array<std::uint8_t, 19> packet(std::uint8_t rid, float value = 1)
{
  std::array<std::uint8_t, 19> bytes{0x55, 0xaa, 1, rid};
  std::uint32_t bits{};
  std::memcpy(&bits, &value, 4);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {bytes[4+i*4+j] = (bits >> (8*j)) & 0xff;}
  }
  const auto crc = harurobo_imu::crc16(bytes.data(), 16);
  bytes[16] = crc & 0xff; bytes[17] = crc >> 8; bytes[18] = 0x0a;
  return bytes;
}

TEST(ImuProtocol, CrcMatchesCcittKnownVector)
{
  const std::uint8_t bytes[]{'1','2','3','4','5','6','7','8','9'};
  EXPECT_EQ(harurobo_imu::crc16(bytes, sizeof(bytes)), 0x29b1);
}

TEST(ImuProtocol, ReassemblesBytewiseAndResynchronizesAfterGarbage)
{
  Parser parser;
  for (auto b : {0, 0x55, 0x55, 0}) {EXPECT_FALSE(parser.feed(b));}
  std::optional<Frame> result;
  for (auto byte : packet(2, -0.5f)) {result = parser.feed(byte);}
  ASSERT_TRUE(result);
  EXPECT_EQ(result->rid, 2);
  EXPECT_FLOAT_EQ(result->values[2], -0.5f);
  EXPECT_EQ(parser.valid_frames, 1u);
}

TEST(ImuProtocol, RejectsCorruptionAndNonfinite)
{
  Parser parser;
  auto bytes = packet(1); bytes[5] ^= 0x40;
  for (auto b : bytes) {EXPECT_FALSE(parser.feed(b));}
  for (auto b : packet(1, std::numeric_limits<float>::quiet_NaN())) {EXPECT_FALSE(parser.feed(b));}
  EXPECT_GE(parser.invalid_frames, 2u);
  std::optional<Frame> result;
  for (auto b : packet(3)) {result = parser.feed(b);}
  EXPECT_TRUE(result);
}

TEST(ImuProtocol, ValidatesAndSkipsOptionalQuaternionWithoutFalseErrors)
{
  Parser parser;
  std::array<std::uint8_t, 23> bytes{0x55, 0xaa, 1, 4, 0, 0, 0x80, 0x3f};
  const auto crc = harurobo_imu::crc16(bytes.data(), 20);
  bytes[20] = crc & 0xff; bytes[21] = crc >> 8; bytes[22] = 0x0a;
  for (auto b : bytes) {EXPECT_FALSE(parser.feed(b));}
  EXPECT_EQ(parser.skipped_quaternions, 1u);
  EXPECT_EQ(parser.invalid_frames, 0u);
  std::optional<Frame> result;
  for (auto b : packet(1)) {result = parser.feed(b);}
  EXPECT_TRUE(result);
}

TEST(ImuProtocol, RejectsBadRidAndFooter)
{
  Parser parser;
  for (auto b : packet(5)) {EXPECT_FALSE(parser.feed(b));}
  auto bytes = packet(1); bytes[18] = 0x0d;
  for (auto b : bytes) {EXPECT_FALSE(parser.feed(b));}
}

TEST(ImuProtocol, RequiresNewCompleteOrderedSet)
{
  Assembler assembler(1);
  const auto t = Assembler::Clock::time_point{};
  EXPECT_FALSE(assembler.accept(Frame{1, 1, {}}, t));
  EXPECT_FALSE(assembler.accept(Frame{1, 3, {}}, t));
  EXPECT_FALSE(assembler.accept(Frame{1, 2, {}}, t));
  EXPECT_FALSE(assembler.accept(Frame{1, 3, {}}, t));
  assembler.accept(Frame{1, 1, {1,2,3}}, t);
  assembler.accept(Frame{1, 2, {4,5,6}}, t);
  const auto sample = assembler.accept(Frame{1, 3, {7,8,9}}, t);
  ASSERT_TRUE(sample);
  EXPECT_FLOAT_EQ(sample->gyro[2], 6);
  EXPECT_FALSE(assembler.accept(Frame{1, 3, {}}, t));
}

TEST(ImuProtocol, DropsExpiredAndDisconnectedSamples)
{
  Assembler assembler(1);
  const auto t = Assembler::Clock::time_point{};
  assembler.accept(Frame{1, 1, {}}, t);
  assembler.accept(Frame{1, 2, {}}, t + 1ms);
  EXPECT_FALSE(assembler.accept(Frame{1, 3, {}}, t + 21ms));
  assembler.accept(Frame{1, 1, {}}, t);
  assembler.accept(Frame{1, 2, {}}, t);
  assembler.reset();
  EXPECT_FALSE(assembler.accept(Frame{1, 3, {}}, t));
  EXPECT_FALSE(assembler.accept(Frame{2, 1, {}}, t));
}

TEST(ImuProtocol, QuaternionDegreesNormalizedAndCcwPositive)
{
  const auto q = harurobo_imu::quaternion({0,0,90});
  EXPECT_NEAR(q[0], 0, 1e-12);
  EXPECT_NEAR(q[1], 0, 1e-12);
  EXPECT_NEAR(q[2], std::sqrt(.5), 1e-12);
  EXPECT_NEAR(q[3], std::sqrt(.5), 1e-12);
}
}  // namespace
