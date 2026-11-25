#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "harurobo_teleop/manual_input.hpp"

namespace
{
using namespace std::chrono_literals;
using harurobo_teleop::Command;
using harurobo_teleop::ManualInput;
const ManualInput::Clock::time_point start{};

void expect_stopped(Command command)
{
  EXPECT_FALSE(command.armed);
  EXPECT_DOUBLE_EQ(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
  EXPECT_DOUBLE_EQ(command.wz, 0.0);
}

TEST(ManualInput, StartupRequiresNeutralReleaseAndNeutralPress)
{
  ManualInput input({});
  expect_stopped(input.command(start));
  input.update({0.0, 0.0, 0.0, true}, start);
  expect_stopped(input.command(start));
  input.update({0.0, 0.0, 0.0, false}, start + 10ms);
  input.update({0.5, 0.0, 0.0, true}, start + 20ms);
  expect_stopped(input.command(start + 20ms));
  input.update({0.0, 0.0, 0.0, true}, start + 30ms);
  expect_stopped(input.command(start + 30ms));
  input.update({0.0, 0.0, 0.0, false}, start + 40ms);
  input.update({0.0, 0.0, 0.0, true}, start + 50ms);
  EXPECT_TRUE(input.command(start + 50ms).armed);
}

TEST(ManualInput, RadialDeadzoneDiagonalCapAndSignedRotation)
{
  ManualInput input({});
  input.update({}, start);
  input.update({0.0, 0.0, 0.0, true}, start + 10ms);
  input.update({0.05, -0.05, -0.05, true}, start + 20ms);
  const auto neutral = input.command(start + 20ms);
  EXPECT_TRUE(neutral.armed);
  EXPECT_DOUBLE_EQ(neutral.vx, 0.0);
  EXPECT_DOUBLE_EQ(neutral.vy, 0.0);
  EXPECT_DOUBLE_EQ(neutral.wz, 0.0);
  input.update({1.0, -1.0, -1.0, true}, start + 30ms);
  const auto diagonal = input.command(start + 30ms);
  EXPECT_NEAR(std::hypot(diagonal.vx, diagonal.vy), 0.25, 1e-12);
  EXPECT_GT(diagonal.vx, 0.0);
  EXPECT_LT(diagonal.vy, 0.0);
  EXPECT_DOUBLE_EQ(diagonal.wz, -0.5);
}

TEST(ManualInput, ReleasingDeadmanStopsImmediately)
{
  ManualInput input({});
  input.update({}, start);
  input.update({0.0, 0.0, 0.0, true}, start + 10ms);
  input.update({1.0, 0.0, 0.0, true}, start + 20ms);
  ASSERT_DOUBLE_EQ(input.command(start + 20ms).vx, 0.25);
  input.update({1.0, 0.0, 0.0, false}, start + 30ms);
  expect_stopped(input.command(start + 30ms));
  input.update({0.0, 0.0, 0.0, true}, start + 40ms);
  expect_stopped(input.command(start + 40ms));
}

TEST(ManualInput, TimeoutAtDeadlineRequiresRearm)
{
  ManualInput input({});
  input.update({}, start);
  input.update({0.0, 0.0, 0.0, true}, start + 10ms);
  input.update({0.5, 0.0, 0.0, true}, start + 20ms);
  EXPECT_TRUE(input.command(start + 169ms).armed);
  expect_stopped(input.command(start + 170ms));
  input.update({0.5, 0.0, 0.0, true}, start + 171ms);
  expect_stopped(input.command(start + 171ms));
  input.update({}, start + 180ms);
  input.update({0.0, 0.0, 0.0, true}, start + 190ms);
  EXPECT_TRUE(input.command(start + 190ms).armed);
}

TEST(ManualInput, IncomingMessageCannotHideTimeout)
{
  ManualInput input({});
  input.update({}, start);
  input.update({0.0, 0.0, 0.0, true}, start + 10ms);
  input.update({1.0, 0.0, 0.0, true}, start + 160ms);
  expect_stopped(input.command(start + 160ms));
}

TEST(ManualInput, RejectsNonFiniteAndOutOfRangeInputs)
{
  ManualInput input({});
  input.update({}, start);
  input.update({0.0, 0.0, 0.0, true}, start + 10ms);
  input.update({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, true}, start + 20ms);
  expect_stopped(input.command(start + 20ms));
  input.update({0.0, 0.0, 2.0, false}, start + 30ms);
  input.update({0.0, 0.0, 0.0, true}, start + 40ms);
  expect_stopped(input.command(start + 40ms));
}
TEST(ManualInput, IndependentYawDeadzoneSuppressesNoiseWithoutChangingTranslation)
{
  ManualInput::Config config;
  config.yaw_deadzone = 0.12;
  ManualInput input(config);
  input.update({0, 0, -0.10, false}, start);
  input.update({0, 0, -0.10, true}, start + 10ms);
  ASSERT_TRUE(input.command(start + 10ms).armed);
  input.update({1, 0, -0.10, true}, start + 20ms);
  EXPECT_DOUBLE_EQ(input.command(start + 20ms).vx, .25);
  EXPECT_DOUBLE_EQ(input.command(start + 20ms).wz, 0);
  input.update({0, 0, .56, true}, start + 30ms);
  EXPECT_NEAR(input.command(start + 30ms).wz, .25, 1e-12);
}

}  // namespace
