#include "gtest/gtest.h"
#include "harurobo_base/single_wheel_interlock.hpp"

namespace
{
using namespace std::chrono_literals;
using harurobo_base::SingleWheelInterlock;
constexpr SingleWheelInterlock::Clock::time_point t{};

TEST(SingleWheelInterlock, RequiresVerifiedStopFeedbackAndExplicitNeutralRearm)
{
  SingleWheelInterlock unverified;
  unverified.feedback(true, t);
  EXPECT_FALSE(unverified.rearm(true, t));

  SingleWheelInterlock gate(true);
  EXPECT_FALSE(gate.rearm(true, t));
  gate.feedback(true, t);
  EXPECT_FALSE(gate.rearm(false, t));
  EXPECT_FALSE(gate.accept_command(t));
  ASSERT_TRUE(gate.rearm(true, t));
  EXPECT_TRUE(gate.accept_command(t));
}

TEST(SingleWheelInterlock, LinkFaultRequiresNewFeedbackAndRearm)
{
  SingleWheelInterlock gate(true);
  gate.feedback(true, t);
  ASSERT_TRUE(gate.rearm(true, t));
  gate.fault();
  EXPECT_FALSE(gate.rearm(true, t + 1ms));
  gate.feedback(true, t + 2ms);
  EXPECT_FALSE(gate.accept_command(t + 3ms));
  EXPECT_TRUE(gate.rearm(true, t + 4ms));
}

TEST(SingleWheelInterlock, LateFeedbackDoesNotHideTheFeedbackTimeout)
{
  SingleWheelInterlock gate(true);
  gate.feedback(true, t);
  ASSERT_TRUE(gate.rearm(true, t));
  ASSERT_TRUE(gate.allowed(t + 99ms));
  gate.feedback(true, t + 100ms);
  EXPECT_FALSE(gate.accept_command(t + 100ms));
}

TEST(SingleWheelInterlock, LateCommandDoesNotHideTheCommandTimeout)
{
  SingleWheelInterlock gate(true);
  gate.feedback(true, t);
  ASSERT_TRUE(gate.rearm(true, t));
  gate.feedback(true, t + 90ms);
  ASSERT_TRUE(gate.allowed(t + 149ms));
  EXPECT_FALSE(gate.accept_command(t + 150ms));
  EXPECT_FALSE(gate.allowed(t + 151ms));
}

TEST(SingleWheelInterlock, DisabledOrFaultedFeedbackStopsAndCannotAutoRearm)
{
  SingleWheelInterlock gate(true);
  gate.feedback(true, t);
  ASSERT_TRUE(gate.rearm(true, t));
  gate.feedback(false, t + 1ms);
  EXPECT_FALSE(gate.allowed(t + 1ms));
  gate.feedback(true, t + 2ms);
  EXPECT_FALSE(gate.allowed(t + 2ms));
}
}  // namespace
