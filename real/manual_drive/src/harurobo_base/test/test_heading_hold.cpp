#include "gtest/gtest.h"
#include "harurobo_base/heading_hold.hpp"

namespace
{
using namespace std::chrono_literals;
using harurobo_base::HeadingHold;
constexpr HeadingHold::Clock::time_point t{};

TEST(HeadingHold, ManualRotationAlwaysOverridesCorrection)
{
  HeadingHold h;
  const auto result = h.update(true, true, -0.5, 1, 0, true, t);
  EXPECT_DOUBLE_EQ(result.wz, -0.5);
  EXPECT_FALSE(result.holding);
}

TEST(HeadingHold, CapturesOnlyAfterRotationHasSettled)
{
  HeadingHold h;
  EXPECT_FALSE(h.update(true, true, 0, 1, 0.2, true, t).holding);
  EXPECT_FALSE(h.update(true, true, 0, 1, 0, true, t + 10ms).holding);
  EXPECT_FALSE(h.update(true, true, 0, 1, 0, true, t + 209ms).holding);
  EXPECT_TRUE(h.update(true, true, 0, 1, 0, true, t + 210ms).holding);
  const auto result = h.update(true, true, 0, 1.3, 0, true, t + 220ms);
  EXPECT_DOUBLE_EQ(result.wz, -0.2);
}

TEST(HeadingHold, WrapsAcrossPiAndDoesNotRotateAtRest)
{
  HeadingHold h;
  h.update(true, true, 0, 3.13, 0, true, t);
  h.update(true, true, 0, 3.13, 0, true, t + 200ms);
  EXPECT_NEAR(h.update(true, true, 0, -3.13, 0, true, t + 210ms).wz,
    -0.023185307179586, 1e-12);
  EXPECT_DOUBLE_EQ(h.update(true, false, 0, 2, 0, true, t + 220ms).wz, 0);
  EXPECT_FALSE(h.update(true, true, 0, 2, 0, true, t + 230ms).holding);
}

TEST(HeadingHold, FaultOrDeadmanClearsTarget)
{
  HeadingHold h;
  h.update(true, true, 0, 0, 0, true, t);
  h.update(true, true, 0, 0, 0, true, t + 200ms);
  const auto fault = h.update(true, true, 0, 1, 0, false, t + 210ms);
  EXPECT_FALSE(fault.healthy);
  EXPECT_DOUBLE_EQ(fault.wz, 0);
  EXPECT_FALSE(h.update(true, true, 0, 1, 0, true, t + 220ms).holding);
  EXPECT_DOUBLE_EQ(h.update(false, true, 0.5, 1, 0, true, t + 230ms).wz, 0);
}
}  // namespace
