#include <limits>

#include "gtest/gtest.h"
#include "harurobo_base/wheel_kinematics.hpp"

namespace
{
using harurobo_base::WheelGeometry;
using harurobo_base::WheelKinematics;

// Synthetic fixture only. Not measured robot dimensions or roller directions.
std::array<WheelGeometry, 4> fixture()
{
  const double c = std::sqrt(0.5);
  return {{{0.2, 0.2, -c, c, 0.05, 100}, {0.2, -0.2, c, c, 0.05, 100},
    {-0.2, 0.2, -c, -c, 0.05, 100}, {-0.2, -0.2, c, -c, 0.05, 100}}};
}

TEST(WheelKinematics, TestVectorsFollowMeasuredDriveVectorFormula)
{
  WheelKinematics k(fixture());
  const double c = std::sqrt(0.5);
  const auto front = k.solve(0.1, 0, 0);
  const auto left = k.solve(0, 0.1, 0);
  const auto ccw = k.solve(0, 0, 0.5);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(front[i], (i % 2 == 0 ? -1 : 1) * 2 * c, 1e-12);
    EXPECT_NEAR(left[i], (i < 2 ? 1 : -1) * 2 * c, 1e-12);
    EXPECT_NEAR(ccw[i], 4 * c, 1e-12);
  }
}

TEST(WheelKinematics, SaturatesEveryWheelWithOneCommonScale)
{
  auto wheels = fixture();
  const auto raw = WheelKinematics(wheels).solve(0.25, -0.1, 0.5);
  wheels[0].max_rad_s = 0.2;
  const auto limited = WheelKinematics(wheels).solve(0.25, -0.1, 0.5);
  const double scale = 0.2 / std::abs(raw[0]);
  ASSERT_LT(scale, 1);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(limited[i], raw[i] * scale, 1e-12);
    EXPECT_LE(std::abs(limited[i]), wheels[i].max_rad_s + 1e-12);
  }
}

TEST(WheelKinematics, RadiusIsAppliedOnce)
{
  auto wheels = fixture();
  const auto a = WheelKinematics(wheels).solve(0.1, 0.1, 0.1);
  for (auto & w : wheels) {w.radius_m *= 2;}
  const auto b = WheelKinematics(wheels).solve(0.1, 0.1, 0.1);
  for (std::size_t i = 0; i < 4; ++i) {EXPECT_DOUBLE_EQ(a[i], 2 * b[i]);}
}

TEST(WheelKinematics, RejectsInvalidGeometryAndSingularLayouts)
{
  auto wheels = fixture();
  wheels[0].radius_m = 0;
  EXPECT_THROW(WheelKinematics{wheels}, std::invalid_argument);
  wheels = fixture();
  wheels[0].tx = 3;
  EXPECT_THROW(WheelKinematics{wheels}, std::invalid_argument);
  wheels = fixture();
  for (auto & w : wheels) {w.tx = 1; w.ty = 0;}
  EXPECT_THROW(WheelKinematics{wheels}, std::invalid_argument);
  wheels = fixture();
  for (auto & w : wheels) {w.x_m = 0; w.y_m = 0;}
  EXPECT_THROW(WheelKinematics{wheels}, std::invalid_argument);
}

TEST(WheelKinematics, ZeroCommandAndInvalidVelocity)
{
  WheelKinematics k(fixture());
  for (double v : k.solve(0, 0, 0)) {EXPECT_DOUBLE_EQ(v, 0);}
  EXPECT_THROW(k.solve(std::numeric_limits<double>::quiet_NaN(), 0, 0), std::invalid_argument);
  EXPECT_THROW(k.solve(1e308, 1e308, 1e308), std::invalid_argument);
}
}  // namespace
