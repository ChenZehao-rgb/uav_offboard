#include <gtest/gtest.h>

#include <uav_offboard_fsm/hold_adjust_velocity_limiter.hpp>

#include <cmath>

namespace
{

using uav_offboard_fsm::hold_adjust::Velocity3;

constexpr double kTolerance = 1e-12;

void expectVelocityNear(const Velocity3 & actual, const Velocity3 & expected)
{
  EXPECT_NEAR(actual[0], expected[0], kTolerance);
  EXPECT_NEAR(actual[1], expected[1], kTolerance);
  EXPECT_NEAR(actual[2], expected[2], kTolerance);
}

void expectSafeStep(const Velocity3 & previous, const Velocity3 & current)
{
  using namespace uav_offboard_fsm::hold_adjust;

  EXPECT_LE(
    std::hypot(current[0], current[1]),
    kMaxXySpeedMps + kTolerance);
  EXPECT_LE(std::abs(current[2]), kMaxZSpeedMps + kTolerance);
  EXPECT_LE(
    std::abs(current[0] - previous[0]),
    kMaxXyDeltaMps + kTolerance);
  EXPECT_LE(
    std::abs(current[1] - previous[1]),
    kMaxXyDeltaMps + kTolerance);
  EXPECT_LE(
    std::abs(current[2] - previous[2]),
    kMaxZDeltaMps + kTolerance);
}

TEST(HoldAdjustVelocityLimiter, LeavesSafeVelocityUnchanged)
{
  const Velocity3 velocity{0.10, -0.10, 0.05};
  const auto result =
    uav_offboard_fsm::hold_adjust::limitVelocity(velocity, velocity);

  expectVelocityNear(result.speed_limited_velocity, velocity);
  expectVelocityNear(result.velocity, velocity);
  EXPECT_FALSE(result.speed_limited);
  EXPECT_FALSE(result.slew_limited);
}

TEST(HoldAdjustVelocityLimiter, ScalesHorizontalSpeedWithoutChangingDirection)
{
  const Velocity3 desired{0.3, 0.4, 0.0};
  const Velocity3 previous{0.12, 0.16, 0.0};
  const auto result =
    uav_offboard_fsm::hold_adjust::limitVelocity(desired, previous);

  expectVelocityNear(
    result.speed_limited_velocity, Velocity3{0.12, 0.16, 0.0});
  expectVelocityNear(result.velocity, Velocity3{0.12, 0.16, 0.0});
  EXPECT_NEAR(
    std::hypot(result.velocity[0], result.velocity[1]), 0.2, kTolerance);
  EXPECT_NEAR(result.velocity[0] / result.velocity[1], 0.75, kTolerance);
  EXPECT_TRUE(result.speed_limited);
  EXPECT_FALSE(result.slew_limited);
}

TEST(HoldAdjustVelocityLimiter, ClampsPositiveAndNegativeVerticalSpeed)
{
  const auto positive = uav_offboard_fsm::hold_adjust::limitVelocity(
    Velocity3{0.0, 0.0, 0.8}, Velocity3{0.0, 0.0, 0.1});
  const auto negative = uav_offboard_fsm::hold_adjust::limitVelocity(
    Velocity3{0.0, 0.0, -0.8}, Velocity3{0.0, 0.0, -0.1});

  expectVelocityNear(positive.velocity, Velocity3{0.0, 0.0, 0.1});
  expectVelocityNear(negative.velocity, Velocity3{0.0, 0.0, -0.1});
  EXPECT_TRUE(positive.speed_limited);
  EXPECT_TRUE(negative.speed_limited);
}

TEST(HoldAdjustVelocityLimiter, LimitsFirstFrameFromZero)
{
  const Velocity3 previous{0.0, 0.0, 0.0};
  const auto result = uav_offboard_fsm::hold_adjust::limitVelocity(
    Velocity3{0.2, 0.0, 0.1}, previous);

  expectVelocityNear(result.velocity, Velocity3{0.05, 0.0, 0.025});
  expectSafeStep(previous, result.velocity);
  EXPECT_TRUE(result.slew_limited);
}

TEST(HoldAdjustVelocityLimiter, ConvergesToSpeedLimitedTarget)
{
  const Velocity3 desired{0.3, 0.4, 0.2};
  const Velocity3 expected[] = {
    {0.0375, 0.05, 0.025},
    {0.075, 0.10, 0.05},
    {0.1125, 0.15, 0.075},
    {0.12, 0.16, 0.10},
  };

  Velocity3 previous{0.0, 0.0, 0.0};
  for (const auto & expected_velocity : expected) {
    const auto result =
      uav_offboard_fsm::hold_adjust::limitVelocity(desired, previous);
    expectVelocityNear(result.velocity, expected_velocity);
    expectSafeStep(previous, result.velocity);
    previous = result.velocity;
  }
}

TEST(HoldAdjustVelocityLimiter, UsesCommonHorizontalScaleForPerAxisBounds)
{
  const Velocity3 previous{0.0, 0.0, 0.0};
  const auto result = uav_offboard_fsm::hold_adjust::limitVelocity(
    Velocity3{0.15, 0.10, 0.0}, previous);

  expectVelocityNear(
    result.velocity, Velocity3{0.05, 1.0 / 30.0, 0.0});
  EXPECT_NEAR(
    result.velocity[0] / result.velocity[1], 1.5, kTolerance);
  expectSafeStep(previous, result.velocity);
}

TEST(HoldAdjustVelocityLimiter, HandlesForwardToReverseStepSafely)
{
  const Velocity3 desired{-0.3, -0.4, -0.2};
  Velocity3 previous{0.12, 0.16, 0.1};

  const auto first =
    uav_offboard_fsm::hold_adjust::limitVelocity(desired, previous);
  expectVelocityNear(first.velocity, Velocity3{0.0825, 0.11, 0.075});
  expectSafeStep(previous, first.velocity);
  previous = first.velocity;

  for (int i = 0; i < 20; ++i) {
    const auto result =
      uav_offboard_fsm::hold_adjust::limitVelocity(desired, previous);
    expectSafeStep(previous, result.velocity);
    previous = result.velocity;
  }
  expectVelocityNear(previous, Velocity3{-0.12, -0.16, -0.1});
}

TEST(HoldAdjustVelocityLimiter, DeceleratesStaleVelocityToZero)
{
  const Velocity3 zero{0.0, 0.0, 0.0};
  const Velocity3 expected[] = {
    {0.0825, 0.11, 0.075},
    {0.045, 0.06, 0.05},
    {0.0075, 0.01, 0.025},
    {0.0, 0.0, 0.0},
  };

  Velocity3 previous{0.12, 0.16, 0.1};
  for (const auto & expected_velocity : expected) {
    const auto result =
      uav_offboard_fsm::hold_adjust::limitVelocity(zero, previous);
    expectVelocityNear(result.velocity, expected_velocity);
    expectSafeStep(previous, result.velocity);
    previous = result.velocity;
  }
}

TEST(HoldAdjustVelocityLimiter, AcceptsExactPerFrameDeltaBoundary)
{
  const Velocity3 previous{0.01, -0.02, 0.03};
  const Velocity3 desired{0.06, -0.07, 0.005};
  const auto result =
    uav_offboard_fsm::hold_adjust::limitVelocity(desired, previous);

  expectVelocityNear(result.velocity, desired);
  expectSafeStep(previous, result.velocity);
}

TEST(HoldAdjustVelocityLimiter, DetectsVelocityChangesAboveEpsilon)
{
  using uav_offboard_fsm::hold_adjust::velocityChanged;

  const Velocity3 velocity{0.1, -0.1, 0.05};
  EXPECT_FALSE(velocityChanged(velocity, velocity));
  EXPECT_FALSE(
    velocityChanged(
      velocity, Velocity3{velocity[0] + 0.5e-9, velocity[1], velocity[2]}));
  EXPECT_TRUE(
    velocityChanged(
      velocity, Velocity3{velocity[0] + 2.0e-9, velocity[1], velocity[2]}));
}

}  // namespace
