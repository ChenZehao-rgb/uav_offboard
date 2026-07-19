#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace uav_offboard_fsm::hold_adjust
{

using Velocity3 = std::array<double, 3>;

inline constexpr double kMaxXySpeedMps = 0.2;
inline constexpr double kMaxZSpeedMps = 0.1;
inline constexpr double kControlPeriodSeconds = 0.05;
inline constexpr double kMaxXyAccelerationMps2 = 1.0;
inline constexpr double kMaxZAccelerationMps2 = 0.5;
inline constexpr double kMaxXyDeltaMps =
  kMaxXyAccelerationMps2 * kControlPeriodSeconds;
inline constexpr double kMaxZDeltaMps =
  kMaxZAccelerationMps2 * kControlPeriodSeconds;
inline constexpr double kVelocityEpsilon = 1e-9;

struct VelocityLimitResult
{
  Velocity3 speed_limited_velocity{0.0, 0.0, 0.0};
  Velocity3 velocity{0.0, 0.0, 0.0};
  bool speed_limited{false};
  bool slew_limited{false};
};

inline bool velocityChanged(
  const Velocity3 & lhs, const Velocity3 & rhs,
  double epsilon = kVelocityEpsilon)
{
  return std::abs(lhs[0] - rhs[0]) > epsilon ||
         std::abs(lhs[1] - rhs[1]) > epsilon ||
         std::abs(lhs[2] - rhs[2]) > epsilon;
}

inline VelocityLimitResult limitVelocity(
  const Velocity3 & desired_velocity,
  const Velocity3 & previous_velocity)
{
  VelocityLimitResult result;
  result.speed_limited_velocity = desired_velocity;

  const double xy_speed = std::hypot(
    result.speed_limited_velocity[0],
    result.speed_limited_velocity[1]);
  if (xy_speed > kMaxXySpeedMps) {
    const double scale = kMaxXySpeedMps / xy_speed;
    result.speed_limited_velocity[0] *= scale;
    result.speed_limited_velocity[1] *= scale;
    result.speed_limited = true;
  }

  const double limited_z = std::clamp(
    result.speed_limited_velocity[2],
    -kMaxZSpeedMps, kMaxZSpeedMps);
  if (std::abs(limited_z - result.speed_limited_velocity[2]) >
    kVelocityEpsilon)
  {
    result.speed_limited = true;
  }
  result.speed_limited_velocity[2] = limited_z;

  const double delta_x =
    result.speed_limited_velocity[0] - previous_velocity[0];
  const double delta_y =
    result.speed_limited_velocity[1] - previous_velocity[1];
  // A common XY scale keeps the command on the line segment between two
  // velocities inside the 0.2 m/s disk, while bounding each axis step.
  double xy_step_scale = 1.0;
  if (std::abs(delta_x) > kMaxXyDeltaMps) {
    xy_step_scale = std::min(
      xy_step_scale, kMaxXyDeltaMps / std::abs(delta_x));
  }
  if (std::abs(delta_y) > kMaxXyDeltaMps) {
    xy_step_scale = std::min(
      xy_step_scale, kMaxXyDeltaMps / std::abs(delta_y));
  }

  result.velocity[0] =
    previous_velocity[0] + xy_step_scale * delta_x;
  result.velocity[1] =
    previous_velocity[1] + xy_step_scale * delta_y;
  if (xy_step_scale < 1.0) {
    result.slew_limited = true;
  }

  const double delta_z =
    result.speed_limited_velocity[2] - previous_velocity[2];
  const double limited_delta_z = std::clamp(
    delta_z, -kMaxZDeltaMps, kMaxZDeltaMps);
  result.velocity[2] = previous_velocity[2] + limited_delta_z;
  if (std::abs(limited_delta_z - delta_z) > kVelocityEpsilon) {
    result.slew_limited = true;
  }

  return result;
}

}  // namespace uav_offboard_fsm::hold_adjust
