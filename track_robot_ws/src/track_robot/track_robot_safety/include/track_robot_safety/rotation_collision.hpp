#ifndef TRACK_ROBOT_SAFETY__ROTATION_COLLISION_HPP_
#define TRACK_ROBOT_SAFETY__ROTATION_COLLISION_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace track_robot_safety
{

struct RotationCollisionResult
{
  bool collision{false};
  double collision_angle{std::numeric_limits<double>::infinity()};
  double time_to_collision{std::numeric_limits<double>::infinity()};
};

inline double rotationStopAngle(
  const double angular_speed,
  const double angular_braking_deceleration,
  const double response_latency,
  const double fixed_rotation_margin)
{
  const double speed = std::abs(angular_speed);
  if (speed < 1e-4) {
    return 0.0;
  }
  const double deceleration = std::max(std::abs(angular_braking_deceleration), 1e-3);
  return speed * speed / (2.0 * deceleration) +
         speed * std::max(response_latency, 0.0) +
         std::max(fixed_rotation_margin, 0.0);
}

template<typename PointsT>
RotationCollisionResult evaluateRotationCollision(
  const PointsT & obstacle_points,
  const double angular_speed,
  const double half_length,
  const double half_width,
  const double angular_braking_deceleration,
  const double response_latency,
  const double fixed_rotation_margin,
  const double sample_angle)
{
  RotationCollisionResult result;
  const double speed = std::abs(angular_speed);
  if (speed < 1e-4) {
    return result;
  }

  const double stop_angle = rotationStopAngle(
    angular_speed, angular_braking_deceleration, response_latency, fixed_rotation_margin);
  const double step = std::max(std::abs(sample_angle), 1e-4);
  const double direction = angular_speed >= 0.0 ? 1.0 : -1.0;

  for (double angle_magnitude = std::min(step, stop_angle);
    angle_magnitude <= stop_angle + 1e-9;
    angle_magnitude = std::min(angle_magnitude + step, stop_angle))
  {
    const double yaw = direction * angle_magnitude;
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    for (const auto & point : obstacle_points) {
      const double x = static_cast<double>(point.x);
      const double y = static_cast<double>(point.y);
      const double local_x = cosine * x + sine * y;
      const double local_y = -sine * x + cosine * y;
      if (std::abs(local_x) <= half_length && std::abs(local_y) <= half_width) {
        result.collision = true;
        result.collision_angle = yaw;
        result.time_to_collision = angle_magnitude / speed;
        return result;
      }
    }

    if (angle_magnitude >= stop_angle) {
      break;
    }
  }
  return result;
}

}  // namespace track_robot_safety

#endif  // TRACK_ROBOT_SAFETY__ROTATION_COLLISION_HPP_
