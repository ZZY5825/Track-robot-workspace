#include "track_robot_semantic_memory/motion_classifier.hpp"

#include <cmath>
#include <stdexcept>

namespace track_robot_semantic_memory
{

MotionClassifier::MotionClassifier(
  double static_max_speed_mps, double dynamic_min_speed_mps)
: static_max_speed_mps_(static_max_speed_mps),
  dynamic_min_speed_mps_(dynamic_min_speed_mps)
{
  if (!std::isfinite(static_max_speed_mps) ||
    !std::isfinite(dynamic_min_speed_mps) ||
    static_max_speed_mps < 0.0 || dynamic_min_speed_mps <= static_max_speed_mps)
  {
    throw std::invalid_argument("motion thresholds must be finite and ordered");
  }
}

MotionState MotionClassifier::classify(MotionState current, double speed_mps) const
{
  if (!std::isfinite(speed_mps) || speed_mps < 0.0) {
    throw std::invalid_argument("speed must be finite and non-negative");
  }
  if (current == MotionState::kDynamic) {
    return speed_mps <= static_max_speed_mps_ ?
      MotionState::kStatic : MotionState::kDynamic;
  }
  if (current == MotionState::kStatic) {
    return speed_mps >= dynamic_min_speed_mps_ ?
      MotionState::kDynamic : MotionState::kStatic;
  }
  if (speed_mps <= static_max_speed_mps_) {
    return MotionState::kStatic;
  }
  if (speed_mps >= dynamic_min_speed_mps_) {
    return MotionState::kDynamic;
  }
  return MotionState::kUncertain;
}

}  // namespace track_robot_semantic_memory
