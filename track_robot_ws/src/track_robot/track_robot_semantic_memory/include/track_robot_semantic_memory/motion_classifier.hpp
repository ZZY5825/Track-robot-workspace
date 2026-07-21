#pragma once

#include "track_robot_semantic_memory/types.hpp"

namespace track_robot_semantic_memory
{

class MotionClassifier
{
public:
  MotionClassifier(double static_max_speed_mps, double dynamic_min_speed_mps);

  [[nodiscard]] MotionState classify(
    MotionState current, double speed_mps) const;

private:
  double static_max_speed_mps_;
  double dynamic_min_speed_mps_;
};

}  // namespace track_robot_semantic_memory
