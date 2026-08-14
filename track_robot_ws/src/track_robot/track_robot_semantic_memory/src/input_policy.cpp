#include "track_robot_semantic_memory/input_policy.hpp"

#include <stdexcept>

namespace track_robot_semantic_memory
{

bool SemanticInputPolicy::requires_lidar_subscription() const noexcept
{
  return lidar_memory_updates_enabled || association_shadow_mode ||
         camera_attachment_enabled;
}

bool SemanticInputPolicy::allows_direct_lidar_memory_update() const noexcept
{
  return lidar_memory_updates_enabled;
}

void SemanticInputPolicy::validate_static_target_profile() const
{
  if (!static_target_profile) {
    return;
  }
  if (!camera_only_memory_enabled) {
    throw std::invalid_argument(
            "static_target_profile requires camera_only_memory_enabled");
  }
  if (camera_attachment_enabled &&
    (!enable_test_camera_attachment || !allow_degraded_calibration))
  {
    throw std::invalid_argument(
            "static target LiDAR attachment requires the explicit degraded test profile");
  }
}

}  // namespace track_robot_semantic_memory
