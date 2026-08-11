#pragma once

namespace track_robot_semantic_memory
{

struct SemanticInputPolicy
{
  bool lidar_memory_updates_enabled{true};
  bool association_shadow_mode{true};
  bool camera_attachment_enabled{false};
  bool camera_only_memory_enabled{false};
  bool enable_test_camera_attachment{false};
  bool allow_degraded_calibration{false};
  bool static_target_profile{false};

  bool requires_lidar_subscription() const noexcept;
  bool allows_direct_lidar_memory_update() const noexcept;
  void validate_static_target_profile() const;
};

}  // namespace track_robot_semantic_memory
