#include <gtest/gtest.h>

#include <stdexcept>

#include "track_robot_semantic_memory/input_policy.hpp"

namespace semantic_memory = track_robot_semantic_memory;

TEST(InputPolicy, GenericDefaultsSubscribeAndAllowDirectLidarMemoryUpdates)
{
  const semantic_memory::SemanticInputPolicy policy;

  EXPECT_TRUE(policy.requires_lidar_subscription());
  EXPECT_TRUE(policy.allows_direct_lidar_memory_update());
}

TEST(InputPolicy, Phase4ACameraOnlyProfileRejectsAllLidarInput)
{
  semantic_memory::SemanticInputPolicy policy;
  policy.lidar_memory_updates_enabled = false;
  policy.association_shadow_mode = false;
  policy.camera_attachment_enabled = false;

  EXPECT_FALSE(policy.requires_lidar_subscription());
  EXPECT_FALSE(policy.allows_direct_lidar_memory_update());
}

TEST(InputPolicy, FusionProfilesCanBufferLidarWithoutDirectMemoryUpdates)
{
  semantic_memory::SemanticInputPolicy shadow_policy;
  shadow_policy.lidar_memory_updates_enabled = false;
  shadow_policy.association_shadow_mode = true;
  shadow_policy.camera_attachment_enabled = false;
  EXPECT_TRUE(shadow_policy.requires_lidar_subscription());
  EXPECT_FALSE(shadow_policy.allows_direct_lidar_memory_update());

  semantic_memory::SemanticInputPolicy attachment_policy;
  attachment_policy.lidar_memory_updates_enabled = false;
  attachment_policy.association_shadow_mode = false;
  attachment_policy.camera_attachment_enabled = true;
  EXPECT_TRUE(attachment_policy.requires_lidar_subscription());
  EXPECT_FALSE(attachment_policy.allows_direct_lidar_memory_update());
}

TEST(InputPolicy, NonStaticProfilesDoNotRequireCameraOnlyAuthorization)
{
  semantic_memory::SemanticInputPolicy policy;
  policy.static_target_profile = false;
  policy.camera_only_memory_enabled = false;
  policy.camera_attachment_enabled = true;

  EXPECT_NO_THROW(policy.validate_static_target_profile());
}

TEST(InputPolicy, StaticCameraOnlyProfileDoesNotRequireDegradedAttachmentFlags)
{
  semantic_memory::SemanticInputPolicy policy;
  policy.static_target_profile = true;
  policy.camera_only_memory_enabled = true;
  policy.camera_attachment_enabled = false;
  policy.enable_test_camera_attachment = false;
  policy.allow_degraded_calibration = false;

  EXPECT_NO_THROW(policy.validate_static_target_profile());
}

TEST(InputPolicy, StaticProfileRejectsNonCameraOnlyMemory)
{
  semantic_memory::SemanticInputPolicy policy;
  policy.static_target_profile = true;
  policy.camera_only_memory_enabled = false;

  EXPECT_THROW(policy.validate_static_target_profile(), std::invalid_argument);
}

TEST(InputPolicy, StaticLidarAttachmentRequiresBothDegradedTestFlags)
{
  semantic_memory::SemanticInputPolicy policy;
  policy.static_target_profile = true;
  policy.camera_only_memory_enabled = true;
  policy.camera_attachment_enabled = true;

  policy.enable_test_camera_attachment = false;
  policy.allow_degraded_calibration = false;
  EXPECT_THROW(policy.validate_static_target_profile(), std::invalid_argument);

  policy.enable_test_camera_attachment = true;
  policy.allow_degraded_calibration = false;
  EXPECT_THROW(policy.validate_static_target_profile(), std::invalid_argument);

  policy.enable_test_camera_attachment = false;
  policy.allow_degraded_calibration = true;
  EXPECT_THROW(policy.validate_static_target_profile(), std::invalid_argument);

  policy.enable_test_camera_attachment = true;
  policy.allow_degraded_calibration = true;
  EXPECT_NO_THROW(policy.validate_static_target_profile());
}
