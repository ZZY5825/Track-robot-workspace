#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/multisensor_update.hpp"

namespace semantic_memory = track_robot_semantic_memory;

namespace
{

constexpr std::int64_t kSecond = 1'000'000'000LL;

semantic_memory::MultisensorUpdateConfig config()
{
  return semantic_memory::MultisensorUpdateConfig{
    5 * kSecond, 10 * kSecond,
    1 * kSecond, 3 * kSecond,
    0.1, 0.5, 0.1, 0.2};
}

semantic_memory::MultisensorObjectState state(
  semantic_memory::MotionState motion = semantic_memory::MotionState::kStatic)
{
  semantic_memory::MultisensorObjectState value;
  value.lifecycle = semantic_memory::LifecycleState::kConfirmed;
  value.support = semantic_memory::SupportState::kCameraLidar;
  value.visibility = semantic_memory::VisibilityState::kVisible;
  value.motion = motion;
  value.position = {1.0, 2.0, 3.0};
  value.velocity = {1.0, 0.0, 0.0};
  value.position_covariance = {
    0.1, 0.0, 0.0,
    0.0, 0.1, 0.0,
    0.0, 0.0, 0.1};
  value.confidence = 0.8;
  value.state_stamp_ns = kSecond;
  value.last_supported_stamp_ns = kSecond;
  value.last_camera_stamp_ns = kSecond;
  value.last_lidar_stamp_ns = kSecond;
  return value;
}

semantic_memory::MultisensorEvidence evidence(std::int64_t stamp_ns)
{
  semantic_memory::MultisensorEvidence value;
  value.source_stamp_ns = stamp_ns;
  value.camera_visibility_known = true;
  value.in_camera_field_of_view = true;
  value.position = {4.0, 5.0, 6.0};
  value.position_covariance = {
    0.2, 0.0, 0.0,
    0.0, 0.2, 0.0,
    0.0, 0.0, 0.2};
  value.camera_confidence = 0.9;
  value.lidar_confidence = 0.85;
  return value;
}

}  // namespace

TEST(MultisensorUpdate, PolicyMatrixDefinesEveryLifecycleSupportCombination)
{
  for (const auto lifecycle : {
      semantic_memory::LifecycleState::kTentative,
      semantic_memory::LifecycleState::kConfirmed,
      semantic_memory::LifecycleState::kStale,
      semantic_memory::LifecycleState::kLost,
      semantic_memory::LifecycleState::kArchived})
  {
    for (const auto support : {
        semantic_memory::SupportState::kNone,
        semantic_memory::SupportState::kCameraLidar,
        semantic_memory::SupportState::kCameraOnly,
        semantic_memory::SupportState::kLidarOnly,
        semantic_memory::SupportState::kPredictionOnly})
    {
      const auto policy = semantic_memory::multisensor_update_permissions(
        lifecycle, support);
      if (lifecycle == semantic_memory::LifecycleState::kArchived ||
        lifecycle == semantic_memory::LifecycleState::kLost)
      {
        EXPECT_FALSE(policy.position_measurement);
        EXPECT_FALSE(policy.position_prediction);
        EXPECT_FALSE(policy.semantics);
        EXPECT_FALSE(policy.appearance);
      }
      if (lifecycle == semantic_memory::LifecycleState::kConfirmed &&
        support == semantic_memory::SupportState::kCameraOnly)
      {
        EXPECT_FALSE(policy.position_measurement);
        EXPECT_TRUE(policy.semantics);
        EXPECT_TRUE(policy.appearance);
      }
    }
  }
}

TEST(MultisensorUpdate, CameraLidarUpdatesMetricAndConfirmedVisualEvidence)
{
  semantic_memory::MultisensorUpdater updater(config());
  auto input = evidence(2 * kSecond);
  input.camera_observed = true;
  input.lidar_observed = true;
  input.association_confirmed = true;
  input.position_valid = true;
  input.semantic_evidence_valid = true;
  input.appearance_evidence_valid = true;

  const auto result = updater.update(state(), input);

  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.state.support, semantic_memory::SupportState::kCameraLidar);
  EXPECT_TRUE(result.position_updated);
  EXPECT_TRUE(result.covariance_updated);
  EXPECT_TRUE(result.semantics_updated);
  EXPECT_TRUE(result.appearance_updated);
  EXPECT_EQ(result.state.position, input.position);
  EXPECT_EQ(result.state.position_covariance, input.position_covariance);
  EXPECT_GT(result.state.confidence, 0.8);
}

TEST(MultisensorUpdate, CameraOnlyCannotInventDepthButMayUpdateConfirmedSemantics)
{
  semantic_memory::MultisensorUpdater updater(config());
  auto before = state();
  auto input = evidence(2 * kSecond);
  input.camera_observed = true;
  input.association_confirmed = true;
  input.position_valid = true;
  input.explicit_metric_depth = false;
  input.semantic_evidence_valid = true;
  input.appearance_evidence_valid = true;

  const auto result = updater.update(before, input);

  EXPECT_EQ(result.state.support, semantic_memory::SupportState::kCameraOnly);
  EXPECT_FALSE(result.position_updated);
  EXPECT_FALSE(result.covariance_updated);
  EXPECT_EQ(result.state.position, before.position);
  EXPECT_TRUE(result.semantics_updated);
  EXPECT_TRUE(result.appearance_updated);
}

TEST(MultisensorUpdate, LidarOnlyUpdatesGeometryButNotVisualMemory)
{
  semantic_memory::MultisensorUpdater updater(config());
  auto input = evidence(2 * kSecond);
  input.lidar_observed = true;
  input.position_valid = true;
  input.semantic_evidence_valid = true;
  input.appearance_evidence_valid = true;

  const auto result = updater.update(state(), input);

  EXPECT_EQ(result.state.support, semantic_memory::SupportState::kLidarOnly);
  EXPECT_TRUE(result.position_updated);
  EXPECT_FALSE(result.semantics_updated);
  EXPECT_FALSE(result.appearance_updated);
}

TEST(MultisensorUpdate, PredictionAndNoSupportDecayWithoutFabricatingEvidence)
{
  semantic_memory::MultisensorUpdater updater(config());
  auto predicted_input = evidence(2 * kSecond);
  predicted_input.prediction_available = true;

  const auto predicted = updater.update(state(), predicted_input);
  EXPECT_EQ(
    predicted.state.support, semantic_memory::SupportState::kPredictionOnly);
  EXPECT_TRUE(predicted.position_predicted);
  EXPECT_DOUBLE_EQ(predicted.state.position[0], 2.0);
  EXPECT_GT(predicted.state.position_covariance[0], 0.1);
  EXPECT_LT(predicted.state.confidence, 0.8);
  EXPECT_FALSE(predicted.semantics_updated);

  auto unsupported_input = evidence(3 * kSecond);
  const auto unsupported = updater.update(predicted.state, unsupported_input);
  EXPECT_EQ(unsupported.state.support, semantic_memory::SupportState::kNone);
  EXPECT_FALSE(unsupported.position_updated);
  EXPECT_FALSE(unsupported.position_predicted);
  EXPECT_EQ(unsupported.state.position, predicted.state.position);
  EXPECT_LT(unsupported.state.confidence, predicted.state.confidence);
}

TEST(MultisensorUpdate, DistinguishesShortSensorLossAndVisibilityState)
{
  semantic_memory::MultisensorUpdater updater(config());
  auto camera_loss = evidence(2 * kSecond);
  camera_loss.lidar_observed = true;
  camera_loss.position_valid = true;
  camera_loss.in_camera_field_of_view = false;
  const auto lidar_only = updater.update(state(), camera_loss);
  EXPECT_EQ(lidar_only.state.support, semantic_memory::SupportState::kLidarOnly);
  EXPECT_EQ(
    lidar_only.state.visibility,
    semantic_memory::VisibilityState::kOutsideFieldOfView);
  EXPECT_EQ(
    lidar_only.state.lifecycle, semantic_memory::LifecycleState::kConfirmed);

  auto lidar_loss = evidence(3 * kSecond);
  lidar_loss.camera_observed = true;
  lidar_loss.association_confirmed = true;
  lidar_loss.camera_occluded = true;
  const auto camera_only = updater.update(lidar_only.state, lidar_loss);
  EXPECT_EQ(camera_only.state.support, semantic_memory::SupportState::kCameraOnly);
  EXPECT_EQ(
    camera_only.state.visibility, semantic_memory::VisibilityState::kOccluded);
  EXPECT_EQ(
    camera_only.state.lifecycle, semantic_memory::LifecycleState::kConfirmed);
}

TEST(MultisensorUpdate, AmbiguousOrIncorrectChallengerCannotUpdateVisualState)
{
  semantic_memory::MultisensorUpdater updater(config());
  auto input = evidence(2 * kSecond);
  input.camera_observed = true;
  input.lidar_observed = true;
  input.position_valid = true;
  input.association_confirmed = false;
  input.ambiguous = true;
  input.semantic_evidence_valid = true;
  input.appearance_evidence_valid = true;

  const auto result = updater.update(state(), input);

  EXPECT_EQ(result.state.support, semantic_memory::SupportState::kLidarOnly);
  EXPECT_TRUE(result.position_updated);
  EXPECT_FALSE(result.semantics_updated);
  EXPECT_FALSE(result.appearance_updated);
}

TEST(MultisensorUpdate, StaticDynamicTimeoutsAndLostReactivationAreConservative)
{
  semantic_memory::MultisensorUpdater updater(config());
  auto no_support = evidence(3 * kSecond);
  const auto static_result = updater.update(state(), no_support);
  EXPECT_EQ(
    static_result.state.lifecycle, semantic_memory::LifecycleState::kConfirmed);

  const auto dynamic_stale = updater.update(
    state(semantic_memory::MotionState::kDynamic), no_support);
  EXPECT_EQ(
    dynamic_stale.state.lifecycle, semantic_memory::LifecycleState::kStale);

  no_support.source_stamp_ns = 5 * kSecond;
  const auto dynamic_lost = updater.update(
    state(semantic_memory::MotionState::kDynamic), no_support);
  EXPECT_EQ(
    dynamic_lost.state.lifecycle, semantic_memory::LifecycleState::kLost);

  auto reactivation = evidence(6 * kSecond);
  reactivation.camera_observed = true;
  reactivation.lidar_observed = true;
  reactivation.association_confirmed = true;
  reactivation.position_valid = true;
  const auto rejected = updater.update(dynamic_lost.state, reactivation);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.state.lifecycle, semantic_memory::LifecycleState::kLost);

  reactivation.reactivation_confirmed = true;
  const auto restored = updater.update(dynamic_lost.state, reactivation);
  EXPECT_TRUE(restored.accepted);
  EXPECT_EQ(restored.state.lifecycle, semantic_memory::LifecycleState::kConfirmed);
  EXPECT_EQ(restored.state.support, semantic_memory::SupportState::kCameraLidar);
}

TEST(MultisensorUpdate, RejectsRollbackNonfiniteConfidenceAndArchivedMutation)
{
  semantic_memory::MultisensorUpdater updater(config());
  auto rollback = evidence(0);
  EXPECT_THROW(updater.update(state(), rollback), std::invalid_argument);

  auto invalid = evidence(2 * kSecond);
  invalid.camera_confidence = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(updater.update(state(), invalid), std::invalid_argument);

  auto archived = state();
  archived.lifecycle = semantic_memory::LifecycleState::kArchived;
  auto observed = evidence(2 * kSecond);
  observed.lidar_observed = true;
  observed.position_valid = true;
  const auto result = updater.update(archived, observed);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.state.position, archived.position);
}
