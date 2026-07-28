#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/memory_core.hpp"

namespace semantic_memory = track_robot_semantic_memory;

namespace
{

semantic_memory::MemoryDomainKey local_domain()
{
  return {semantic_memory::MemoryMode::kLocalSession, 7U, "odom"};
}

semantic_memory::MemoryDomainKey observation_domain()
{
  return {semantic_memory::MemoryMode::kObservationOnly, 7U, "base_link"};
}

semantic_memory::MemoryCoreConfig test_config()
{
  semantic_memory::MemoryCoreConfig config;
  config.max_objects = 8U;
  config.max_history = 4U;
  config.rollback_tolerance_ns = 5;
  config.static_lifecycle = {2U, 10, 20, 30};
  config.dynamic_lifecycle = {2U, 5, 10, 20};
  config.static_process_noise = 0.01;
  config.dynamic_process_noise = 0.10;
  return config;
}

semantic_memory::ReidentificationConfig reidentification_config()
{
  return semantic_memory::ReidentificationConfig{
    3.0, 5'000'000'000LL, 0.75, 0.70, 3U};
}

semantic_memory::LidarObservation observation(
  std::uint64_t source_epoch,
  std::int64_t tracklet_id,
  std::int64_t stamp,
  double x,
  double vx = 0.0)
{
  semantic_memory::LidarObservation value;
  value.source_key = {source_epoch, tracklet_id};
  value.source_stamp_ns = stamp;
  value.position = {x, 0.0, 0.0};
  value.velocity = {vx, 0.0, 0.0};
  value.extent = {1.0, 0.5, 0.5};
  value.position_covariance = {0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.2};
  value.confidence = 0.9;
  return value;
}

semantic_memory::VisualMemorySupplement supplement(
  std::int64_t camera_stamp = 2,
  std::uint64_t observation_id = 50U)
{
  semantic_memory::VisualMemorySupplement value;
  value.lidar_key = {10U, 1};
  value.visual_key = {
    semantic_memory::VisualAssociationKind::kUpstreamProposal, 20U, 5U};
  value.observation_producer_epoch_id = 20U;
  value.observation_id = observation_id;
  value.visual_candidate_id = observation_id + 100U;
  value.camera_stamp_ns = camera_stamp;
  value.association_confidence = 0.9;
  value.association_confirmed = true;
  value.semantic_evidence_valid = true;
  value.appearance_evidence_valid = true;
  value.appearance_descriptor = semantic_memory::AppearanceDescriptor{
    "openclip", "checkpoint-a", 1U, 2U, true, {1.0, 0.0}};
  value.appearance_quality = 0.9;
  value.semantic_labels.push_back({
      "person", 0.8, "detector", 2U, observation_id});
  return value;
}

semantic_memory::CameraObservation camera_observation(
  std::uint64_t camera_track_id,
  std::int64_t stamp,
  std::uint64_t observation_id)
{
  semantic_memory::CameraObservation value;
  value.visual_key = {
    semantic_memory::VisualAssociationKind::kCameraTrack,
    20U,
    camera_track_id};
  value.observation_producer_epoch_id = 20U;
  value.observation_id = observation_id;
  value.visual_candidate_id = observation_id + 100U;
  value.query_id = 30U;
  value.query_version = 1U;
  value.camera_stamp_ns = stamp;
  value.semantic_confidence = 0.8;
  value.image_width = 1280U;
  value.image_height = 720U;
  value.roi_x = 100U;
  value.roi_y = 200U;
  value.roi_width = 80U;
  value.roi_height = 120U;
  value.semantic_labels.push_back({
      "blue bottle", 0.8, "yolov8s-worldv2", 1U, observation_id});
  return value;
}

bool has_event(
  const semantic_memory::MemoryUpdateResult & result,
  semantic_memory::MemoryEventType type)
{
  return std::any_of(result.events.begin(), result.events.end(),
    [type](const auto & event) {return event.type == type;});
}

}  // namespace

TEST(MemoryCore, EmptyBatchInitializesDomainWithoutFabricatingObjects)
{
  semantic_memory::MemoryCore core(test_config(), 100U);

  const auto result = core.update(local_domain(), 0, {});

  EXPECT_TRUE(result.objects.empty());
  EXPECT_TRUE(has_event(result, semantic_memory::MemoryEventType::kDomainChanged));
  EXPECT_EQ(result.memory_epoch_id, 100U);
}

TEST(MemoryCore, CameraTrackCreatesAndPreservesCameraOnlyIdentity)
{
  semantic_memory::MemoryCore core(test_config(), 100U);

  const auto first = core.update_camera(
    local_domain(), camera_observation(7U, 1, 50U));
  const auto repeated = core.update_camera(
    local_domain(), camera_observation(7U, 2, 51U));
  const auto distinct = core.update_camera(
    local_domain(), camera_observation(8U, 2, 52U));

  ASSERT_EQ(first.objects.size(), 1U);
  EXPECT_FALSE(first.objects[0].lidar_key.has_value());
  EXPECT_EQ(first.objects[0].support, semantic_memory::SupportState::kCameraOnly);
  ASSERT_EQ(repeated.objects.size(), 1U);
  EXPECT_EQ(
    repeated.objects[0].key.global_object_id,
    first.objects[0].key.global_object_id);
  EXPECT_EQ(repeated.objects[0].camera_observation_count, 2U);
  ASSERT_EQ(distinct.objects.size(), 2U);
  EXPECT_NE(
    distinct.objects[0].key.global_object_id,
    distinct.objects[1].key.global_object_id);
}

TEST(MemoryCore, CameraRollbackAdvancesEpochAndCameraOnlyLifecycleExpires)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  const auto first = core.update_camera(
    local_domain(), camera_observation(7U, 100, 50U));
  const auto rollback = core.update_camera(
    local_domain(), camera_observation(7U, 90, 51U));

  EXPECT_EQ(rollback.memory_epoch_id, first.memory_epoch_id + 1U);
  ASSERT_EQ(rollback.objects.size(), 1U);
  EXPECT_TRUE(has_event(
    rollback, semantic_memory::MemoryEventType::kDomainChanged));

  (void)core.update_camera(
    local_domain(), camera_observation(7U, 91, 52U));
  const auto stale = core.update_camera(
    local_domain(), camera_observation(8U, 102, 53U));
  const auto lost = core.update_camera(
    local_domain(), camera_observation(8U, 113, 54U));
  const auto first_track = [](const auto & object) {
      return object.attached_visual_key.has_value() &&
             object.attached_visual_key->local_id == 7U;
    };
  const auto stale_object = std::find_if(
    stale.objects.begin(), stale.objects.end(), first_track);
  ASSERT_NE(stale_object, stale.objects.end());
  EXPECT_EQ(stale_object->lifecycle, semantic_memory::LifecycleState::kStale);
  const auto lost_object = std::find_if(
    lost.objects.begin(), lost.objects.end(), first_track);
  ASSERT_NE(lost_object, lost.objects.end());
  EXPECT_EQ(lost_object->lifecycle, semantic_memory::LifecycleState::kLost);
}

TEST(MemoryCore, LidarGeometryMergesIntoCameraOwnedIdentity)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  const auto camera = core.update_camera(
    local_domain(), camera_observation(7U, 1, 50U));
  const auto camera_key = camera.objects[0].key;
  const auto lidar = core.update(
    local_domain(), 2, {observation(10U, 1, 2, 3.0)});
  ASSERT_EQ(lidar.objects.size(), 2U);

  const auto attached = core.attach_lidar_geometry(
    local_domain(),
    {semantic_memory::VisualAssociationKind::kCameraTrack, 20U, 7U},
    {10U, 1},
    0.85);

  ASSERT_TRUE(attached.accepted);
  ASSERT_EQ(attached.snapshot.objects.size(), 1U);
  const auto & object = attached.snapshot.objects[0];
  EXPECT_EQ(object.key, camera_key);
  ASSERT_TRUE(object.lidar_key.has_value());
  EXPECT_EQ(object.lidar_key->local_object_id, 1);
  EXPECT_DOUBLE_EQ(object.position[0], 3.0);
  EXPECT_EQ(object.support, semantic_memory::SupportState::kCameraLidar);
  EXPECT_DOUBLE_EQ(object.association_confidence, 0.85);
}

TEST(MemoryCore, LidarLossFallsBackToCurrentCameraIdentity)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  (void)core.update_camera(
    local_domain(), camera_observation(7U, 1, 50U));
  (void)core.update(
    local_domain(), 2, {observation(10U, 1, 2, 3.0)});
  const auto attached = core.attach_lidar_geometry(
    local_domain(),
    {semantic_memory::VisualAssociationKind::kCameraTrack, 20U, 7U},
    {10U, 1},
    0.85);
  ASSERT_TRUE(attached.accepted);
  (void)core.update_camera(
    local_domain(), camera_observation(7U, 24, 51U));

  const auto lidar_lost = core.update(local_domain(), 25, {});

  ASSERT_EQ(lidar_lost.objects.size(), 1U);
  EXPECT_FALSE(lidar_lost.objects[0].lidar_key.has_value());
  EXPECT_EQ(
    lidar_lost.objects[0].support,
    semantic_memory::SupportState::kCameraOnly);
  EXPECT_NE(
    lidar_lost.objects[0].lifecycle,
    semantic_memory::LifecycleState::kLost);
}

TEST(MemoryCore, ObservationOnlyModeNeverPersistsBaseFrameObjectsAcrossBatches)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  const auto first = core.update(
    observation_domain(), 1, {observation(10U, 1, 1, 1.0)});
  const auto second = core.update(
    observation_domain(), 2, {observation(10U, 1, 2, 2.0)});

  ASSERT_EQ(first.objects.size(), 1U);
  ASSERT_EQ(second.objects.size(), 1U);
  EXPECT_EQ(first.memory_epoch_id, 100U);
  EXPECT_EQ(second.memory_epoch_id, 101U);
  EXPECT_NE(first.objects[0].key, second.objects[0].key);
  EXPECT_EQ(second.objects[0].observation_count, 1U);
  ASSERT_EQ(second.objects[0].short_history.size(), 1U);
  EXPECT_DOUBLE_EQ(second.objects[0].position[0], 2.0);
  EXPECT_TRUE(has_event(second, semantic_memory::MemoryEventType::kMemoryReset));
}

TEST(MemoryCore, MultipleTrackletsGetDistinctStableIdsAndRepeatedConfirmation)
{
  semantic_memory::MemoryCore core(test_config(), 100U);

  auto first = core.update(local_domain(), 1, {
    observation(10U, 2, 1, 2.0), observation(10U, 1, 1, 1.0)});
  ASSERT_EQ(first.objects.size(), 2U);
  EXPECT_EQ(first.objects[0].key.global_object_id, 1U);
  ASSERT_TRUE(first.objects[0].lidar_key.has_value());
  EXPECT_EQ(first.objects[0].lidar_key->local_object_id, 1);
  EXPECT_EQ(first.objects[1].key.global_object_id, 2U);
  ASSERT_TRUE(first.objects[1].lidar_key.has_value());
  EXPECT_EQ(first.objects[1].lidar_key->local_object_id, 2);
  EXPECT_EQ(first.objects[0].lifecycle, semantic_memory::LifecycleState::kTentative);

  auto second = core.update(local_domain(), 2, {
    observation(10U, 2, 2, 2.1), observation(10U, 1, 2, 1.1)});
  ASSERT_EQ(second.objects.size(), 2U);
  EXPECT_EQ(second.objects[0].key.global_object_id, 1U);
  EXPECT_EQ(second.objects[1].key.global_object_id, 2U);
  EXPECT_EQ(second.objects[0].lifecycle, semantic_memory::LifecycleState::kConfirmed);
  EXPECT_TRUE(has_event(second, semantic_memory::MemoryEventType::kConfirmed));
}

TEST(MemoryCore, SourceEpochChangeCreatesNewIdentityWithoutReusingId)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 1, {observation(10U, 1, 1, 1.0)});

  const auto changed = core.update(
    local_domain(), 2, {observation(11U, 1, 2, 1.0)});

  ASSERT_EQ(changed.objects.size(), 2U);
  EXPECT_EQ(changed.objects[0].key.global_object_id, 1U);
  EXPECT_EQ(changed.objects[1].key.global_object_id, 2U);
  EXPECT_NE(changed.objects[0].lidar_key, changed.objects[1].lidar_key);
}

TEST(MemoryCore, RejectsEveryAmbiguousDuplicateSourceKeyInOneBatch)
{
  semantic_memory::MemoryCore core(test_config(), 100U);

  const auto result = core.update(local_domain(), 1, {
    observation(10U, 1, 1, 1.0), observation(10U, 1, 1, 9.0)});

  EXPECT_TRUE(result.objects.empty());
  EXPECT_EQ(result.rejected_observations, 2U);
}

TEST(MemoryCore, StaticVelocityStaysZeroAndDynamicObjectsPredictForward)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 1, {
    observation(10U, 1, 1, 1.0, 0.0),
    observation(10U, 2, 1, 2.0, 1.0)});

  const auto predicted = core.update(local_domain(), 4, {});

  ASSERT_EQ(predicted.objects.size(), 2U);
  EXPECT_EQ(predicted.objects[0].motion, semantic_memory::MotionState::kStatic);
  EXPECT_DOUBLE_EQ(predicted.objects[0].velocity[0], 0.0);
  EXPECT_EQ(predicted.objects[1].motion, semantic_memory::MotionState::kDynamic);
  EXPECT_GT(predicted.objects[1].position[0], 2.0);
  EXPECT_EQ(predicted.objects[1].support, semantic_memory::SupportState::kPredictionOnly);
}

TEST(MemoryCore, StaticAndDynamicObjectsUseSeparateRetentionProfiles)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 1, {
    observation(10U, 1, 1, 1.0, 0.0),
    observation(10U, 2, 1, 2.0, 1.0)});
  core.update(local_domain(), 2, {
    observation(10U, 1, 2, 1.0, 0.0),
    observation(10U, 2, 2, 2.0, 1.0)});

  const auto aged = core.update(local_domain(), 8, {});

  ASSERT_EQ(aged.objects.size(), 2U);
  EXPECT_EQ(aged.objects[0].lifecycle, semantic_memory::LifecycleState::kConfirmed);
  EXPECT_EQ(aged.objects[1].lifecycle, semantic_memory::LifecycleState::kStale);
  EXPECT_GT(aged.objects[1].position_covariance[0],
    aged.objects[0].position_covariance[0]);
}

TEST(MemoryCore, ShortHistoryIsBoundedAndKeepsNewestSamples)
{
  auto config = test_config();
  config.max_history = 2U;
  semantic_memory::MemoryCore core(config, 100U);
  core.update(local_domain(), 1, {observation(10U, 1, 1, 1.0)});
  core.update(local_domain(), 2, {observation(10U, 1, 2, 2.0)});

  const auto updated = core.update(
    local_domain(), 3, {observation(10U, 1, 3, 3.0)});

  ASSERT_EQ(updated.objects[0].short_history.size(), 2U);
  EXPECT_EQ(updated.objects[0].short_history[0].source_stamp_ns, 2);
  EXPECT_EQ(updated.objects[0].short_history[1].source_stamp_ns, 3);
}

TEST(MemoryCore, SmallOutOfOrderBatchIsRejectedWithoutRewindingState)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 100, {observation(10U, 1, 100, 1.0)});

  const auto out_of_order = core.update(
    local_domain(), 98, {observation(10U, 1, 98, 9.0)});

  ASSERT_EQ(out_of_order.objects.size(), 1U);
  EXPECT_DOUBLE_EQ(out_of_order.objects[0].position[0], 1.0);
  EXPECT_EQ(out_of_order.rejected_observations, 1U);
}

TEST(MemoryCore, MissingSupportTransitionsAndArchivedTrackReappearsWithNewId)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 1, {observation(10U, 1, 1, 1.0)});
  core.update(local_domain(), 2, {observation(10U, 1, 2, 1.0)});

  EXPECT_EQ(core.update(local_domain(), 13, {}).objects[0].lifecycle,
    semantic_memory::LifecycleState::kStale);
  auto lost = core.update(local_domain(), 23, {});
  EXPECT_EQ(lost.objects[0].lifecycle, semantic_memory::LifecycleState::kLost);
  EXPECT_TRUE(has_event(lost, semantic_memory::MemoryEventType::kLost));
  auto archived = core.update(local_domain(), 33, {});
  EXPECT_EQ(archived.objects[0].lifecycle, semantic_memory::LifecycleState::kArchived);
  EXPECT_TRUE(has_event(archived, semantic_memory::MemoryEventType::kArchived));

  const auto reappeared = core.update(
    local_domain(), 34, {observation(10U, 1, 34, 1.0)});
  ASSERT_EQ(reappeared.objects.size(), 2U);
  EXPECT_EQ(reappeared.objects.back().key.global_object_id, 2U);
  EXPECT_EQ(reappeared.objects.back().lifecycle,
    semantic_memory::LifecycleState::kTentative);
}

TEST(MemoryCore, LostTrackReactivationPreservesIdentityBeforeArchive)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 1, {observation(10U, 1, 1, 1.0)});
  core.update(local_domain(), 2, {observation(10U, 1, 2, 1.0)});
  auto lost = core.update(local_domain(), 23, {});
  ASSERT_EQ(lost.objects[0].lifecycle, semantic_memory::LifecycleState::kLost);

  const auto recovered = core.update(
    local_domain(), 24, {observation(10U, 1, 24, 1.2)});

  ASSERT_EQ(recovered.objects.size(), 1U);
  EXPECT_EQ(recovered.objects[0].key.global_object_id, 1U);
  EXPECT_EQ(recovered.objects[0].lifecycle,
    semantic_memory::LifecycleState::kConfirmed);
}

TEST(MemoryCore, CapacityNeverEvictsActiveObjectAndEvictsArchivedOldestFirst)
{
  auto config = test_config();
  config.max_objects = 1U;
  semantic_memory::MemoryCore core(config, 100U);
  core.update(local_domain(), 1, {observation(10U, 1, 1, 1.0)});
  core.update(local_domain(), 2, {observation(10U, 1, 2, 1.0)});

  auto rejected = core.update(
    local_domain(), 3, {observation(10U, 2, 3, 2.0)});
  EXPECT_EQ(rejected.objects.size(), 1U);
  EXPECT_EQ(rejected.rejected_observations, 1U);
  ASSERT_TRUE(rejected.objects[0].lidar_key.has_value());
  EXPECT_EQ(rejected.objects[0].lidar_key->local_object_id, 1);

  core.update(local_domain(), 33, {});
  auto inserted = core.update(
    local_domain(), 34, {observation(10U, 2, 34, 2.0)});
  ASSERT_EQ(inserted.objects.size(), 1U);
  ASSERT_TRUE(inserted.objects[0].lidar_key.has_value());
  EXPECT_EQ(inserted.objects[0].lidar_key->local_object_id, 2);
  EXPECT_EQ(inserted.objects[0].key.global_object_id, 2U);
}

TEST(MemoryCore, TimestampRollbackAndDomainChangeBothIsolateOldObjects)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 100, {observation(10U, 1, 100, 1.0)});

  auto rollback = core.update(
    local_domain(), 10, {observation(11U, 1, 10, 1.0)});
  ASSERT_EQ(rollback.objects.size(), 1U);
  EXPECT_EQ(rollback.memory_epoch_id, 101U);
  EXPECT_EQ(rollback.objects[0].key.memory_epoch_id, 101U);
  EXPECT_TRUE(has_event(rollback, semantic_memory::MemoryEventType::kDomainChanged));

  const semantic_memory::MemoryDomainKey world{
    semantic_memory::MemoryMode::kWorld, 8U, "map"};
  auto domain_changed = core.update(
    world, 11, {observation(11U, 1, 11, 1.0)});
  ASSERT_EQ(domain_changed.objects.size(), 1U);
  EXPECT_EQ(domain_changed.memory_epoch_id, 102U);
  EXPECT_EQ(domain_changed.objects[0].key.memory_epoch_id, 102U);
}

TEST(MemoryCore, InvalidObservationCannotPartiallyMutateDomainOrObjects)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 1, {observation(10U, 1, 1, 1.0)});
  auto invalid = observation(10U, 1, 2, 9.0);
  invalid.confidence = 2.0;
  const semantic_memory::MemoryDomainKey world{
    semantic_memory::MemoryMode::kWorld, 8U, "map"};

  EXPECT_THROW(core.update(world, 2, {invalid}), std::invalid_argument);
  const auto valid = core.update(
    local_domain(), 2, {observation(10U, 1, 2, 2.0)});

  ASSERT_EQ(valid.objects.size(), 1U);
  EXPECT_EQ(valid.memory_epoch_id, 100U);
  EXPECT_EQ(valid.objects[0].key.global_object_id, 1U);
  EXPECT_EQ(valid.objects[0].observation_count, 2U);
}

TEST(MemoryCore, ConfirmedDelayedVisualSupplementNeverRewindsMetricState)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 100, {observation(10U, 1, 100, 1.0)});
  const auto before = core.update(
    local_domain(), 101, {observation(10U, 1, 101, 2.0)});
  ASSERT_EQ(before.objects.size(), 1U);

  const auto result = core.supplement_visual(local_domain(), supplement(99));

  ASSERT_TRUE(result.accepted);
  ASSERT_EQ(result.snapshot.objects.size(), 1U);
  const auto & object = result.snapshot.objects[0];
  EXPECT_EQ(object.position, before.objects[0].position);
  EXPECT_EQ(object.velocity, before.objects[0].velocity);
  EXPECT_EQ(object.position_covariance, before.objects[0].position_covariance);
  EXPECT_EQ(object.state_stamp_ns, before.objects[0].state_stamp_ns);
  EXPECT_EQ(object.last_camera_seen_ns, 99);
  EXPECT_EQ(object.camera_observation_count, 1U);
  EXPECT_EQ(object.support, semantic_memory::SupportState::kCameraLidar);
  EXPECT_EQ(object.semantic_labels.size(), 1U);
  EXPECT_EQ(object.appearance_update_count, 1U);
  EXPECT_EQ(object.appearance_prototype_count, 1U);
  EXPECT_EQ(object.appearance_encoder_id, "openclip");
  EXPECT_EQ(object.appearance_checkpoint_id, "checkpoint-a");
  EXPECT_EQ(object.appearance_descriptor_version, 1U);
  EXPECT_EQ(object.appearance_summary_id.rfind("appearance-v1-", 0U), 0U);
  EXPECT_TRUE(result.appearance_accepted);
  EXPECT_FALSE(result.appearance_reason.empty());
  EXPECT_TRUE(has_event(
      result.snapshot, semantic_memory::MemoryEventType::kAssociationAttached));
}

TEST(MemoryCore, InvalidOptionalAppearanceDoesNotRollbackValidAttachment)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 1, {observation(10U, 1, 1, 1.0)});
  core.update(local_domain(), 2, {observation(10U, 1, 2, 1.0)});
  const auto first = core.supplement_visual(local_domain(), supplement(2));
  ASSERT_TRUE(first.accepted);
  ASSERT_TRUE(first.appearance_accepted);
  ASSERT_EQ(first.snapshot.objects.size(), 1U);
  const auto before = first.snapshot.objects.front();

  auto invalid = supplement(3, 51U);
  invalid.appearance_descriptor->values[0] =
    std::numeric_limits<double>::quiet_NaN();
  const auto rejected_appearance = core.supplement_visual(
    local_domain(), invalid);

  ASSERT_TRUE(rejected_appearance.accepted);
  EXPECT_FALSE(rejected_appearance.appearance_accepted);
  EXPECT_FALSE(rejected_appearance.appearance_reason.empty());
  ASSERT_EQ(rejected_appearance.snapshot.objects.size(), 1U);
  const auto & after = rejected_appearance.snapshot.objects.front();
  EXPECT_EQ(after.appearance_update_count, before.appearance_update_count);
  EXPECT_EQ(after.appearance_prototype_count, before.appearance_prototype_count);
  EXPECT_EQ(after.appearance_summary_id, before.appearance_summary_id);
  EXPECT_EQ(after.last_camera_seen_ns, 3);
  EXPECT_EQ(after.camera_observation_count, 2U);
}

TEST(MemoryCore, VisualSupplementIsIdempotentAndRejectsCameraRollback)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 1, {observation(10U, 1, 1, 1.0)});
  core.update(local_domain(), 2, {observation(10U, 1, 2, 1.0)});
  const auto first = core.supplement_visual(local_domain(), supplement(2));
  ASSERT_TRUE(first.accepted);

  const auto duplicate = core.supplement_visual(local_domain(), supplement(2));
  auto older_evidence = supplement(1, 51U);
  const auto older = core.supplement_visual(local_domain(), older_evidence);

  EXPECT_FALSE(duplicate.accepted);
  EXPECT_FALSE(older.accepted);
  EXPECT_EQ(duplicate.snapshot.objects[0].camera_observation_count, 1U);
  EXPECT_EQ(older.snapshot.objects[0].camera_observation_count, 1U);
  EXPECT_TRUE(duplicate.snapshot.events.empty());
  EXPECT_TRUE(older.snapshot.events.empty());
}

TEST(MemoryCore, MissingObjectAndTaskConditionedLabelsFailClosed)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 1, {observation(10U, 1, 1, 1.0)});
  core.update(local_domain(), 2, {observation(10U, 1, 2, 1.0)});
  auto input = supplement(2);
  input.semantic_labels = {{"find_me", 1.0, "query", 1U, 50U}};
  const auto accepted = core.supplement_visual(local_domain(), input);
  ASSERT_TRUE(accepted.accepted);
  EXPECT_TRUE(accepted.snapshot.objects[0].semantic_labels.empty());

  input.lidar_key.local_object_id = 999;
  input.camera_stamp_ns = 3;
  input.observation_id = 51U;
  const auto missing = core.supplement_visual(local_domain(), input);
  EXPECT_FALSE(missing.accepted);
  EXPECT_EQ(missing.reason, "attached LiDAR object is not active in this memory epoch");
}

TEST(MemoryCore, StableVisualIdentityHasExactlyOneMemoryOwner)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 1, {
    observation(10U, 1, 1, 1.0), observation(10U, 2, 1, 2.0)});
  core.update(local_domain(), 2, {
    observation(10U, 1, 2, 1.0), observation(10U, 2, 2, 2.0)});

  const auto first = core.supplement_visual(local_domain(), supplement(2));
  ASSERT_TRUE(first.accepted);

  auto moved = supplement(3, 51U);
  moved.lidar_key = {10U, 2};
  const auto second = core.supplement_visual(local_domain(), moved);

  ASSERT_TRUE(second.accepted);
  ASSERT_EQ(second.snapshot.objects.size(), 2U);
  const auto owners = std::count_if(
    second.snapshot.objects.begin(), second.snapshot.objects.end(),
    [&moved](const auto & object) {
      return object.attached_visual_key == moved.visual_key;
    });
  EXPECT_EQ(owners, 1);
  EXPECT_FALSE(second.snapshot.objects[0].attached_visual_key.has_value());
  EXPECT_EQ(second.snapshot.objects[0].support,
    semantic_memory::SupportState::kLidarOnly);
  EXPECT_DOUBLE_EQ(second.snapshot.objects[0].association_confidence, 0.0);
  ASSERT_TRUE(second.snapshot.objects[1].attached_visual_key.has_value());
  EXPECT_EQ(*second.snapshot.objects[1].attached_visual_key, moved.visual_key);
  EXPECT_TRUE(has_event(
      second.snapshot, semantic_memory::MemoryEventType::kAssociationDetached));
  EXPECT_TRUE(has_event(
      second.snapshot, semantic_memory::MemoryEventType::kAssociationAttached));
}

TEST(MemoryCore, VisualOwnerTransferDoesNotFabricateMissingLidarSupport)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 1, {
    observation(10U, 1, 1, 1.0), observation(10U, 2, 1, 2.0)});
  core.update(local_domain(), 2, {
    observation(10U, 1, 2, 1.0), observation(10U, 2, 2, 2.0)});
  ASSERT_TRUE(core.supplement_visual(
      local_domain(), supplement(2)).accepted);

  const auto predicted = core.update(
    local_domain(), 3, {observation(10U, 2, 3, 2.0)});
  ASSERT_EQ(predicted.objects.size(), 2U);
  ASSERT_EQ(predicted.objects[0].support,
    semantic_memory::SupportState::kPredictionOnly);

  auto moved = supplement(3, 51U);
  moved.lidar_key = {10U, 2};
  const auto transferred = core.supplement_visual(local_domain(), moved);

  ASSERT_TRUE(transferred.accepted);
  ASSERT_EQ(transferred.snapshot.objects.size(), 2U);
  EXPECT_EQ(transferred.snapshot.objects[0].support,
    semantic_memory::SupportState::kPredictionOnly);
  EXPECT_FALSE(
    transferred.snapshot.objects[0].attached_visual_key.has_value());
  EXPECT_EQ(transferred.snapshot.objects[1].support,
    semantic_memory::SupportState::kCameraLidar);
}

TEST(MemoryCore, BuildsCompleteReidentificationEvidenceFromOwnedBanks)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 1, {observation(10U, 1, 1, 1.0)});
  auto confirmed = core.update(
    local_domain(), 2, {observation(10U, 1, 2, 1.0)});
  const auto old_key = confirmed.objects.front().key;
  ASSERT_TRUE(core.supplement_visual(local_domain(), supplement(2)).accepted);

  core.update(local_domain(), 23, {observation(10U, 2, 23, 1.2)});
  auto replacement = core.update(
    local_domain(), 24, {observation(10U, 2, 24, 1.2)});
  ASSERT_EQ(replacement.objects.size(), 2U);
  auto moved = supplement(24, 51U);
  moved.lidar_key = {10U, 2};
  const auto applied = core.supplement_visual(local_domain(), moved);
  ASSERT_TRUE(applied.accepted);
  ASSERT_TRUE(applied.appearance_accepted);
  ASSERT_TRUE(applied.current_appearance_evidence.has_value());
  const auto candidate = std::find_if(
    applied.snapshot.objects.begin(), applied.snapshot.objects.end(),
    [&moved](const auto & object) {return object.lidar_key == moved.lidar_key;});
  ASSERT_NE(candidate, applied.snapshot.objects.end());

  const auto frame = core.make_reidentification_frame(
    local_domain(), 1U, reidentification_config(),
    {*applied.current_appearance_evidence});

  ASSERT_EQ(frame.candidates.size(), 1U);
  ASSERT_EQ(frame.lost_targets.size(), 1U);
  ASSERT_EQ(frame.pairs.size(), 1U);
  EXPECT_EQ(frame.lost_targets.front(), old_key);
  EXPECT_EQ(frame.pairs.front().lost_key, old_key);
  EXPECT_EQ(frame.pairs.front().candidate_key, frame.candidates.front());
  EXPECT_EQ(frame.pairs.front().expected_candidate_lidar_key,
    (semantic_memory::ProducerObjectKey{10U, 2}));
  EXPECT_NEAR(frame.pairs.front().appearance_similarity, 1.0, 1e-9);
  EXPECT_TRUE(frame.pairs.front().domain_compatible);

  const auto * prototypes = core.appearance_prototypes(old_key);
  ASSERT_NE(prototypes, nullptr);
  EXPECT_EQ(prototypes->size(), 1U);
  EXPECT_EQ(core.appearance_prototypes({999U, old_key.global_object_id}), nullptr);
  EXPECT_EQ(core.appearance_prototypes({100U, 999U}), nullptr);

  const auto stale_frame = core.make_reidentification_frame(
    local_domain(), 2U, reidentification_config(), {});
  EXPECT_TRUE(stale_frame.candidates.empty());
  EXPECT_EQ(stale_frame.lost_targets, frame.lost_targets);
  EXPECT_TRUE(stale_frame.pairs.empty());

  auto different = moved;
  different.camera_stamp_ns = 25;
  different.observation_id = 52U;
  different.visual_candidate_id = 152U;
  different.appearance_descriptor->values = {0.0, 1.0};
  const auto different_applied = core.supplement_visual(
    local_domain(), different);
  ASSERT_TRUE(different_applied.appearance_accepted);
  ASSERT_TRUE(different_applied.current_appearance_evidence.has_value());
  const auto current_descriptor_frame = core.make_reidentification_frame(
    local_domain(), 3U, reidentification_config(),
    {*different_applied.current_appearance_evidence});
  ASSERT_EQ(current_descriptor_frame.pairs.size(), 1U);
  EXPECT_NEAR(
    current_descriptor_frame.pairs.front().appearance_similarity, 0.5, 1e-9);

  core.update(local_domain(), 26, {});
  EXPECT_THROW(
    (void)core.make_reidentification_frame(
      local_domain(), 4U, reidentification_config(),
      {*different_applied.current_appearance_evidence}),
    std::invalid_argument);
}

TEST(MemoryCore, ConfirmedReidentificationTransfersIdentityAtomically)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 1, {observation(10U, 1, 1, 1.0)});
  const auto old_snapshot = core.update(
    local_domain(), 2, {observation(10U, 1, 2, 1.0)});
  const auto old_key = old_snapshot.objects.front().key;
  ASSERT_TRUE(core.supplement_visual(local_domain(), supplement(2)).accepted);
  core.update(local_domain(), 23, {observation(10U, 2, 23, 1.2)});
  const auto replacement_snapshot = core.update(
    local_domain(), 24, {observation(10U, 2, 24, 1.2)});
  const auto replacement_key = replacement_snapshot.objects.back().key;
  auto moved = supplement(24, 51U);
  moved.lidar_key = {10U, 2};
  ASSERT_TRUE(core.supplement_visual(local_domain(), moved).accepted);

  const auto wrong_guard = core.reidentify(
    local_domain(), old_key, replacement_key,
    semantic_memory::ProducerObjectKey{10U, 999}, moved.visual_key);
  EXPECT_FALSE(wrong_guard.accepted);
  ASSERT_EQ(wrong_guard.snapshot.objects.size(), 2U);

  const auto transferred = core.reidentify(
    local_domain(), old_key, replacement_key, moved.lidar_key, moved.visual_key);

  ASSERT_TRUE(transferred.accepted);
  EXPECT_EQ(transferred.preserved_key, old_key);
  ASSERT_EQ(transferred.snapshot.objects.size(), 1U);
  const auto & object = transferred.snapshot.objects.front();
  EXPECT_EQ(object.key, old_key);
  EXPECT_EQ(object.lidar_key, moved.lidar_key);
  EXPECT_EQ(object.position[0], replacement_snapshot.objects.back().position[0]);
  EXPECT_EQ(object.first_seen_ns, 1);
  EXPECT_EQ(object.compatible_hit_count, 4U);
  EXPECT_EQ(object.observation_count, 4U);
  EXPECT_EQ(object.camera_observation_count, 2U);
  EXPECT_EQ(object.semantic_update_count, 2U);
  EXPECT_EQ(object.appearance_update_count, 2U);
  EXPECT_EQ(object.reidentification_state,
    semantic_memory::ReidentificationState::kConfirmed);
  EXPECT_TRUE(has_event(
      transferred.snapshot, semantic_memory::MemoryEventType::kReidentified));

  const auto continued = core.update(
    local_domain(), 25, {observation(10U, 2, 25, 1.3)});
  ASSERT_EQ(continued.objects.size(), 1U);
  EXPECT_EQ(continued.objects.front().key, old_key);
  EXPECT_EQ(continued.objects.front().reidentification_state,
    semantic_memory::ReidentificationState::kNotRequired);
}

TEST(MemoryCore, RejectsAppearanceConfigurationThatCanBreakTransactions)
{
  auto no_capacity = test_config();
  no_capacity.max_feature_prototypes = 0U;
  EXPECT_THROW(
    semantic_memory::MemoryCore(no_capacity, 100U), std::invalid_argument);

  auto unsafe_tolerance = test_config();
  unsafe_tolerance.appearance_normalization_tolerance = 1.0;
  EXPECT_THROW(
    semantic_memory::MemoryCore(unsafe_tolerance, 100U),
    std::invalid_argument);
}

TEST(MemoryCore, ExplicitResetAdvancesEpochAndClearsAllIdentityState)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  core.update(local_domain(), 1, {observation(10U, 1, 1, 1.0)});
  const auto populated = core.update(
    local_domain(), 2, {observation(10U, 1, 2, 1.0)});
  const auto old_key = populated.objects.front().key;
  ASSERT_TRUE(core.supplement_visual(local_domain(), supplement(2)).accepted);
  ASSERT_NE(core.appearance_prototypes(old_key), nullptr);

  const auto reset = core.reset(local_domain());

  EXPECT_EQ(reset.memory_epoch_id, 101U);
  EXPECT_TRUE(reset.objects.empty());
  EXPECT_TRUE(reset.active_objects.empty());
  ASSERT_EQ(reset.events.size(), 1U);
  EXPECT_EQ(reset.events.front().type,
    semantic_memory::MemoryEventType::kMemoryReset);
  EXPECT_EQ(core.appearance_prototypes(old_key), nullptr);

  const auto recreated = core.update(
    local_domain(), 3, {observation(10U, 2, 3, 2.0)});
  ASSERT_EQ(recreated.objects.size(), 1U);
  EXPECT_EQ(recreated.objects.front().key.memory_epoch_id, 101U);
  EXPECT_EQ(recreated.objects.front().key.global_object_id, 1U);
}

TEST(MemoryCore, ExplicitResetRejectsWrongDomainTransactionallyAndWrapsEpoch)
{
  semantic_memory::MemoryCore core(test_config(), 100U);
  const auto before = core.update(
    local_domain(), 1, {observation(10U, 1, 1, 1.0)});
  const semantic_memory::MemoryDomainKey wrong{
    semantic_memory::MemoryMode::kWorld, 7U, "map"};
  EXPECT_THROW((void)core.reset(wrong), std::invalid_argument);
  const auto unchanged = core.update(
    local_domain(), 2, {observation(10U, 1, 2, 1.0)});
  EXPECT_EQ(unchanged.memory_epoch_id, before.memory_epoch_id);
  EXPECT_EQ(unchanged.objects.front().key, before.objects.front().key);

  semantic_memory::MemoryCore wrapping(
    test_config(), std::numeric_limits<std::uint64_t>::max());
  wrapping.update(local_domain(), 1, {});
  EXPECT_EQ(wrapping.reset(local_domain()).memory_epoch_id, 1U);
}
