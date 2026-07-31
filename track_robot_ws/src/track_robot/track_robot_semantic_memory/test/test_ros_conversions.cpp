#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "track_robot_interfaces/msg/semantic_lidar_tracklet_array.hpp"
#include "track_robot_interfaces/msg/association_debug.hpp"
#include "track_robot_interfaces/msg/semantic_localization_state.hpp"
#include "track_robot_interfaces/msg/semantic_memory_event.hpp"
#include "track_robot_interfaces/msg/semantic_observation.hpp"
#include "track_robot_interfaces/msg/semantic_object.hpp"
#include "track_robot_semantic_memory/ros_conversions.hpp"

namespace semantic_memory = track_robot_semantic_memory;
namespace interfaces = track_robot_interfaces::msg;

namespace
{

interfaces::SemanticLocalizationState local_state()
{
  interfaces::SemanticLocalizationState state;
  state.header.frame_id = "odom";
  state.memory_mode = interfaces::SemanticLocalizationState::MEMORY_LOCAL_SESSION;
  state.localization_epoch_id = 7U;
  state.canonical_frame_id = "odom";
  state.local_frame_id = "odom";
  state.world_frame_id = "map";
  state.base_frame_id = "base_link";
  state.local_healthy = true;
  return state;
}

interfaces::LidarTracklet tracklet()
{
  interfaces::LidarTracklet value;
  value.tracklet_id = 4;
  value.position.x = 1.0;
  value.position.y = 2.0;
  value.position.z = 3.0;
  value.velocity.x = 1.0;
  value.size.x = 2.0;
  value.size.y = 4.0;
  value.size.z = 6.0;
  value.confidence = 0.8F;
  value.observation_quality = 0.75F;
  value.position_covariance_xy = {1.0F, 0.0F, 0.0F, 4.0F};
  value.last_measurement_stamp.sec = 12;
  value.last_measurement_stamp.nanosec = 34U;
  value.active = true;
  return value;
}

geometry_msgs::msg::TransformStamped identity_transform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "lidar";
  transform.transform.rotation.w = 1.0;
  return transform;
}

semantic_memory::MemoryObject memory_object()
{
  semantic_memory::MemoryObject object;
  object.key = {101U, 9U};
  object.lidar_key = {5U, 4};
  object.position = {1.0, 2.0, 3.0};
  object.velocity = {0.1, 0.2, 0.3};
  object.extent = {2.0, 1.0, 0.5};
  object.position_covariance = {
    0.1, 0.0, 0.0,
    0.0, 0.2, 0.0,
    0.0, 0.0, 0.3};
  object.lifecycle = semantic_memory::LifecycleState::kConfirmed;
  object.support = semantic_memory::SupportState::kLidarOnly;
  object.visibility = semantic_memory::VisibilityState::kUnknown;
  object.motion = semantic_memory::MotionState::kStatic;
  object.first_seen_ns = 1000000001LL;
  object.last_seen_ns = 2000000002LL;
  object.state_stamp_ns = 3000000003LL;
  object.observation_count = 2U;
  object.confidence = 0.8;
  object.short_history.push_back(
    {1000000001LL, {1.0, 2.0, 3.0}, semantic_memory::SupportState::kLidarOnly});
  return object;
}

}  // namespace

TEST(RosConversions, DerivesOnlyHealthyConsistentLocalizationDomains)
{
  const auto domain = semantic_memory::domain_from_localization_state(local_state());
  EXPECT_EQ(domain.mode(), semantic_memory::MemoryMode::kLocalSession);
  EXPECT_EQ(domain.localization_epoch_id(), 7U);
  EXPECT_EQ(domain.canonical_frame_id(), "odom");

  auto unhealthy = local_state();
  unhealthy.local_healthy = false;
  EXPECT_THROW(
    (void)semantic_memory::domain_from_localization_state(unhealthy),
    std::invalid_argument);

  auto inconsistent = local_state();
  inconsistent.canonical_frame_id = "map";
  EXPECT_THROW(
    (void)semantic_memory::domain_from_localization_state(inconsistent),
    std::invalid_argument);

  auto unsupported = local_state();
  unsupported.memory_mode = 99U;
  EXPECT_THROW(
    (void)semantic_memory::domain_from_localization_state(unsupported),
    std::invalid_argument);
}

TEST(RosConversions, ConvertsTrackletAtSourceStampWithRigidTransform)
{
  auto transform = identity_transform();
  transform.transform.translation.x = 10.0;
  transform.transform.translation.y = 20.0;
  transform.transform.translation.z = 30.0;
  const double half_sqrt_two = std::sqrt(0.5);
  transform.transform.rotation.z = half_sqrt_two;
  transform.transform.rotation.w = half_sqrt_two;

  const auto observation = semantic_memory::lidar_observation_from_tracklet(
    tracklet(), 5U, transform);

  EXPECT_EQ(observation.source_key.producer_epoch_id, 5U);
  EXPECT_EQ(observation.source_key.local_object_id, 4);
  EXPECT_EQ(observation.source_stamp_ns, 12000000034LL);
  EXPECT_NEAR(observation.position[0], 8.0, 1e-9);
  EXPECT_NEAR(observation.position[1], 21.0, 1e-9);
  EXPECT_NEAR(observation.position[2], 33.0, 1e-9);
  EXPECT_NEAR(observation.velocity[0], 0.0, 1e-9);
  EXPECT_NEAR(observation.velocity[1], 1.0, 1e-9);
  EXPECT_NEAR(observation.extent[0], 4.0, 1e-9);
  EXPECT_NEAR(observation.extent[1], 2.0, 1e-9);
  EXPECT_NEAR(observation.extent[2], 6.0, 1e-9);
  EXPECT_NEAR(observation.position_covariance[0], 4.0, 1e-9);
  EXPECT_NEAR(observation.position_covariance[4], 1.0, 1e-9);
  EXPECT_NEAR(observation.position_covariance[8], 4.0, 1e-9);
  EXPECT_NEAR(observation.confidence, 0.75, 1e-9);
}

TEST(RosConversions, RejectsMalformedTrackletTimeAndTransform)
{
  auto invalid_time = tracklet();
  invalid_time.last_measurement_stamp.sec = -1;
  EXPECT_THROW(
    (void)semantic_memory::lidar_observation_from_tracklet(
      invalid_time, 5U, identity_transform()),
    std::invalid_argument);

  auto invalid_transform = identity_transform();
  invalid_transform.transform.rotation.w = 0.0;
  EXPECT_THROW(
    (void)semantic_memory::lidar_observation_from_tracklet(
      tracklet(), 5U, invalid_transform),
    std::invalid_argument);
}

TEST(RosConversions, MapsMemoryObjectAndBoundedHistoryWithoutLosingIdentity)
{
  const semantic_memory::MemoryDomainKey domain{
    semantic_memory::MemoryMode::kLocalSession, 7U, "odom"};
  const auto output = semantic_memory::semantic_object_from_memory(
    memory_object(), domain);

  EXPECT_EQ(output.header.frame_id, "odom");
  EXPECT_EQ(output.header.stamp.sec, 3);
  EXPECT_EQ(output.header.stamp.nanosec, 3U);
  EXPECT_EQ(output.memory_epoch_id, 101U);
  EXPECT_EQ(output.global_object_id, 9U);
  EXPECT_TRUE(output.lidar_tracklet_id_valid);
  EXPECT_EQ(output.lidar_source_epoch_id, 5U);
  EXPECT_EQ(output.lidar_tracklet_id, 4);
  EXPECT_EQ(output.memory_mode, interfaces::SemanticObject::MEMORY_LOCAL_SESSION);
  EXPECT_EQ(output.localization_epoch_id, 7U);
  EXPECT_EQ(output.lifecycle_state, interfaces::SemanticObject::LIFECYCLE_CONFIRMED);
  EXPECT_EQ(output.support_state, interfaces::SemanticObject::SUPPORT_LIDAR_ONLY);
  EXPECT_EQ(output.motion_class, interfaces::SemanticObject::MOTION_STATIC);
  EXPECT_EQ(output.lidar_observation_count, 2U);
  ASSERT_EQ(output.short_history.size(), 1U);
  EXPECT_EQ(output.short_history[0].stamp.sec, 1);
  EXPECT_EQ(output.short_history[0].support_state,
    interfaces::SemanticObject::SUPPORT_LIDAR_ONLY);
}

TEST(RosConversions, MapsCameraOnlyObjectWithoutFabricatingLidarOrPosition)
{
  const semantic_memory::MemoryDomainKey domain{
    semantic_memory::MemoryMode::kLocalSession, 7U, "odom"};
  auto object = memory_object();
  object.lidar_key.reset();
  object.support = semantic_memory::SupportState::kCameraOnly;
  object.attached_visual_key = semantic_memory::VisualAssociationKey{
    semantic_memory::VisualAssociationKind::kCameraTrack, 20U, 7U};
  object.last_camera_seen_ns = object.state_stamp_ns;
  object.camera_observation_count = 2U;
  object.observation_count = 0U;
  object.short_history.clear();

  const auto output = semantic_memory::semantic_object_from_memory(
    object, domain);

  EXPECT_FALSE(output.lidar_tracklet_id_valid);
  EXPECT_TRUE(output.camera_track_id_valid);
  EXPECT_EQ(output.camera_source_epoch_id, 20U);
  EXPECT_EQ(output.camera_track_id, 7);
  EXPECT_FALSE(output.position_valid);
  EXPECT_FALSE(output.velocity_valid);
  EXPECT_FALSE(output.extent_valid);
  EXPECT_EQ(
    output.support_state,
    interfaces::SemanticObject::SUPPORT_CAMERA_ONLY);
  EXPECT_EQ(output.lidar_observation_count, 0U);
  EXPECT_EQ(output.camera_observation_count, 2U);
}

TEST(RosConversions, BuildsReliableSnapshotShapeAndMapsCoreEvents)
{
  const semantic_memory::MemoryDomainKey domain{
    semantic_memory::MemoryMode::kLocalSession, 7U, "odom"};
  semantic_memory::MemoryUpdateResult result;
  result.memory_epoch_id = 101U;
  result.active_objects.push_back(memory_object());
  std_msgs::msg::Header header;
  header.frame_id = "odom";
  header.stamp.sec = 4;

  const auto snapshot = semantic_memory::semantic_object_array_from_result(
    result, domain, header, 12U);
  EXPECT_EQ(snapshot.header.frame_id, "odom");
  EXPECT_EQ(snapshot.memory_epoch_id, 101U);
  EXPECT_EQ(snapshot.snapshot_sequence, 12U);
  ASSERT_EQ(snapshot.objects.size(), 1U);
  EXPECT_EQ(snapshot.objects[0].global_object_id, 9U);

  semantic_memory::MemoryEvent event;
  event.type = semantic_memory::MemoryEventType::kArchived;
  event.object_key = {101U, 9U};
  event.previous_lifecycle = semantic_memory::LifecycleState::kLost;
  event.current_lifecycle = semantic_memory::LifecycleState::kArchived;
  const auto ros_event = semantic_memory::semantic_event_from_memory(
    event, header, 13U, 101U);
  EXPECT_EQ(ros_event.sequence, 13U);
  EXPECT_EQ(ros_event.event_type, interfaces::SemanticMemoryEvent::EVENT_OBJECT_ARCHIVED);
  EXPECT_EQ(ros_event.memory_epoch_id, 101U);
  EXPECT_EQ(ros_event.global_object_id, 9U);
  EXPECT_EQ(ros_event.previous_lifecycle_state,
    interfaces::SemanticObject::LIFECYCLE_LOST);
  EXPECT_EQ(ros_event.current_lifecycle_state,
    interfaces::SemanticObject::LIFECYCLE_ARCHIVED);

  event.type = semantic_memory::MemoryEventType::kMemoryReset;
  event.object_key = {};
  const auto reset_event = semantic_memory::semantic_event_from_memory(
    event, header, 14U, 102U);
  EXPECT_EQ(reset_event.event_type,
    interfaces::SemanticMemoryEvent::EVENT_MEMORY_RESET);
  EXPECT_EQ(reset_event.reason, "observation_only_batch_reset");

  event.type = semantic_memory::MemoryEventType::kReidentified;
  event.object_key = {101U, 9U};
  const auto reidentified = semantic_memory::semantic_event_from_memory(
    event, header, 15U, 101U);
  EXPECT_EQ(reidentified.event_type,
    interfaces::SemanticMemoryEvent::EVENT_REIDENTIFIED);
  EXPECT_EQ(reidentified.reason, "object_reidentified");
}

TEST(RosConversions, DerivesDeterministicNonzeroEpochSeedFromInputDomains)
{
  const semantic_memory::MemoryDomainKey local{
    semantic_memory::MemoryMode::kLocalSession, 7U, "odom"};
  const semantic_memory::MemoryDomainKey world{
    semantic_memory::MemoryMode::kWorld, 7U, "map"};

  const auto first = semantic_memory::derive_memory_epoch_seed(local, 10U);
  EXPECT_NE(first, 0U);
  EXPECT_EQ(first, semantic_memory::derive_memory_epoch_seed(local, 10U));
  EXPECT_NE(first, semantic_memory::derive_memory_epoch_seed(local, 11U));
  EXPECT_NE(first, semantic_memory::derive_memory_epoch_seed(world, 10U));
}

TEST(RosConversions, RequiresLocalizationAndLidarSourceTimesToBeClose)
{
  EXPECT_TRUE(semantic_memory::source_times_within_tolerance(100, 120, 20));
  EXPECT_TRUE(semantic_memory::source_times_within_tolerance(120, 100, 20));
  EXPECT_FALSE(semantic_memory::source_times_within_tolerance(100, 121, 20));
  EXPECT_FALSE(semantic_memory::source_times_within_tolerance(-1, 100, 20));
  EXPECT_THROW(
    (void)semantic_memory::source_times_within_tolerance(100, 100, -1),
    std::invalid_argument);
}

TEST(RosConversions, PropagatesPredictionOnlyEvidenceIntoVisualSupplement)
{
  interfaces::SemanticObservation observation;
  observation.producer_epoch_id = 20U;
  observation.observation_id = 30U;
  observation.visual_candidate_id = 40U;
  observation.camera_stamp.sec = 1;
  observation.evidence_flags = interfaces::SemanticObservation::EVIDENCE_PREDICTION;
  observation.appearance_confidence = 0.9F;
  observation.appearance_descriptor.encoder_id = "openclip";
  observation.appearance_descriptor.checkpoint_id = "checkpoint-a";
  observation.appearance_descriptor.version = 1U;
  observation.appearance_descriptor.dimension = 2U;
  observation.appearance_descriptor.l2_normalized = true;
  observation.appearance_descriptor.values = {1.0F, 0.0F};
  observation.appearance_descriptor.quality = 0.9F;
  const semantic_memory::VisualAssociationKey visual_key{
    semantic_memory::VisualAssociationKind::kCameraTrack, 20U, 5U};
  const semantic_memory::LidarAssociationKey lidar_key{10U, 1};

  const auto prediction_only =
    semantic_memory::visual_supplement_from_semantic_observation(
    observation, visual_key, lidar_key, 0.9, true);
  EXPECT_TRUE(prediction_only.prediction_only);
  EXPECT_TRUE(prediction_only.appearance_evidence_valid);

  observation.evidence_flags |= interfaces::SemanticObservation::EVIDENCE_CAMERA;
  const auto physically_observed =
    semantic_memory::visual_supplement_from_semantic_observation(
    observation, visual_key, lidar_key, 0.9, true);
  EXPECT_FALSE(physically_observed.prediction_only);
}

TEST(RosConversions, ConvertsBoundedCameraObservationWithoutLidarGeometry)
{
  interfaces::SemanticObservation observation;
  observation.producer_epoch_id = 20U;
  observation.observation_id = 30U;
  observation.visual_candidate_id = 40U;
  observation.query_id = 50U;
  observation.query_version = 1U;
  observation.camera_stamp.sec = 1;
  observation.image_width = 1280U;
  observation.image_height = 720U;
  observation.roi.x_offset = 100U;
  observation.roi.y_offset = 200U;
  observation.roi.width = 80U;
  observation.roi.height = 120U;
  observation.language_relevance = 0.8F;
  observation.appearance_descriptor.encoder_id = "dinov3:vits16plus";
  observation.appearance_descriptor.checkpoint_id = "dino.pth";
  observation.appearance_descriptor.version = 1U;
  observation.appearance_descriptor.dimension = 2U;
  observation.appearance_descriptor.l2_normalized = true;
  observation.appearance_descriptor.values = {1.0F, 0.0F};
  observation.appearance_descriptor.quality = 0.9F;
  interfaces::SemanticLabelEvidence label;
  label.label = "blue bottle";
  label.confidence = 0.8F;
  label.provenance = "yolov8s-worldv2";
  label.evidence_kind = label.EVIDENCE_TASK_CONDITIONED;
  label.source_observation_id = 30U;
  observation.semantic_labels.push_back(label);
  const semantic_memory::VisualAssociationKey visual_key{
    semantic_memory::VisualAssociationKind::kCameraTrack, 20U, 7U};

  const auto output =
    semantic_memory::camera_observation_from_semantic_observation(
    observation, visual_key, true);

  EXPECT_EQ(output.visual_key, visual_key);
  EXPECT_EQ(output.camera_stamp_ns, 1000000000LL);
  EXPECT_EQ(output.query_id, 50U);
  EXPECT_DOUBLE_EQ(output.semantic_confidence, 0.8F);
  ASSERT_TRUE(output.appearance_descriptor.has_value());
  EXPECT_EQ(output.appearance_descriptor->encoder_id, "dinov3:vits16plus");
  ASSERT_EQ(output.semantic_labels.size(), 1U);
  EXPECT_EQ(output.semantic_labels[0].label, "blue bottle");
}

TEST(RosConversions, ConvertsStereoDepthGeometryIntoCameraObservation)
{
  interfaces::SemanticObservation observation;
  observation.producer_epoch_id = 20U;
  observation.observation_id = 30U;
  observation.visual_candidate_id = 40U;
  observation.query_id = 50U;
  observation.query_version = 1U;
  observation.camera_stamp.sec = 1;
  observation.image_width = 1280U;
  observation.image_height = 720U;
  observation.roi.width = 80U;
  observation.roi.height = 120U;
  observation.language_relevance = 0.8F;
  observation.position_valid = true;
  observation.position_frame_id = "base_link";
  observation.localization_epoch_id = 7U;
  observation.position.x = 2.3;
  observation.position.y = 0.2;
  observation.position.z = 0.5;
  observation.position_covariance = {
    0.04F, 0.0F, 0.0F,
    0.0F, 0.04F, 0.0F,
    0.0F, 0.0F, 0.09F};
  observation.geometry_confidence = 0.9F;
  observation.evidence_flags =
    interfaces::SemanticObservation::EVIDENCE_CAMERA |
    interfaces::SemanticObservation::EVIDENCE_STEREO_DEPTH;
  const semantic_memory::VisualAssociationKey visual_key{
    semantic_memory::VisualAssociationKind::kCameraTrack, 20U, 7U};

  const auto output =
    semantic_memory::camera_observation_from_semantic_observation(
    observation, visual_key, false);

  EXPECT_TRUE(output.position_valid);
  EXPECT_EQ(output.position_frame_id, "base_link");
  EXPECT_EQ(output.localization_epoch_id, 7U);
  EXPECT_DOUBLE_EQ(output.position[0], 2.3);
  EXPECT_DOUBLE_EQ(output.position_covariance[8], 0.09F);
  EXPECT_DOUBLE_EQ(output.geometry_confidence, 0.9F);
  EXPECT_TRUE(output.stereo_depth_evidence);
}

TEST(RosConversions, PreservesInvalidAssociationTermsAsNanInShadowDebug)
{
  semantic_memory::PairAssociationScore score;
  score.accepted_by_gates = false;
  score.total_score = 0.75;
  score.terms.push_back(semantic_memory::invalid_association_term(
      "visual_cosine", false, "descriptor unavailable"));
  score.terms.push_back(semantic_memory::maximum_gate(
      "source_time_delta_s", 0.02, 0.1));
  std_msgs::msg::Header header;
  header.frame_id = "camera_optical";
  header.stamp.sec = 12;

  const auto output = semantic_memory::association_debug_from_score(
    score, header, 101U, 7U, 11U, 9U, 12,
    interfaces::AssociationDebug::DECISION_REJECTED_GATE,
    0.0, "shadow_gate_rejected");

  EXPECT_EQ(output.memory_epoch_id, 101U);
  EXPECT_EQ(output.observation_producer_epoch_id, 7U);
  EXPECT_EQ(output.visual_candidate_id, 11U);
  EXPECT_EQ(output.lidar_source_epoch_id, 9U);
  EXPECT_EQ(output.lidar_tracklet_id, 12);
  ASSERT_EQ(output.terms.size(), 2U);
  EXPECT_FALSE(output.terms[0].valid);
  EXPECT_TRUE(std::isnan(output.terms[0].raw_value));
  EXPECT_TRUE(std::isnan(output.terms[0].contribution));
  EXPECT_TRUE(output.terms[1].gate_passed);
  EXPECT_EQ(output.decision, interfaces::AssociationDebug::DECISION_REJECTED_GATE);
  EXPECT_GE(output.assignment_cost, 0.0F);
  EXPECT_EQ(output.reason, "shadow_gate_rejected");
}

TEST(RosConversions, PublishesAcceptedVisualAttachmentMetadata)
{
  auto object = memory_object();
  object.attached_visual_key = semantic_memory::VisualAssociationKey{
    semantic_memory::VisualAssociationKind::kCameraTrack, 22U, 7U};
  object.visual_observation_producer_epoch_id = 22U;
  object.visual_candidate_id = 88U;
  object.last_camera_seen_ns = 9;
  object.camera_observation_count = 3U;
  object.appearance_update_count = 2U;
  object.appearance_summary_id = "appearance-v1-0123456789abcdef";
  object.appearance_prototype_count = 2U;
  object.appearance_encoder_id = "openclip";
  object.appearance_checkpoint_id = "checkpoint-a";
  object.appearance_descriptor_version = 1U;
  object.reidentification_state = semantic_memory::ReidentificationState::kPending;
  object.semantic_labels.push_back({"person", 0.8, "detector", 2U, 44U});

  const auto output = semantic_memory::semantic_object_from_memory(
    object, semantic_memory::MemoryDomainKey{
      semantic_memory::MemoryMode::kLocalSession, 7U, "odom"});

  EXPECT_TRUE(output.camera_track_id_valid);
  EXPECT_EQ(output.camera_source_epoch_id, 22U);
  EXPECT_EQ(output.camera_track_id, 7);
  EXPECT_TRUE(output.visual_candidate_id_valid);
  EXPECT_EQ(output.visual_producer_epoch_id, 22U);
  EXPECT_EQ(output.visual_candidate_id, 88U);
  EXPECT_EQ(output.last_camera_seen.sec, 0);
  EXPECT_EQ(output.last_camera_seen.nanosec, 9U);
  EXPECT_EQ(output.camera_observation_count, 3U);
  EXPECT_EQ(output.appearance_summary_id, "appearance-v1-0123456789abcdef");
  EXPECT_EQ(output.appearance_prototype_count, 2U);
  EXPECT_EQ(output.appearance_encoder_id, "openclip");
  EXPECT_EQ(output.appearance_checkpoint_id, "checkpoint-a");
  EXPECT_EQ(output.appearance_descriptor_version, 1U);
  EXPECT_EQ(output.reidentification_state, output.REID_PENDING);
  ASSERT_EQ(output.semantic_labels.size(), 1U);
  EXPECT_EQ(output.semantic_labels[0].label, "person");
}

TEST(RosConversions, RuntimeViewOverlaysOnlyTaskAndInspectionFields)
{
  auto permanent = memory_object();
  permanent.appearance_summary_id = "appearance-v1-0123456789abcdef";
  const semantic_memory::RuntimeObjectView view{
    permanent,
    semantic_memory::InspectionState::kComplete,
    semantic_memory::SemanticTaskKey{41U, 3U},
    0.875};

  const auto output = semantic_memory::semantic_object_from_runtime_view(
    view, semantic_memory::MemoryDomainKey{
      semantic_memory::MemoryMode::kLocalSession, 7U, "odom"});

  EXPECT_EQ(output.memory_epoch_id, permanent.key.memory_epoch_id);
  EXPECT_EQ(output.global_object_id, permanent.key.global_object_id);
  EXPECT_EQ(output.appearance_summary_id, permanent.appearance_summary_id);
  EXPECT_EQ(output.inspection_state, output.INSPECTION_COMPLETE);
  EXPECT_EQ(output.active_query_id, 41U);
  EXPECT_EQ(output.active_query_version, 3U);
  EXPECT_FLOAT_EQ(output.task_relevance, 0.875F);
}

TEST(RosConversions, MapsInspectionChangedEvent)
{
  semantic_memory::MemoryEvent event;
  event.type = semantic_memory::MemoryEventType::kInspectionChanged;
  event.object_key = {101U, 9U};
  std_msgs::msg::Header header;
  header.frame_id = "odom";

  const auto output = semantic_memory::semantic_event_from_memory(
    event, header, 17U, 101U);

  EXPECT_EQ(
    output.event_type,
    interfaces::SemanticMemoryEvent::EVENT_INSPECTION_CHANGED);
  EXPECT_EQ(output.reason, "inspection_state_changed");
}
