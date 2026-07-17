#include "track_robot_semantic_memory/ros_conversions.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"
#include "track_robot_interfaces/msg/semantic_object_history_sample.hpp"

namespace track_robot_semantic_memory
{
namespace
{

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

std::int64_t time_to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= kNanosecondsPerSecond) {
    throw std::invalid_argument("ROS source time is outside its valid range");
  }
  return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<std::int64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time nanoseconds_to_time(std::int64_t stamp_ns)
{
  constexpr auto max_seconds = std::numeric_limits<std::int32_t>::max();
  if (stamp_ns < 0 || stamp_ns / kNanosecondsPerSecond > max_seconds) {
    throw std::invalid_argument("memory source time cannot be represented by ROS Time");
  }
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(stamp_ns / kNanosecondsPerSecond);
  stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % kNanosecondsPerSecond);
  return stamp;
}

bool finite(double value)
{
  return std::isfinite(value);
}

using Matrix3 = std::array<double, 9>;
using Vector3 = std::array<double, 3>;

Matrix3 rotation_matrix(const geometry_msgs::msg::Quaternion & quaternion)
{
  if (!finite(quaternion.x) || !finite(quaternion.y) ||
    !finite(quaternion.z) || !finite(quaternion.w))
  {
    throw std::invalid_argument("transform quaternion must be finite");
  }
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (norm < 1e-12) {
    throw std::invalid_argument("transform quaternion must have non-zero norm");
  }
  const double x = quaternion.x / norm;
  const double y = quaternion.y / norm;
  const double z = quaternion.z / norm;
  const double w = quaternion.w / norm;
  return {
    1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
    2.0 * (x * z + y * w),
    2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
    2.0 * (y * z - x * w),
    2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
    1.0 - 2.0 * (x * x + y * y)};
}

Vector3 multiply(const Matrix3 & matrix, const Vector3 & vector)
{
  Vector3 output{};
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column < 3U; ++column) {
      output[row] += matrix[row * 3U + column] * vector[column];
    }
  }
  return output;
}

Matrix3 rotate_covariance(const Matrix3 & rotation, const Matrix3 & covariance)
{
  Matrix3 intermediate{};
  Matrix3 output{};
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column < 3U; ++column) {
      for (std::size_t index = 0U; index < 3U; ++index) {
        intermediate[row * 3U + column] +=
          rotation[row * 3U + index] * covariance[index * 3U + column];
      }
    }
  }
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column < 3U; ++column) {
      for (std::size_t index = 0U; index < 3U; ++index) {
        output[row * 3U + column] +=
          intermediate[row * 3U + index] * rotation[column * 3U + index];
      }
    }
  }
  return output;
}

std::uint8_t event_type(MemoryEventType type)
{
  using RosEvent = track_robot_interfaces::msg::SemanticMemoryEvent;
  switch (type) {
    case MemoryEventType::kCreated:
      return RosEvent::EVENT_OBJECT_CREATED;
    case MemoryEventType::kConfirmed:
      return RosEvent::EVENT_OBJECT_CONFIRMED;
    case MemoryEventType::kLifecycleChanged:
    case MemoryEventType::kLost:
      return RosEvent::EVENT_LIFECYCLE_CHANGED;
    case MemoryEventType::kArchived:
    case MemoryEventType::kCapacityEvicted:
      return RosEvent::EVENT_OBJECT_ARCHIVED;
    case MemoryEventType::kDomainChanged:
      return RosEvent::EVENT_DOMAIN_CHANGED;
    case MemoryEventType::kMemoryReset:
      return RosEvent::EVENT_MEMORY_RESET;
    case MemoryEventType::kObservationRejected:
      return RosEvent::EVENT_OBSERVATION_REJECTED;
    case MemoryEventType::kAssociationAttached:
      return RosEvent::EVENT_ASSOCIATION_ATTACHED;
    case MemoryEventType::kAssociationDetached:
      return RosEvent::EVENT_ASSOCIATION_DETACHED;
    case MemoryEventType::kReidentified:
      return RosEvent::EVENT_REIDENTIFIED;
    case MemoryEventType::kInspectionChanged:
      return RosEvent::EVENT_INSPECTION_CHANGED;
  }
  throw std::invalid_argument("unsupported memory event type");
}

std::string event_reason(MemoryEventType type)
{
  switch (type) {
    case MemoryEventType::kCreated: return "object_created";
    case MemoryEventType::kConfirmed: return "object_confirmed";
    case MemoryEventType::kLifecycleChanged: return "lifecycle_changed";
    case MemoryEventType::kLost: return "object_lost";
    case MemoryEventType::kArchived: return "object_archived";
    case MemoryEventType::kDomainChanged: return "memory_domain_changed";
    case MemoryEventType::kMemoryReset: return "observation_only_batch_reset";
    case MemoryEventType::kObservationRejected: return "observation_rejected";
    case MemoryEventType::kCapacityEvicted: return "capacity_evicted";
    case MemoryEventType::kAssociationAttached: return "association_attached";
    case MemoryEventType::kAssociationDetached: return "association_detached";
    case MemoryEventType::kReidentified: return "object_reidentified";
    case MemoryEventType::kInspectionChanged: return "inspection_state_changed";
  }
  return "unknown";
}

}  // namespace

bool source_times_within_tolerance(
  std::int64_t first_ns,
  std::int64_t second_ns,
  std::int64_t tolerance_ns)
{
  if (tolerance_ns < 0) {
    throw std::invalid_argument("source-time tolerance must be non-negative");
  }
  if (first_ns < 0 || second_ns < 0) {
    return false;
  }
  const auto lower = std::min(first_ns, second_ns);
  const auto upper = std::max(first_ns, second_ns);
  return upper - lower <= tolerance_ns;
}

std::uint64_t derive_memory_epoch_seed(
  const MemoryDomainKey & domain,
  std::uint64_t lidar_source_epoch_id)
{
  if (lidar_source_epoch_id == 0U) {
    throw std::invalid_argument("LiDAR source epoch must be non-zero");
  }
  std::uint64_t hash = 14695981039346656037ULL;
  const auto append_byte = [&hash](std::uint8_t value) {
      hash ^= value;
      hash *= 1099511628211ULL;
    };
  append_byte(static_cast<std::uint8_t>(domain.mode()));
  for (std::uint64_t value : {
      domain.localization_epoch_id(), lidar_source_epoch_id})
  {
    for (std::size_t index = 0U; index < sizeof(value); ++index) {
      append_byte(static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU));
    }
  }
  append_byte(0xffU);
  for (const unsigned char character : domain.canonical_frame_id()) {
    append_byte(character);
  }
  return hash == 0U ? 1U : hash;
}

MemoryDomainKey domain_from_localization_state(
  const track_robot_interfaces::msg::SemanticLocalizationState & state)
{
  using State = track_robot_interfaces::msg::SemanticLocalizationState;
  MemoryMode mode;
  std::string expected_frame;
  switch (state.memory_mode) {
    case State::MEMORY_OBSERVATION_ONLY:
      mode = MemoryMode::kObservationOnly;
      expected_frame = state.base_frame_id;
      break;
    case State::MEMORY_LOCAL_SESSION:
      if (!state.local_healthy) {
        throw std::invalid_argument("local-session memory requires healthy local localization");
      }
      mode = MemoryMode::kLocalSession;
      expected_frame = state.local_frame_id;
      break;
    case State::MEMORY_WORLD:
      if (!state.world_healthy) {
        throw std::invalid_argument("world memory requires healthy world localization");
      }
      mode = MemoryMode::kWorld;
      expected_frame = state.world_frame_id;
      break;
    default:
      throw std::invalid_argument("localization state contains an unsupported memory mode");
  }
  if (state.localization_epoch_id == 0U) {
    throw std::invalid_argument("localization epoch must be non-zero");
  }
  if (expected_frame.empty() || state.canonical_frame_id != expected_frame ||
    state.header.frame_id != state.canonical_frame_id)
  {
    throw std::invalid_argument("localization state canonical frame is inconsistent");
  }
  return MemoryDomainKey(mode, state.localization_epoch_id, state.canonical_frame_id);
}

LidarObservation lidar_observation_from_tracklet(
  const track_robot_interfaces::msg::LidarTracklet & tracklet,
  std::uint64_t source_epoch_id,
  const geometry_msgs::msg::TransformStamped & transform)
{
  if (source_epoch_id == 0U || tracklet.tracklet_id < 0) {
    throw std::invalid_argument("LiDAR producer identity must be valid");
  }
  const auto rotation = rotation_matrix(transform.transform.rotation);
  if (!finite(transform.transform.translation.x) ||
    !finite(transform.transform.translation.y) ||
    !finite(transform.transform.translation.z))
  {
    throw std::invalid_argument("transform translation must be finite");
  }
  const Vector3 source_position{
    tracklet.position.x, tracklet.position.y, tracklet.position.z};
  const Vector3 source_velocity{
    tracklet.velocity.x, tracklet.velocity.y, tracklet.velocity.z};
  const Vector3 source_extent{tracklet.size.x, tracklet.size.y, tracklet.size.z};
  if (!std::all_of(source_position.begin(), source_position.end(), finite) ||
    !std::all_of(source_velocity.begin(), source_velocity.end(), finite) ||
    !std::all_of(source_extent.begin(), source_extent.end(), finite) ||
    std::any_of(source_extent.begin(), source_extent.end(),
      [](double value) {return value < 0.0;}))
  {
    throw std::invalid_argument("LiDAR tracklet geometry is invalid");
  }
  const double xx = tracklet.position_covariance_xy[0];
  const double xy = 0.5 * (
    static_cast<double>(tracklet.position_covariance_xy[1]) +
    static_cast<double>(tracklet.position_covariance_xy[2]));
  const double yy = tracklet.position_covariance_xy[3];
  if (!finite(xx) || !finite(xy) || !finite(yy) || xx < 0.0 || yy < 0.0) {
    throw std::invalid_argument("LiDAR XY covariance is invalid");
  }
  const Matrix3 source_covariance{
    xx, xy, 0.0,
    xy, yy, 0.0,
    0.0, 0.0, std::max(xx, yy)};

  LidarObservation output;
  output.source_key = {source_epoch_id, tracklet.tracklet_id};
  output.source_stamp_ns = time_to_nanoseconds(tracklet.last_measurement_stamp);
  output.position = multiply(rotation, source_position);
  output.position[0] += transform.transform.translation.x;
  output.position[1] += transform.transform.translation.y;
  output.position[2] += transform.transform.translation.z;
  output.velocity = multiply(rotation, source_velocity);
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column < 3U; ++column) {
      output.extent[row] +=
        std::abs(rotation[row * 3U + column]) * source_extent[column];
    }
  }
  output.position_covariance = rotate_covariance(rotation, source_covariance);
  const double confidence = tracklet.confidence;
  const double quality = tracklet.observation_quality;
  if (!finite(confidence) || confidence < 0.0 || confidence > 1.0 ||
    !finite(quality) || quality < 0.0 || quality > 1.0)
  {
    throw std::invalid_argument("LiDAR tracklet confidence is invalid");
  }
  output.confidence = quality > 0.0 ? std::min(confidence, quality) : confidence;
  output.validate();
  return output;
}

VisualMemorySupplement visual_supplement_from_semantic_observation(
  const track_robot_interfaces::msg::SemanticObservation & observation,
  const VisualAssociationKey & visual_key,
  const LidarAssociationKey & lidar_key,
  double association_confidence,
  bool appearance_memory_enabled)
{
  using Observation = track_robot_interfaces::msg::SemanticObservation;
  VisualMemorySupplement supplement;
  supplement.lidar_key = {lidar_key.source_epoch_id, lidar_key.tracklet_id};
  supplement.visual_key = visual_key;
  supplement.observation_producer_epoch_id = observation.producer_epoch_id;
  supplement.observation_id = observation.observation_id;
  supplement.visual_candidate_id = observation.visual_candidate_id;
  supplement.camera_stamp_ns = time_to_nanoseconds(observation.camera_stamp);
  supplement.association_confidence = association_confidence;
  supplement.association_confirmed = true;
  supplement.semantic_evidence_valid = !observation.semantic_labels.empty();
  const auto physical_evidence = Observation::EVIDENCE_CAMERA |
    Observation::EVIDENCE_LIDAR | Observation::EVIDENCE_STEREO_DEPTH;
  supplement.prediction_only =
    (observation.evidence_flags & Observation::EVIDENCE_PREDICTION) != 0U &&
    (observation.evidence_flags & physical_evidence) == 0U;
  supplement.appearance_evidence_valid = appearance_memory_enabled &&
    !observation.appearance_descriptor.values.empty() &&
    std::isfinite(observation.appearance_confidence) &&
    observation.appearance_confidence > 0.0F;
  if (supplement.appearance_evidence_valid) {
    AppearanceDescriptor descriptor;
    descriptor.encoder_id = observation.appearance_descriptor.encoder_id;
    descriptor.checkpoint_id = observation.appearance_descriptor.checkpoint_id;
    descriptor.version = observation.appearance_descriptor.version;
    descriptor.dimension = observation.appearance_descriptor.dimension;
    descriptor.l2_normalized = observation.appearance_descriptor.l2_normalized;
    descriptor.values.reserve(observation.appearance_descriptor.values.size());
    for (const float value : observation.appearance_descriptor.values) {
      descriptor.values.push_back(static_cast<double>(value));
    }
    supplement.appearance_descriptor = std::move(descriptor);
    supplement.appearance_quality =
      static_cast<double>(observation.appearance_descriptor.quality);
    if (!std::isfinite(supplement.appearance_quality) ||
      std::abs(supplement.appearance_quality -
      static_cast<double>(observation.appearance_confidence)) > 1e-6)
    {
      supplement.appearance_quality =
        std::numeric_limits<double>::quiet_NaN();
    }
  }
  for (const auto & label : observation.semantic_labels) {
    supplement.semantic_labels.push_back({
        label.label, label.confidence, label.provenance,
        label.evidence_kind, label.source_observation_id});
  }
  supplement.validate();
  return supplement;
}

track_robot_interfaces::msg::SemanticObject semantic_object_from_memory(
  const MemoryObject & object,
  const MemoryDomainKey & domain)
{
  if (!object.key.valid() || !object.lidar_key.valid() ||
    object.short_history.size() > 16U)
  {
    throw std::invalid_argument("memory object violates the public ROS contract");
  }
  track_robot_interfaces::msg::SemanticObject output;
  output.header.frame_id = domain.canonical_frame_id();
  output.header.stamp = nanoseconds_to_time(object.state_stamp_ns);
  output.memory_epoch_id = object.key.memory_epoch_id;
  output.global_object_id = object.key.global_object_id;
  output.lidar_tracklet_id_valid = true;
  output.lidar_source_epoch_id = object.lidar_key.producer_epoch_id;
  output.lidar_tracklet_id = object.lidar_key.local_object_id;
  if (object.attached_visual_key.has_value()) {
    if (object.attached_visual_key->kind == VisualAssociationKind::kCameraTrack) {
      output.camera_track_id_valid = true;
      output.camera_source_epoch_id = object.attached_visual_key->producer_epoch_id;
      output.camera_track_id = static_cast<std::int64_t>(
        object.attached_visual_key->local_id);
    }
    output.visual_candidate_id_valid = object.visual_candidate_id != 0U;
    output.visual_producer_epoch_id =
      object.visual_observation_producer_epoch_id;
    output.visual_candidate_id = object.visual_candidate_id;
  }
  output.memory_mode = static_cast<std::uint8_t>(domain.mode());
  output.localization_epoch_id = domain.localization_epoch_id();
  output.position_frame_id = domain.canonical_frame_id();
  output.position_valid = true;
  output.position.x = object.position[0];
  output.position.y = object.position[1];
  output.position.z = object.position[2];
  output.velocity_valid = true;
  output.velocity.x = object.velocity[0];
  output.velocity.y = object.velocity[1];
  output.velocity.z = object.velocity[2];
  output.extent_valid = true;
  output.extent.x = object.extent[0];
  output.extent.y = object.extent[1];
  output.extent.z = object.extent[2];
  for (std::size_t index = 0U; index < object.position_covariance.size(); ++index) {
    output.position_covariance[index] =
      static_cast<float>(object.position_covariance[index]);
  }
  output.lifecycle_state = static_cast<std::uint8_t>(object.lifecycle);
  output.support_state = static_cast<std::uint8_t>(object.support);
  output.visibility_state = static_cast<std::uint8_t>(object.visibility);
  output.motion_class = static_cast<std::uint8_t>(object.motion);
  output.reidentification_state = static_cast<std::uint8_t>(
    object.reidentification_state);
  output.inspection_state = output.INSPECTION_NOT_INSPECTED;
  output.first_seen = nanoseconds_to_time(object.first_seen_ns);
  output.last_seen = nanoseconds_to_time(std::max(
      object.last_seen_ns, object.last_camera_seen_ns));
  output.last_lidar_seen = nanoseconds_to_time(object.last_seen_ns);
  if (object.last_camera_seen_ns >= 0) {
    output.last_camera_seen = nanoseconds_to_time(object.last_camera_seen_ns);
  }
  output.observation_count = object.observation_count +
    object.camera_observation_count;
  output.camera_observation_count = object.camera_observation_count;
  output.lidar_observation_count = object.observation_count;
  output.appearance_summary_id = object.appearance_summary_id;
  output.appearance_prototype_count = object.appearance_prototype_count;
  output.appearance_encoder_id = object.appearance_encoder_id;
  output.appearance_checkpoint_id = object.appearance_checkpoint_id;
  output.appearance_descriptor_version = object.appearance_descriptor_version;
  for (const auto & label : object.semantic_labels) {
    track_robot_interfaces::msg::SemanticLabelEvidence converted;
    converted.label = label.label;
    converted.confidence = static_cast<float>(label.confidence);
    converted.provenance = label.provenance;
    converted.evidence_kind = label.evidence_kind;
    converted.source_observation_id = label.source_observation_id;
    output.semantic_labels.push_back(std::move(converted));
  }
  const double uncertainty = std::sqrt(std::max({
      0.0, object.position_covariance[0], object.position_covariance[4],
      object.position_covariance[8]}));
  for (const auto & history : object.short_history) {
    track_robot_interfaces::msg::SemanticObjectHistorySample sample;
    sample.stamp = nanoseconds_to_time(history.source_stamp_ns);
    sample.position.x = history.position[0];
    sample.position.y = history.position[1];
    sample.position.z = history.position[2];
    sample.uncertainty = static_cast<float>(uncertainty);
    sample.lifecycle_state = static_cast<std::uint8_t>(history.lifecycle);
    sample.support_state = static_cast<std::uint8_t>(history.support);
    output.short_history.push_back(sample);
  }
  output.association_confidence = static_cast<float>(
    object.attached_visual_key.has_value() ?
    object.association_confidence : object.confidence);
  output.fusion_confidence = static_cast<float>(object.confidence);
  output.uncertainty = static_cast<float>(uncertainty);
  return output;
}

track_robot_interfaces::msg::SemanticObject semantic_object_from_runtime_view(
  const RuntimeObjectView & view,
  const MemoryDomainKey & domain)
{
  auto output = semantic_object_from_memory(view.object, domain);
  output.inspection_state = static_cast<std::uint8_t>(view.inspection);
  if (view.active_task.has_value()) {
    output.active_query_id = view.active_task->query_id;
    output.active_query_version = view.active_task->query_version;
    output.task_relevance = static_cast<float>(view.task_relevance);
  }
  return output;
}

track_robot_interfaces::msg::SemanticObjectArray semantic_object_array_from_result(
  const MemoryUpdateResult & result,
  const MemoryDomainKey & domain,
  const std_msgs::msg::Header & header,
  std::uint64_t snapshot_sequence)
{
  if (result.memory_epoch_id == 0U || result.active_objects.size() > 256U ||
    snapshot_sequence == 0U)
  {
    throw std::invalid_argument("memory snapshot violates the public ROS contract");
  }
  track_robot_interfaces::msg::SemanticObjectArray output;
  output.header = header;
  output.header.frame_id = domain.canonical_frame_id();
  output.memory_epoch_id = result.memory_epoch_id;
  output.snapshot_sequence = snapshot_sequence;
  for (const auto & object : result.active_objects) {
    if (object.key.memory_epoch_id != result.memory_epoch_id) {
      throw std::invalid_argument("snapshot contains an object from another memory epoch");
    }
    output.objects.push_back(semantic_object_from_memory(object, domain));
  }
  return output;
}

track_robot_interfaces::msg::SemanticMemoryEvent semantic_event_from_memory(
  const MemoryEvent & event,
  const std_msgs::msg::Header & header,
  std::uint64_t sequence,
  std::uint64_t memory_epoch_id)
{
  if (sequence == 0U || memory_epoch_id == 0U) {
    throw std::invalid_argument("memory event sequence and epoch must be non-zero");
  }
  if (event.object_key.valid() && event.object_key.memory_epoch_id != memory_epoch_id) {
    throw std::invalid_argument("memory event object belongs to another epoch");
  }
  track_robot_interfaces::msg::SemanticMemoryEvent output;
  output.header = header;
  output.sequence = sequence;
  output.memory_epoch_id = memory_epoch_id;
  output.global_object_id = event.object_key.global_object_id;
  output.event_type = event_type(event.type);
  output.previous_lifecycle_state =
    static_cast<std::uint8_t>(event.previous_lifecycle);
  output.current_lifecycle_state =
    static_cast<std::uint8_t>(event.current_lifecycle);
  output.reason = event_reason(event.type);
  return output;
}

track_robot_interfaces::msg::AssociationDebug association_debug_from_score(
  const PairAssociationScore & score,
  const std_msgs::msg::Header & header,
  std::uint64_t memory_epoch_id,
  std::uint64_t observation_producer_epoch_id,
  std::uint64_t visual_candidate_id,
  std::uint64_t lidar_source_epoch_id,
  std::int64_t lidar_tracklet_id,
  std::uint8_t decision,
  double top_two_margin,
  const std::string & reason)
{
  using Debug = track_robot_interfaces::msg::AssociationDebug;
  if (score.terms.size() > 24U || !finite(score.total_score) ||
    !finite(top_two_margin) || decision > Debug::DECISION_AMBIGUOUS ||
    reason.size() > 256U)
  {
    throw std::invalid_argument("association debug violates its bounded ROS contract");
  }
  Debug output;
  output.header = header;
  output.memory_epoch_id = memory_epoch_id;
  output.observation_producer_epoch_id = observation_producer_epoch_id;
  output.visual_candidate_id = visual_candidate_id;
  output.lidar_source_epoch_id = lidar_source_epoch_id;
  output.lidar_tracklet_id = lidar_tracklet_id;
  output.global_object_id_valid = false;
  output.total_score = static_cast<float>(score.total_score);
  output.assignment_cost = static_cast<float>(std::max(0.0, 1.0 - score.total_score));
  output.top_two_margin = static_cast<float>(top_two_margin);
  output.decision = decision;
  output.reason = reason;
  for (const auto & term : score.terms) {
    if (term.name.empty() || term.name.size() > 128U ||
      !finite(term.weight) || term.weight < 0.0)
    {
      throw std::invalid_argument("association term violates its bounded ROS contract");
    }
    track_robot_interfaces::msg::AssociationTerm converted;
    converted.name = term.name;
    converted.valid = term.valid;
    converted.hard_gate = term.hard_gate;
    converted.gate_passed = term.gate_passed;
    converted.raw_value = static_cast<float>(term.raw_value);
    converted.normalized_value = static_cast<float>(term.normalized_value);
    converted.weight = static_cast<float>(term.weight);
    converted.contribution = static_cast<float>(term.contribution);
    output.terms.push_back(std::move(converted));
  }
  return output;
}

}  // namespace track_robot_semantic_memory
