#include "track_robot_semantic_memory/multisensor_update.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace track_robot_semantic_memory
{
namespace
{

constexpr double kNanosecondsPerSecond = 1.0e9;

bool finite_nonnegative(double value) noexcept
{
  return std::isfinite(value) && value >= 0.0;
}

template<std::size_t Size>
bool all_finite(const std::array<double, Size> & values) noexcept
{
  return std::all_of(
    values.begin(), values.end(),
    [](double value) {return std::isfinite(value);});
}

void validate_state(const MultisensorObjectState & state)
{
  if (state.state_stamp_ns < 0 || state.last_supported_stamp_ns < 0 ||
    state.last_camera_stamp_ns < 0 || state.last_lidar_stamp_ns < 0 ||
    state.last_supported_stamp_ns > state.state_stamp_ns ||
    state.last_camera_stamp_ns > state.state_stamp_ns ||
    state.last_lidar_stamp_ns > state.state_stamp_ns ||
    !all_finite(state.position) || !all_finite(state.velocity) ||
    !all_finite(state.position_covariance) ||
    !std::isfinite(state.confidence) || state.confidence < 0.0 ||
    state.confidence > 1.0 || state.position_covariance[0] < 0.0 ||
    state.position_covariance[4] < 0.0 || state.position_covariance[8] < 0.0)
  {
    throw std::invalid_argument("multisensor state is invalid");
  }
}

void validate_evidence(const MultisensorEvidence & evidence)
{
  if (evidence.source_stamp_ns < 0 ||
    !all_finite(evidence.position) ||
    !all_finite(evidence.position_covariance) ||
    !std::isfinite(evidence.camera_confidence) ||
    evidence.camera_confidence < 0.0 || evidence.camera_confidence > 1.0 ||
    !std::isfinite(evidence.lidar_confidence) ||
    evidence.lidar_confidence < 0.0 || evidence.lidar_confidence > 1.0)
  {
    throw std::invalid_argument("multisensor evidence is invalid");
  }
  if (evidence.position_valid &&
    (evidence.position_covariance[0] < 0.0 ||
    evidence.position_covariance[4] < 0.0 ||
    evidence.position_covariance[8] < 0.0))
  {
    throw std::invalid_argument("measurement covariance is invalid");
  }
}

SupportState support_from_evidence(const MultisensorEvidence & evidence) noexcept
{
  const bool camera_valid = evidence.camera_observed &&
    evidence.association_confirmed && !evidence.ambiguous;
  if (camera_valid && evidence.lidar_observed) {
    return SupportState::kCameraLidar;
  }
  if (evidence.lidar_observed) {
    return SupportState::kLidarOnly;
  }
  if (camera_valid) {
    return SupportState::kCameraOnly;
  }
  if (evidence.prediction_available) {
    return SupportState::kPredictionOnly;
  }
  return SupportState::kNone;
}

VisibilityState visibility_from_evidence(
  const MultisensorEvidence & evidence) noexcept
{
  if (!evidence.camera_visibility_known) {
    return VisibilityState::kUnknown;
  }
  if (!evidence.in_camera_field_of_view) {
    return VisibilityState::kOutsideFieldOfView;
  }
  if (evidence.camera_occluded) {
    return VisibilityState::kOccluded;
  }
  if (evidence.camera_observed && evidence.association_confirmed &&
    !evidence.ambiguous)
  {
    return VisibilityState::kVisible;
  }
  return VisibilityState::kUnknown;
}

void increment_saturating(std::uint32_t * value) noexcept
{
  if (*value != std::numeric_limits<std::uint32_t>::max()) {
    ++(*value);
  }
}

}  // namespace

MultisensorUpdatePermissions multisensor_update_permissions(
  LifecycleState lifecycle, SupportState support) noexcept
{
  MultisensorUpdatePermissions output;
  if (lifecycle == LifecycleState::kArchived ||
    lifecycle == LifecycleState::kLost)
  {
    return output;
  }

  const bool permanent_visual_updates = lifecycle == LifecycleState::kConfirmed ||
    lifecycle == LifecycleState::kStale;
  switch (support) {
    case SupportState::kCameraLidar:
      output.position_measurement = true;
      output.covariance = true;
      output.semantics = permanent_visual_updates;
      output.appearance = permanent_visual_updates;
      output.confidence_increase = true;
      break;
    case SupportState::kCameraOnly:
      output.semantics = permanent_visual_updates;
      output.appearance = permanent_visual_updates;
      output.confidence_increase = permanent_visual_updates;
      break;
    case SupportState::kLidarOnly:
      output.position_measurement = true;
      output.covariance = true;
      output.confidence_increase = true;
      break;
    case SupportState::kPredictionOnly:
      output.position_prediction = true;
      output.covariance = true;
      output.confidence_decay = true;
      break;
    case SupportState::kNone:
      output.confidence_decay = true;
      break;
  }
  return output;
}

void MultisensorUpdateConfig::validate() const
{
  if (static_stale_after_ns < 0 || static_lost_after_ns <= static_stale_after_ns ||
    dynamic_stale_after_ns < 0 || dynamic_lost_after_ns <= dynamic_stale_after_ns ||
    !finite_nonnegative(static_process_noise) ||
    !finite_nonnegative(dynamic_process_noise) ||
    !finite_nonnegative(static_confidence_decay_per_second) ||
    !finite_nonnegative(dynamic_confidence_decay_per_second))
  {
    throw std::invalid_argument("multisensor update config is invalid");
  }
}

MultisensorUpdater::MultisensorUpdater(MultisensorUpdateConfig config)
: config_(std::move(config))
{
  config_.validate();
}

MultisensorUpdateResult MultisensorUpdater::update(
  const MultisensorObjectState & state,
  const MultisensorEvidence & evidence) const
{
  validate_state(state);
  validate_evidence(evidence);
  if (evidence.source_stamp_ns <= state.state_stamp_ns) {
    throw std::invalid_argument("multisensor source time must increase");
  }

  MultisensorUpdateResult result;
  result.state = state;
  if (state.lifecycle == LifecycleState::kArchived) {
    result.reason = "archived objects are immutable";
    return result;
  }

  const SupportState support = support_from_evidence(evidence);
  const bool measurement_support = support == SupportState::kCameraLidar ||
    support == SupportState::kCameraOnly || support == SupportState::kLidarOnly;
  if (state.lifecycle == LifecycleState::kLost && measurement_support &&
    !evidence.reactivation_confirmed)
  {
    result.reason = "lost object requires strict multi-frame reactivation";
    return result;
  }
  if (state.lifecycle == LifecycleState::kLost &&
    evidence.reactivation_confirmed && measurement_support)
  {
    result.state.lifecycle = LifecycleState::kConfirmed;
  } else if (state.lifecycle == LifecycleState::kStale && measurement_support) {
    result.state.lifecycle = LifecycleState::kConfirmed;
  }

  result.state.support = support;
  result.state.visibility = visibility_from_evidence(evidence);
  const auto permissions = multisensor_update_permissions(
    result.state.lifecycle, support);
  const double elapsed_seconds = static_cast<double>(
    evidence.source_stamp_ns - state.state_stamp_ns) / kNanosecondsPerSecond;

  const bool camera_valid = evidence.camera_observed &&
    evidence.association_confirmed && !evidence.ambiguous;
  if (evidence.lidar_observed && !evidence.position_valid) {
    throw std::invalid_argument("LiDAR support requires a metric position");
  }
  const bool camera_metric = support == SupportState::kCameraOnly &&
    evidence.explicit_metric_depth && evidence.position_valid;
  if (permissions.position_measurement && evidence.position_valid) {
    result.state.position = evidence.position;
    result.state.position_covariance = evidence.position_covariance;
    result.position_updated = true;
    result.covariance_updated = true;
  } else if (camera_metric) {
    result.state.position = evidence.position;
    result.state.position_covariance = evidence.position_covariance;
    result.position_updated = true;
    result.covariance_updated = true;
  } else if (permissions.position_prediction) {
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
      result.state.position[axis] += state.velocity[axis] * elapsed_seconds;
    }
    const double noise = state.motion == MotionState::kDynamic ?
      config_.dynamic_process_noise : config_.static_process_noise;
    for (const std::size_t diagonal : {0U, 4U, 8U}) {
      result.state.position_covariance[diagonal] += noise * elapsed_seconds;
    }
    result.position_predicted = true;
    result.covariance_updated = true;
  }

  if (permissions.semantics && camera_valid &&
    evidence.semantic_evidence_valid)
  {
    increment_saturating(&result.state.semantic_update_count);
    result.semantics_updated = true;
  }
  if (permissions.appearance && camera_valid &&
    evidence.appearance_evidence_valid)
  {
    increment_saturating(&result.state.appearance_update_count);
    result.appearance_updated = true;
  }

  if (support == SupportState::kCameraLidar) {
    const double observed = std::max(
      evidence.camera_confidence, evidence.lidar_confidence);
    result.state.confidence += 0.25 * (observed - result.state.confidence);
  } else if (support == SupportState::kLidarOnly) {
    result.state.confidence += 0.20 * (
      evidence.lidar_confidence - result.state.confidence);
  } else if (support == SupportState::kCameraOnly) {
    result.state.confidence += 0.10 * (
      evidence.camera_confidence - result.state.confidence);
  } else {
    const double decay_rate = state.motion == MotionState::kDynamic ?
      config_.dynamic_confidence_decay_per_second :
      config_.static_confidence_decay_per_second;
    const double multiplier = support == SupportState::kNone ? 2.0 : 1.0;
    result.state.confidence *= std::exp(
      -decay_rate * multiplier * elapsed_seconds);
  }
  result.state.confidence = std::clamp(result.state.confidence, 0.0, 1.0);

  if (camera_valid) {
    result.state.last_camera_stamp_ns = evidence.source_stamp_ns;
  }
  if (evidence.lidar_observed) {
    result.state.last_lidar_stamp_ns = evidence.source_stamp_ns;
  }
  if (measurement_support) {
    result.state.last_supported_stamp_ns = evidence.source_stamp_ns;
  } else {
    const std::int64_t unsupported_age = evidence.source_stamp_ns -
      state.last_supported_stamp_ns;
    const bool dynamic = state.motion == MotionState::kDynamic;
    const std::int64_t stale_after = dynamic ?
      config_.dynamic_stale_after_ns : config_.static_stale_after_ns;
    const std::int64_t lost_after = dynamic ?
      config_.dynamic_lost_after_ns : config_.static_lost_after_ns;
    if (unsupported_age > lost_after) {
      result.state.lifecycle = LifecycleState::kLost;
    } else if (unsupported_age > stale_after &&
      result.state.lifecycle == LifecycleState::kConfirmed)
    {
      result.state.lifecycle = LifecycleState::kStale;
    }
  }
  result.state.state_stamp_ns = evidence.source_stamp_ns;
  result.accepted = true;
  result.reason = "multisensor update applied";
  return result;
}

}  // namespace track_robot_semantic_memory
