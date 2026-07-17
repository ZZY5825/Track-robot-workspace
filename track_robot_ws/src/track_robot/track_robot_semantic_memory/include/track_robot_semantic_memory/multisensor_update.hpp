#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "track_robot_semantic_memory/types.hpp"

namespace track_robot_semantic_memory
{

struct MultisensorUpdatePermissions
{
  bool position_measurement{false};
  bool position_prediction{false};
  bool covariance{false};
  bool semantics{false};
  bool appearance{false};
  bool confidence_increase{false};
  bool confidence_decay{false};
};

[[nodiscard]] MultisensorUpdatePermissions multisensor_update_permissions(
  LifecycleState lifecycle, SupportState support) noexcept;

struct MultisensorUpdateConfig
{
  std::int64_t static_stale_after_ns{5'000'000'000LL};
  std::int64_t static_lost_after_ns{10'000'000'000LL};
  std::int64_t dynamic_stale_after_ns{1'000'000'000LL};
  std::int64_t dynamic_lost_after_ns{3'000'000'000LL};
  double static_process_noise{0.1};
  double dynamic_process_noise{0.5};
  double static_confidence_decay_per_second{0.1};
  double dynamic_confidence_decay_per_second{0.2};

  void validate() const;
};

struct MultisensorObjectState
{
  LifecycleState lifecycle{LifecycleState::kTentative};
  SupportState support{SupportState::kNone};
  VisibilityState visibility{VisibilityState::kUnknown};
  MotionState motion{MotionState::kUncertain};
  std::array<double, 3U> position{};
  std::array<double, 3U> velocity{};
  std::array<double, 9U> position_covariance{};
  double confidence{0.0};
  std::int64_t state_stamp_ns{0};
  std::int64_t last_supported_stamp_ns{0};
  std::int64_t last_camera_stamp_ns{0};
  std::int64_t last_lidar_stamp_ns{0};
  std::uint32_t semantic_update_count{0U};
  std::uint32_t appearance_update_count{0U};
};

struct MultisensorEvidence
{
  std::int64_t source_stamp_ns{0};
  bool camera_observed{false};
  bool lidar_observed{false};
  bool prediction_available{false};
  bool camera_visibility_known{false};
  bool in_camera_field_of_view{false};
  bool camera_occluded{false};
  bool association_confirmed{false};
  bool ambiguous{false};
  bool reactivation_confirmed{false};
  bool explicit_metric_depth{false};
  bool position_valid{false};
  std::array<double, 3U> position{};
  std::array<double, 9U> position_covariance{};
  bool semantic_evidence_valid{false};
  bool appearance_evidence_valid{false};
  double camera_confidence{0.0};
  double lidar_confidence{0.0};
};

struct MultisensorUpdateResult
{
  MultisensorObjectState state;
  bool accepted{false};
  bool position_updated{false};
  bool position_predicted{false};
  bool covariance_updated{false};
  bool semantics_updated{false};
  bool appearance_updated{false};
  std::string reason;
};

class MultisensorUpdater
{
public:
  explicit MultisensorUpdater(MultisensorUpdateConfig config);

  MultisensorUpdateResult update(
    const MultisensorObjectState & state,
    const MultisensorEvidence & evidence) const;

private:
  MultisensorUpdateConfig config_;
};

}  // namespace track_robot_semantic_memory
