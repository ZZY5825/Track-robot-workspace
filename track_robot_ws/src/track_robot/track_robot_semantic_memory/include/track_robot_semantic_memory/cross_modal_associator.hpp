#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "track_robot_semantic_memory/association_terms.hpp"

namespace track_robot_semantic_memory
{

struct AssociationWeights
{
  double position_consistency{0.0};
  double projected_centroid{0.0};
  double inside_fraction{0.0};
  double projected_iou{0.0};
  double visual_cosine{0.0};
  double extent_consistency{0.0};
  double point_count_consistency{0.0};
  double motion_continuity{0.0};
  double previous_association{0.0};
  double detector_confidence{0.0};
  double geometry_confidence{0.0};
  double sensor_confidence{0.0};
};

struct AssociationConfig
{
  double max_source_time_delta_s{0.1};
  double max_evidence_age_s{0.5};
  double max_position_nis{9.21};
  double minimum_size_ratio{0.25};
  double maximum_size_ratio{4.0};
  double max_relative_speed_mps{3.0};
  bool require_position_nis{true};
  bool require_size_ratio{true};
  bool require_motion_gate{true};
  bool require_descriptors{false};
  double position_distance_max_m{3.0};
  double center_distance_max_px{200.0};
  double descriptor_normalization_tolerance{1e-4};
  AssociationWeights weights{};
};

struct PairEvidence
{
  std::int64_t visual_stamp_ns{0};
  std::int64_t lidar_stamp_ns{0};
  std::int64_t evaluation_stamp_ns{0};
  std::string visual_domain;
  std::string lidar_domain;
  bool transform_available{false};
  bool calibration_available{false};
  bool field_of_view_compatible{false};
  std::optional<double> position_nis;
  std::optional<double> size_ratio;
  std::optional<double> relative_speed_mps;
  std::optional<double> position_distance_m;
  std::optional<double> projected_centroid_distance_px;
  std::optional<double> projected_inside_fraction;
  std::optional<double> projected_iou;
  std::optional<AppearanceDescriptor> visual_descriptor;
  std::optional<AppearanceDescriptor> memory_descriptor;
  std::optional<double> extent_consistency;
  std::optional<double> point_count_consistency;
  std::optional<double> motion_continuity;
  std::optional<double> previous_association;
  std::optional<double> detector_confidence;
  std::optional<double> geometry_confidence;
  std::optional<double> sensor_confidence;
};

struct PairAssociationScore
{
  bool accepted_by_gates{false};
  double total_score{0.0};
  std::vector<AssociationTermResult> terms;
  std::string rejection_reason;
};

class CrossModalAssociator
{
public:
  explicit CrossModalAssociator(AssociationConfig config);

  [[nodiscard]] PairAssociationScore score(
    const PairEvidence & evidence) const;

private:
  AssociationConfig config_;
};

}  // namespace track_robot_semantic_memory
