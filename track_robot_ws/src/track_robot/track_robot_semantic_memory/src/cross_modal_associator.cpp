#include "track_robot_semantic_memory/cross_modal_associator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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

AssociationTermResult missing_term(
  const char * name, bool hard_gate, const char * reason)
{
  return invalid_association_term(name, hard_gate, reason);
}

template<typename Factory>
AssociationTermResult optional_term(
  const char * name, const std::optional<double> & value, Factory factory)
{
  if (!value.has_value()) {
    return missing_term(name, false, "evidence unavailable");
  }
  return factory(*value);
}

void add(PairAssociationScore * score, AssociationTermResult term)
{
  if (term.valid && !term.hard_gate && std::isfinite(term.contribution)) {
    score->total_score += term.contribution;
  }
  score->terms.push_back(std::move(term));
}

}  // namespace

CrossModalAssociator::CrossModalAssociator(AssociationConfig config)
: config_(std::move(config))
{
  if (!finite_nonnegative(config_.max_source_time_delta_s) ||
    !finite_nonnegative(config_.max_evidence_age_s) ||
    !finite_nonnegative(config_.max_position_nis) ||
    !finite_nonnegative(config_.minimum_size_ratio) ||
    !finite_nonnegative(config_.maximum_size_ratio) ||
    config_.maximum_size_ratio < config_.minimum_size_ratio ||
    !finite_nonnegative(config_.max_relative_speed_mps) ||
    !finite_nonnegative(config_.position_distance_max_m) ||
    config_.position_distance_max_m == 0.0 ||
    !finite_nonnegative(config_.center_distance_max_px) ||
    config_.center_distance_max_px == 0.0 ||
    !finite_nonnegative(config_.descriptor_normalization_tolerance))
  {
    throw std::invalid_argument("association gate/normalization config is invalid");
  }
  const std::array<double, 12U> weights{
    config_.weights.position_consistency,
    config_.weights.projected_centroid,
    config_.weights.inside_fraction,
    config_.weights.projected_iou,
    config_.weights.visual_cosine,
    config_.weights.extent_consistency,
    config_.weights.point_count_consistency,
    config_.weights.motion_continuity,
    config_.weights.previous_association,
    config_.weights.detector_confidence,
    config_.weights.geometry_confidence,
    config_.weights.sensor_confidence};
  for (const double weight : weights) {
    if (!finite_nonnegative(weight)) {
      throw std::invalid_argument(
              "association weights must be finite and nonnegative");
    }
  }
}

PairAssociationScore CrossModalAssociator::score(
  const PairEvidence & evidence) const
{
  PairAssociationScore result;
  result.terms.reserve(22U);

  const long double time_delta_ns = std::abs(
    static_cast<long double>(evidence.visual_stamp_ns) -
    static_cast<long double>(evidence.lidar_stamp_ns));
  add(&result, maximum_gate(
      "source_time_delta_s",
      static_cast<double>(time_delta_ns / kNanosecondsPerSecond),
      config_.max_source_time_delta_s));

  const std::int64_t oldest_stamp = std::min(
    evidence.visual_stamp_ns, evidence.lidar_stamp_ns);
  const long double age_ns = static_cast<long double>(evidence.evaluation_stamp_ns) -
    static_cast<long double>(oldest_stamp);
  add(&result, maximum_gate(
      "evidence_age_s", static_cast<double>(age_ns / kNanosecondsPerSecond),
      config_.max_evidence_age_s));

  if (evidence.visual_domain.empty() || evidence.lidar_domain.empty()) {
    add(&result, missing_term(
        "spatial_domain", true, "spatial domain unavailable"));
  } else {
    add(&result, boolean_gate(
        "spatial_domain", evidence.visual_domain == evidence.lidar_domain,
        "spatial/localization domains differ"));
  }
  add(&result, boolean_gate(
      "transform_available", evidence.transform_available,
      "source-time transform unavailable"));
  add(&result, boolean_gate(
      "calibration_available", evidence.calibration_available,
      "camera calibration unavailable"));
  add(&result, boolean_gate(
      "field_of_view", evidence.field_of_view_compatible,
      "projection outside field of view"));

  add(&result, evidence.position_nis.has_value() ?
    maximum_gate("position_nis", *evidence.position_nis, config_.max_position_nis) :
    missing_term("position_nis", true, "3D innovation unavailable"));
  add(&result, evidence.size_ratio.has_value() ?
    range_gate(
      "size_ratio", *evidence.size_ratio, config_.minimum_size_ratio,
      config_.maximum_size_ratio) :
    missing_term("size_ratio", true, "size evidence unavailable"));
  add(&result, evidence.relative_speed_mps.has_value() ?
    maximum_gate(
      "relative_speed_mps", *evidence.relative_speed_mps,
      config_.max_relative_speed_mps) :
    missing_term("relative_speed_mps", true, "motion evidence unavailable"));

  const bool have_descriptors = evidence.visual_descriptor.has_value() &&
    evidence.memory_descriptor.has_value();
  if (have_descriptors) {
    add(&result, descriptor_compatibility_gate(
        *evidence.visual_descriptor, *evidence.memory_descriptor,
        config_.descriptor_normalization_tolerance));
  } else {
    add(&result, missing_term(
        "descriptor_compatibility", true, "descriptor evidence unavailable"));
  }

  add(&result, optional_term(
      "position_consistency", evidence.position_distance_m,
      [this](double value) {
        return lower_is_better_term(
          "position_consistency", value, 0.0,
          config_.position_distance_max_m,
          config_.weights.position_consistency);
      }));
  add(&result, optional_term(
      "projected_centroid", evidence.projected_centroid_distance_px,
      [this](double value) {
        return lower_is_better_term(
          "projected_centroid", value, 0.0,
          config_.center_distance_max_px,
          config_.weights.projected_centroid);
      }));
  add(&result, optional_term(
      "inside_fraction", evidence.projected_inside_fraction,
      [this](double value) {
        return higher_is_better_term(
          "inside_fraction", value, 0.0, 1.0,
          config_.weights.inside_fraction);
      }));
  add(&result, optional_term(
      "projected_iou", evidence.projected_iou,
      [this](double value) {
        return higher_is_better_term(
          "projected_iou", value, 0.0, 1.0,
          config_.weights.projected_iou);
      }));
  if (have_descriptors) {
    add(&result, descriptor_cosine_term(
        *evidence.visual_descriptor, *evidence.memory_descriptor,
        config_.weights.visual_cosine,
        config_.descriptor_normalization_tolerance));
  } else {
    add(&result, missing_term(
        "visual_cosine", false, "descriptor evidence unavailable"));
  }

  const auto unit_term = [&result](
    const char * name, const std::optional<double> & value, double weight)
    {
      add(&result, optional_term(
          name, value,
          [name, weight](double raw) {
            return higher_is_better_term(name, raw, 0.0, 1.0, weight);
          }));
    };
  unit_term(
    "extent_consistency", evidence.extent_consistency,
    config_.weights.extent_consistency);
  unit_term(
    "point_count_consistency", evidence.point_count_consistency,
    config_.weights.point_count_consistency);
  unit_term(
    "motion_continuity", evidence.motion_continuity,
    config_.weights.motion_continuity);
  unit_term(
    "previous_association", evidence.previous_association,
    config_.weights.previous_association);
  unit_term(
    "detector_confidence", evidence.detector_confidence,
    config_.weights.detector_confidence);
  unit_term(
    "geometry_confidence", evidence.geometry_confidence,
    config_.weights.geometry_confidence);
  unit_term(
    "sensor_confidence", evidence.sensor_confidence,
    config_.weights.sensor_confidence);

  bool required_gate_failed = false;
  for (const auto & term : result.terms) {
    if (!term.hard_gate) {
      if (!term.valid && term.reason.find("invalid") != std::string::npos) {
        required_gate_failed = true;
        if (result.rejection_reason.empty()) {
          result.rejection_reason = term.name + ": " + term.reason;
        }
      }
      continue;
    }
    bool required = true;
    if (term.name == "position_nis") {
      required = config_.require_position_nis;
    } else if (term.name == "size_ratio") {
      required = config_.require_size_ratio;
    } else if (term.name == "relative_speed_mps") {
      required = config_.require_motion_gate;
    } else if (term.name == "descriptor_compatibility") {
      required = config_.require_descriptors;
    }
    const bool malformed_supplied_evidence = !term.valid &&
      term.reason.find("unavailable") == std::string::npos;
    if (malformed_supplied_evidence ||
      (required && (!term.valid || !term.gate_passed)) ||
      (term.valid && !term.gate_passed))
    {
      required_gate_failed = true;
      if (result.rejection_reason.empty()) {
        result.rejection_reason = term.name + ": " + term.reason;
      }
    }
  }
  result.accepted_by_gates = !required_gate_failed;
  return result;
}

}  // namespace track_robot_semantic_memory
