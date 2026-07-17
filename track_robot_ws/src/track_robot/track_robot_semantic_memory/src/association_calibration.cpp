#include "track_robot_semantic_memory/association_calibration.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

namespace track_robot_semantic_memory
{
namespace
{

constexpr double kTolerance = 1.0e-12;

using NamedValue = std::pair<const char *, double>;

bool close(double left, double right) noexcept
{
  return std::isfinite(left) && std::isfinite(right) &&
         std::abs(left - right) <= kTolerance;
}

void require_double(
  const nlohmann::json & values, const char * name, double expected)
{
  if (!values.contains(name) || !close(values.at(name).get<double>(), expected)) {
    throw std::invalid_argument(
            std::string("association calibration contract mismatch: ") + name);
  }
}

std::array<NamedValue, 12U> weights(const AssociationWeights & value)
{
  return {{
      {"position_consistency", value.position_consistency},
      {"projected_centroid", value.projected_centroid},
      {"inside_fraction", value.inside_fraction},
      {"projected_iou", value.projected_iou},
      {"visual_cosine", value.visual_cosine},
      {"extent_consistency", value.extent_consistency},
      {"point_count_consistency", value.point_count_consistency},
      {"motion_continuity", value.motion_continuity},
      {"previous_association", value.previous_association},
      {"detector_confidence", value.detector_confidence},
      {"geometry_confidence", value.geometry_confidence},
      {"sensor_confidence", value.sensor_confidence}}};
}

}  // namespace

void validate_association_calibration_report(
  const nlohmann::json & report,
  const AssociationCalibrationRuntimeContract & runtime)
{
  try {
    const auto & selected = report.at("selected_parameters");
    const auto & metrics = selected.at("association_metrics");
    const auto & calibrated_weights = selected.at(
      "term_weights_from_median_separation");
    const auto & contract = report.at("runtime_contract");
    const auto & gates = contract.at("hard_gates");
    const auto & report_weights = contract.at("soft_weights");
    const auto & confirmation = contract.at("confirmation");

    if (runtime.camera_calibration_id.empty() ||
      report.at("status").get<std::string>() != "calibrated" ||
      !report.at("camera_attachment_allowed").get<bool>() ||
      report.at("counts").at("positive").get<std::size_t>() < 20U ||
      report.at("counts").at("negative").get<std::size_t>() < 20U ||
      report.at("hard_gate_pass_counts").at("positive").get<std::size_t>() < 20U ||
      metrics.at("precision").get<double>() < 0.95 ||
      metrics.at("recall").get<double>() < 0.80 ||
      contract.at("scoring_contract_version").get<std::string>() !=
      runtime.scoring_contract_version ||
      contract.at("camera_calibration_id").get<std::string>() !=
      runtime.camera_calibration_id)
    {
      throw std::invalid_argument(
              "association calibration report does not authorize attachment");
    }

    require_double(selected, "match_threshold", runtime.runtime.match_threshold);
    require_double(
      selected, "ambiguity_margin",
      runtime.runtime.confirmation.ambiguity_margin);
    require_double(
      metrics, "threshold", runtime.runtime.match_threshold);

    const std::array<NamedValue, 9U> gate_values{{
        {"max_source_time_delta_s", runtime.association.max_source_time_delta_s},
        {"max_evidence_age_s", runtime.association.max_evidence_age_s},
        {"max_position_nis", runtime.association.max_position_nis},
        {"minimum_size_ratio", runtime.association.minimum_size_ratio},
        {"maximum_size_ratio", runtime.association.maximum_size_ratio},
        {"max_relative_speed_mps", runtime.association.max_relative_speed_mps},
        {"position_distance_max_m", runtime.association.position_distance_max_m},
        {"center_distance_max_px", runtime.association.center_distance_max_px},
        {"descriptor_normalization_tolerance",
          runtime.association.descriptor_normalization_tolerance}}};
    for (const auto & value : gate_values) {
      require_double(gates, value.first, value.second);
    }
    if (gates.at("require_position_nis").get<bool>() !=
      runtime.association.require_position_nis ||
      gates.at("require_size_ratio").get<bool>() !=
      runtime.association.require_size_ratio ||
      gates.at("require_motion_gate").get<bool>() !=
      runtime.association.require_motion_gate ||
      gates.at("require_descriptors").get<bool>() !=
      runtime.association.require_descriptors)
    {
      throw std::invalid_argument("association required-evidence contract mismatch");
    }

    double weight_sum = 0.0;
    for (const auto & value : weights(runtime.association.weights)) {
      require_double(report_weights, value.first, value.second);
      const double calibrated = calibrated_weights.contains(value.first) ?
        calibrated_weights.at(value.first).get<double>() : 0.0;
      if (!close(calibrated, value.second)) {
        throw std::invalid_argument("calibrated and runtime weights differ");
      }
      weight_sum += value.second;
    }
    if (!close(weight_sum, 1.0)) {
      throw std::invalid_argument("runtime association weights must sum to one");
    }

    require_double(
      confirmation, "previous_association_hysteresis",
      runtime.runtime.confirmation.previous_association_hysteresis);
    if (confirmation.at("confirmation_frames").get<std::uint32_t>() !=
      runtime.runtime.confirmation.confirmation_frames ||
      confirmation.at("detach_after_misses").get<std::uint32_t>() !=
      runtime.runtime.confirmation.detach_after_misses ||
      confirmation.at("cooldown_frames").get<std::uint64_t>() !=
      runtime.runtime.confirmation.cooldown_frames)
    {
      throw std::invalid_argument("association confirmation contract mismatch");
    }
  } catch (const std::invalid_argument &) {
    throw;
  } catch (const std::exception & error) {
    throw std::invalid_argument(
            std::string("association calibration report is incomplete: ") + error.what());
  }
}

}  // namespace track_robot_semantic_memory
