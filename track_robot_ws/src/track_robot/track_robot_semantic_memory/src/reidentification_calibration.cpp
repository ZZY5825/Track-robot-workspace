#include "track_robot_semantic_memory/reidentification_calibration.hpp"

#include <stdexcept>

#include <nlohmann/json.hpp>

namespace track_robot_semantic_memory
{

void validate_reidentification_calibration_report(
  const nlohmann::json & report,
  const ReidentificationCalibrationExpectation & expectation)
{
  expectation.config.validate();
  try {
    const auto & selected = report.at("selected_parameters");
    const bool valid =
      !expectation.contract_version.empty() &&
      report.at("contract_version").get<std::string>() ==
      expectation.contract_version &&
      report.at("status").get<std::string>() == "calibrated" &&
      report.at("reidentification_allowed").get<bool>() &&
      selected.at("maximum_age_ns").get<std::int64_t>() ==
      expectation.config.maximum_age_ns &&
      selected.at("maximum_spatial_distance_m").get<double>() ==
      expectation.config.maximum_spatial_distance_m &&
      selected.at("minimum_appearance_similarity").get<double>() ==
      expectation.config.minimum_appearance_similarity &&
      selected.at("minimum_combined_score").get<double>() ==
      expectation.config.minimum_combined_score &&
      selected.at("ambiguity_margin").get<double>() ==
      expectation.config.ambiguity_margin &&
      selected.at("confirmation_frames").get<std::uint32_t>() ==
      expectation.config.confirmation_frames;
    if (!valid) {
      throw std::invalid_argument(
              "reidentification calibration report does not match runtime");
    }
  } catch (const nlohmann::json::exception & error) {
    throw std::invalid_argument(
            std::string("reidentification calibration report is invalid: ") +
            error.what());
  }
}

}  // namespace track_robot_semantic_memory
