#pragma once

#include <string>

#include <nlohmann/json_fwd.hpp>

#include "track_robot_semantic_memory/reidentification.hpp"

namespace track_robot_semantic_memory
{

struct ReidentificationCalibrationExpectation
{
  std::string contract_version;
  ReidentificationConfig config;
};

void validate_reidentification_calibration_report(
  const nlohmann::json & report,
  const ReidentificationCalibrationExpectation & expectation);

}  // namespace track_robot_semantic_memory
