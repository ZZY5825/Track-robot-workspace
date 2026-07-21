#pragma once

#include <string>

#include <nlohmann/json_fwd.hpp>

#include "track_robot_semantic_memory/cross_modal_associator.hpp"
#include "track_robot_semantic_memory/runtime_association.hpp"

namespace track_robot_semantic_memory
{

struct AssociationCalibrationRuntimeContract
{
  std::string scoring_contract_version{"stage2d_association_v1"};
  std::string camera_calibration_id;
  AssociationConfig association;
  RuntimeAssociationConfig runtime;
};

void validate_association_calibration_report(
  const nlohmann::json & report,
  const AssociationCalibrationRuntimeContract & runtime_contract);

}  // namespace track_robot_semantic_memory
