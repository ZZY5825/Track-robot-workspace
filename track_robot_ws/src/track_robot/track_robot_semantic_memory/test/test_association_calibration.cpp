#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "track_robot_semantic_memory/association_calibration.hpp"

namespace semantic_memory = track_robot_semantic_memory;

namespace
{

semantic_memory::AssociationCalibrationRuntimeContract runtime_contract()
{
  semantic_memory::AssociationCalibrationRuntimeContract value;
  value.camera_calibration_id = "zed_left_rectified_v1";
  value.association.maximum_size_ratio = 40.0;
  value.association.require_position_nis = false;
  value.association.require_size_ratio = false;
  value.association.require_motion_gate = false;
  value.association.require_descriptors = false;
  value.association.weights.projected_centroid = 0.30;
  value.association.weights.inside_fraction = 0.40;
  value.association.weights.projected_iou = 0.15;
  value.association.weights.extent_consistency = 0.15;
  value.runtime.match_threshold = 0.63;
  value.runtime.confirmation.ambiguity_margin = 0.05;
  value.runtime.confirmation.confirmation_frames = 3U;
  value.runtime.confirmation.detach_after_misses = 2U;
  value.runtime.confirmation.previous_association_hysteresis = 0.02;
  value.runtime.confirmation.cooldown_frames = 2U;
  return value;
}

nlohmann::json valid_report()
{
  return {
    {"status", "calibrated"},
    {"camera_attachment_allowed", true},
    {"counts", {{"positive", 22}, {"negative", 79}}},
    {"hard_gate_pass_counts", {{"positive", 22}, {"negative", 38}}},
    {"selected_parameters", {
        {"match_threshold", 0.63},
        {"ambiguity_margin", 0.05},
        {"association_metrics", {
            {"precision", 1.0}, {"recall", 0.82}, {"threshold", 0.63}}},
        {"term_weights_from_median_separation", {
            {"projected_centroid", 0.30},
            {"inside_fraction", 0.40},
            {"projected_iou", 0.15},
            {"extent_consistency", 0.15}}}}},
    {"runtime_contract", {
        {"scoring_contract_version", "stage2d_association_v1"},
        {"camera_calibration_id", "zed_left_rectified_v1"},
        {"hard_gates", {
            {"max_source_time_delta_s", 0.1},
            {"max_evidence_age_s", 0.5},
            {"max_position_nis", 9.21},
            {"minimum_size_ratio", 0.25},
            {"maximum_size_ratio", 40.0},
            {"max_relative_speed_mps", 3.0},
            {"position_distance_max_m", 3.0},
            {"center_distance_max_px", 200.0},
            {"descriptor_normalization_tolerance", 0.0001},
            {"require_position_nis", false},
            {"require_size_ratio", false},
            {"require_motion_gate", false},
            {"require_descriptors", false}}},
        {"soft_weights", {
            {"position_consistency", 0.0},
            {"projected_centroid", 0.30},
            {"inside_fraction", 0.40},
            {"projected_iou", 0.15},
            {"visual_cosine", 0.0},
            {"extent_consistency", 0.15},
            {"point_count_consistency", 0.0},
            {"motion_continuity", 0.0},
            {"previous_association", 0.0},
            {"detector_confidence", 0.0},
            {"geometry_confidence", 0.0},
            {"sensor_confidence", 0.0}}},
        {"confirmation", {
            {"confirmation_frames", 3},
            {"detach_after_misses", 2},
            {"previous_association_hysteresis", 0.02},
            {"cooldown_frames", 2}}}}}
  };
}

}  // namespace

TEST(AssociationCalibration, CompleteMatchingRuntimeContractIsAuthorized)
{
  EXPECT_NO_THROW(semantic_memory::validate_association_calibration_report(
      valid_report(), runtime_contract()));
}

TEST(AssociationCalibration, AnyScoringOrCalibrationDriftFailsClosed)
{
  auto changed_weight = valid_report();
  changed_weight["runtime_contract"]["soft_weights"]["projected_iou"] = 0.16;
  EXPECT_THROW(semantic_memory::validate_association_calibration_report(
      changed_weight, runtime_contract()), std::invalid_argument);

  auto changed_gate = valid_report();
  changed_gate["runtime_contract"]["hard_gates"]["maximum_size_ratio"] = 4.0;
  EXPECT_THROW(semantic_memory::validate_association_calibration_report(
      changed_gate, runtime_contract()), std::invalid_argument);

  auto changed_calibration = valid_report();
  changed_calibration["runtime_contract"]["camera_calibration_id"] = "other";
  EXPECT_THROW(semantic_memory::validate_association_calibration_report(
      changed_calibration, runtime_contract()), std::invalid_argument);
}

TEST(AssociationCalibration, NonNormalizedRuntimeWeightsFailClosed)
{
  auto contract = runtime_contract();
  contract.association.weights.sensor_confidence = 0.1;
  EXPECT_THROW(semantic_memory::validate_association_calibration_report(
      valid_report(), contract), std::invalid_argument);
}
