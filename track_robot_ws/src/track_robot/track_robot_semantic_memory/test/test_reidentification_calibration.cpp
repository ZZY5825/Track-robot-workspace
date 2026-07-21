#include <functional>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "track_robot_semantic_memory/reidentification_calibration.hpp"

namespace semantic_memory = track_robot_semantic_memory;
using Json = nlohmann::json;

namespace
{

semantic_memory::ReidentificationConfig config()
{
  semantic_memory::ReidentificationConfig value{
    3.0, 5'000'000'000LL, 0.75, 0.70, 3U};
  value.ambiguity_margin = 0.05;
  return value;
}

Json report()
{
  return Json{
    {"contract_version", "stage2e_reidentification_v1"},
    {"status", "calibrated"},
    {"reidentification_allowed", true},
    {"selected_parameters", {
      {"maximum_age_ns", 5'000'000'000LL},
      {"maximum_spatial_distance_m", 3.0},
      {"minimum_appearance_similarity", 0.75},
      {"minimum_combined_score", 0.70},
      {"ambiguity_margin", 0.05},
      {"confirmation_frames", 3}}}};
}

}  // namespace

TEST(ReidentificationCalibration, ExactCheckedFixturePasses)
{
  EXPECT_NO_THROW(semantic_memory::validate_reidentification_calibration_report(
      report(), {"stage2e_reidentification_v1", config()}));
}

TEST(ReidentificationCalibration, StatusAllowedAndEveryParameterDriftFailClosed)
{
  const semantic_memory::ReidentificationCalibrationExpectation expected{
    "stage2e_reidentification_v1", config()};
  const std::vector<std::function<void(Json &)>> mutations{
    [](Json & value) {value["contract_version"] = "wrong";},
    [](Json & value) {value["status"] = "uncalibrated";},
    [](Json & value) {value["reidentification_allowed"] = false;},
    [](Json & value) {value["selected_parameters"]["maximum_age_ns"] = 4;},
    [](Json & value) {value["selected_parameters"]["maximum_spatial_distance_m"] = 2.0;},
    [](Json & value) {value["selected_parameters"]["minimum_appearance_similarity"] = 0.74;},
    [](Json & value) {value["selected_parameters"]["minimum_combined_score"] = 0.69;},
    [](Json & value) {value["selected_parameters"]["ambiguity_margin"] = 0.04;},
    [](Json & value) {value["selected_parameters"]["confirmation_frames"] = 2;}};
  for (const auto & mutate : mutations) {
    auto changed = report();
    mutate(changed);
    EXPECT_THROW(
      semantic_memory::validate_reidentification_calibration_report(
        changed, expected),
      std::invalid_argument);
  }
}
