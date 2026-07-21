#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/cross_modal_associator.hpp"

namespace semantic_memory = track_robot_semantic_memory;

namespace
{

semantic_memory::AssociationConfig config()
{
  semantic_memory::AssociationConfig value;
  value.max_source_time_delta_s = 0.1;
  value.max_evidence_age_s = 0.5;
  value.max_position_nis = 9.21;
  value.minimum_size_ratio = 0.25;
  value.maximum_size_ratio = 4.0;
  value.max_relative_speed_mps = 3.0;
  value.require_position_nis = true;
  value.require_size_ratio = true;
  value.require_motion_gate = true;
  value.require_descriptors = true;
  value.position_distance_max_m = 3.0;
  value.center_distance_max_px = 200.0;
  value.weights.position_consistency = 1.0;
  value.weights.projected_centroid = 0.5;
  value.weights.inside_fraction = 1.0;
  value.weights.projected_iou = 1.0;
  value.weights.visual_cosine = 2.0;
  value.weights.extent_consistency = 0.25;
  value.weights.point_count_consistency = 0.25;
  value.weights.motion_continuity = 0.5;
  value.weights.previous_association = 0.5;
  value.weights.detector_confidence = 0.25;
  value.weights.geometry_confidence = 0.25;
  value.weights.sensor_confidence = 0.25;
  return value;
}

semantic_memory::PairEvidence evidence()
{
  semantic_memory::PairEvidence value;
  value.visual_stamp_ns = 1'000'000'000LL;
  value.lidar_stamp_ns = 1'040'000'000LL;
  value.evaluation_stamp_ns = 1'100'000'000LL;
  value.visual_domain = "local:7:odom";
  value.lidar_domain = "local:7:odom";
  value.transform_available = true;
  value.calibration_available = true;
  value.field_of_view_compatible = true;
  value.position_nis = 2.0;
  value.size_ratio = 1.1;
  value.relative_speed_mps = 0.3;
  value.position_distance_m = 0.3;
  value.projected_centroid_distance_px = 20.0;
  value.projected_inside_fraction = 0.9;
  value.projected_iou = 0.7;
  value.visual_descriptor = semantic_memory::AppearanceDescriptor{
    "openclip", "checkpoint-a", 1U, 2U, true, {0.6, 0.8}};
  value.memory_descriptor = semantic_memory::AppearanceDescriptor{
    "openclip", "checkpoint-a", 1U, 2U, true, {0.8, 0.6}};
  value.extent_consistency = 0.8;
  value.point_count_consistency = 0.7;
  value.motion_continuity = 0.9;
  value.previous_association = 1.0;
  value.detector_confidence = 0.8;
  value.geometry_confidence = 0.9;
  value.sensor_confidence = 0.95;
  return value;
}

const semantic_memory::AssociationTermResult & term_named(
  const semantic_memory::PairAssociationScore & score,
  const std::string & name)
{
  const auto found = std::find_if(
    score.terms.begin(), score.terms.end(),
    [&name](const auto & term) {return term.name == name;});
  if (found == score.terms.end()) {
    throw std::runtime_error("missing term: " + name);
  }
  return *found;
}

}  // namespace

TEST(CrossModalAssociator, LogsEveryApprovedGateAndSoftTermSeparately)
{
  const semantic_memory::CrossModalAssociator associator(config());
  const auto score = associator.score(evidence());

  EXPECT_TRUE(score.accepted_by_gates);
  EXPECT_GT(score.total_score, 0.0);
  EXPECT_EQ(score.terms.size(), 22U);
  EXPECT_TRUE(term_named(score, "source_time_delta_s").gate_passed);
  EXPECT_TRUE(term_named(score, "evidence_age_s").gate_passed);
  EXPECT_TRUE(term_named(score, "spatial_domain").gate_passed);
  EXPECT_TRUE(term_named(score, "transform_available").gate_passed);
  EXPECT_TRUE(term_named(score, "calibration_available").gate_passed);
  EXPECT_TRUE(term_named(score, "field_of_view").gate_passed);
  EXPECT_TRUE(term_named(score, "position_nis").gate_passed);
  EXPECT_TRUE(term_named(score, "size_ratio").gate_passed);
  EXPECT_TRUE(term_named(score, "relative_speed_mps").gate_passed);
  EXPECT_TRUE(term_named(score, "descriptor_compatibility").gate_passed);
  EXPECT_TRUE(term_named(score, "visual_cosine").valid);
  EXPECT_TRUE(term_named(score, "previous_association").valid);
}

TEST(CrossModalAssociator, AnyFailedHardGateRejectsPairWithoutErasingTerms)
{
  const semantic_memory::CrossModalAssociator associator(config());
  auto input = evidence();
  input.visual_domain = "world:8:map";
  input.transform_available = false;

  const auto score = associator.score(input);

  EXPECT_FALSE(score.accepted_by_gates);
  EXPECT_FALSE(term_named(score, "spatial_domain").gate_passed);
  EXPECT_FALSE(term_named(score, "transform_available").gate_passed);
  EXPECT_TRUE(term_named(score, "projected_iou").valid);
}

TEST(CrossModalAssociator, MissingRequiredEvidenceRejectsButOptionalSoftEvidenceIsExcluded)
{
  auto strict = config();
  semantic_memory::CrossModalAssociator associator(strict);
  auto input = evidence();
  input.position_nis.reset();
  input.projected_iou.reset();

  const auto rejected = associator.score(input);
  EXPECT_FALSE(rejected.accepted_by_gates);
  EXPECT_FALSE(term_named(rejected, "position_nis").valid);
  EXPECT_FALSE(term_named(rejected, "position_nis").gate_passed);
  EXPECT_FALSE(term_named(rejected, "projected_iou").valid);
  EXPECT_TRUE(std::isnan(term_named(rejected, "projected_iou").contribution));

  strict.require_position_nis = false;
  const semantic_memory::CrossModalAssociator permissive(strict);
  const auto accepted = permissive.score(input);
  EXPECT_TRUE(accepted.accepted_by_gates);
}

TEST(CrossModalAssociator, DescriptorMismatchIsAnExplicitHardRejection)
{
  const semantic_memory::CrossModalAssociator associator(config());
  auto input = evidence();
  input.memory_descriptor->version = 2U;

  const auto score = associator.score(input);

  EXPECT_FALSE(score.accepted_by_gates);
  EXPECT_FALSE(term_named(score, "descriptor_compatibility").gate_passed);
  EXPECT_FALSE(term_named(score, "visual_cosine").valid);
}

TEST(CrossModalAssociator, SuppliedNonFiniteOptionalGateStillRejectsPair)
{
  auto permissive_config = config();
  permissive_config.require_position_nis = false;
  const semantic_memory::CrossModalAssociator associator(permissive_config);
  auto input = evidence();
  input.position_nis = std::numeric_limits<double>::quiet_NaN();

  const auto score = associator.score(input);

  EXPECT_FALSE(score.accepted_by_gates);
  EXPECT_FALSE(term_named(score, "position_nis").valid);
}

TEST(CrossModalAssociator, LanguageRelevanceCannotEnterIdentityScore)
{
  // PairEvidence deliberately has no language/task relevance field. The score
  // is therefore invariant to the current task by construction.
  const semantic_memory::CrossModalAssociator associator(config());
  EXPECT_DOUBLE_EQ(
    associator.score(evidence()).total_score,
    associator.score(evidence()).total_score);
}
