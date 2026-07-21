#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/association_terms.hpp"

namespace semantic_memory = track_robot_semantic_memory;

namespace
{

semantic_memory::AppearanceDescriptor descriptor(
  std::vector<double> values = {0.6, 0.8})
{
  return semantic_memory::AppearanceDescriptor{
    "openclip", "checkpoint-a", 1U, 2U, true, std::move(values)};
}

}  // namespace

TEST(AssociationTerms, NormalizesHigherAndLowerEvidenceIndependently)
{
  const auto higher = semantic_memory::higher_is_better_term(
    "inside_fraction", 0.75, 0.0, 1.0, 2.0);
  EXPECT_TRUE(higher.valid);
  EXPECT_FALSE(higher.hard_gate);
  EXPECT_DOUBLE_EQ(higher.normalized_value, 0.75);
  EXPECT_DOUBLE_EQ(higher.contribution, 1.5);

  const auto lower = semantic_memory::lower_is_better_term(
    "center_distance", 25.0, 0.0, 100.0, 0.4);
  EXPECT_TRUE(lower.valid);
  EXPECT_DOUBLE_EQ(lower.normalized_value, 0.75);
  EXPECT_DOUBLE_EQ(lower.contribution, 0.3);
}

TEST(AssociationTerms, InvalidEvidenceNeverBecomesZeroValuedEvidence)
{
  const auto missing = semantic_memory::invalid_association_term(
    "projected_iou", false, "ROI unavailable");
  EXPECT_FALSE(missing.valid);
  EXPECT_FALSE(missing.gate_passed);
  EXPECT_TRUE(std::isnan(missing.raw_value));
  EXPECT_TRUE(std::isnan(missing.normalized_value));
  EXPECT_TRUE(std::isnan(missing.contribution));
  EXPECT_EQ(missing.reason, "ROI unavailable");

  const auto non_finite = semantic_memory::higher_is_better_term(
    "confidence", std::numeric_limits<double>::infinity(), 0.0, 1.0, 1.0);
  EXPECT_FALSE(non_finite.valid);
  EXPECT_TRUE(std::isnan(non_finite.contribution));
}

TEST(AssociationTerms, HardMaximumGateIsSeparateFromSoftScore)
{
  const auto passing = semantic_memory::maximum_gate(
    "source_time_delta_s", 0.04, 0.1);
  EXPECT_TRUE(passing.valid);
  EXPECT_TRUE(passing.hard_gate);
  EXPECT_TRUE(passing.gate_passed);
  EXPECT_DOUBLE_EQ(passing.contribution, 0.0);

  const auto failing = semantic_memory::maximum_gate(
    "source_time_delta_s", 0.11, 0.1);
  EXPECT_TRUE(failing.valid);
  EXPECT_FALSE(failing.gate_passed);
}

TEST(AssociationTerms, DescriptorCompatibilityChecksFullModelIdentityAndShape)
{
  const auto reference = descriptor();
  const auto compatible = descriptor({0.8, 0.6});
  const auto gate = semantic_memory::descriptor_compatibility_gate(
    reference, compatible, 1e-5);
  EXPECT_TRUE(gate.valid);
  EXPECT_TRUE(gate.gate_passed);

  auto wrong_checkpoint = compatible;
  wrong_checkpoint.checkpoint_id = "checkpoint-b";
  EXPECT_FALSE(semantic_memory::descriptor_compatibility_gate(
      reference, wrong_checkpoint, 1e-5).gate_passed);

  auto wrong_dimension = compatible;
  wrong_dimension.dimension = 3U;
  EXPECT_FALSE(semantic_memory::descriptor_compatibility_gate(
      reference, wrong_dimension, 1e-5).gate_passed);

  auto declared_but_not_normalized = compatible;
  declared_but_not_normalized.values = {1.0, 1.0};
  EXPECT_FALSE(semantic_memory::descriptor_compatibility_gate(
      reference, declared_but_not_normalized, 1e-5).gate_passed);

  auto non_finite = compatible;
  non_finite.values[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(semantic_memory::descriptor_compatibility_gate(
      reference, non_finite, 1e-5).gate_passed);
}

TEST(AssociationTerms, CompatibleDescriptorProducesCosineSoftTerm)
{
  const auto cosine = semantic_memory::descriptor_cosine_term(
    descriptor(), descriptor({0.8, 0.6}), 2.0, 1e-5);

  ASSERT_TRUE(cosine.valid);
  EXPECT_NEAR(cosine.raw_value, 0.96, 1e-12);
  EXPECT_NEAR(cosine.normalized_value, 0.98, 1e-12);
  EXPECT_NEAR(cosine.contribution, 1.96, 1e-12);
}
