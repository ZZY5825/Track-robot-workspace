#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/appearance_memory.hpp"

namespace semantic_memory = track_robot_semantic_memory;

namespace
{

semantic_memory::AppearanceObservation observation(
  std::vector<double> values, double quality = 0.9)
{
  return semantic_memory::AppearanceObservation{
    semantic_memory::AppearanceDescriptor{
      "openclip", "checkpoint-a", 1U,
      static_cast<std::uint16_t>(values.size()), true, std::move(values)},
    quality, true, false, false};
}

semantic_memory::AppearanceMemoryConfig config()
{
  return semantic_memory::AppearanceMemoryConfig{4U, 0.5, 0.8, 1e-5};
}

}  // namespace

TEST(AppearanceMemory, CreatesThenConfidenceWeightedEmaRenormalizesNearestPrototype)
{
  semantic_memory::AppearanceMemory memory(config());
  EXPECT_EQ(
    memory.update(observation({1.0, 0.0}, 1.0)).decision,
    semantic_memory::AppearanceUpdateDecision::kPrototypeCreated);

  const auto updated = memory.update(observation({0.8, 0.6}, 1.0));

  EXPECT_EQ(
    updated.decision,
    semantic_memory::AppearanceUpdateDecision::kPrototypeUpdated);
  ASSERT_EQ(memory.prototypes().size(), 1U);
  EXPECT_NEAR(memory.prototypes()[0].descriptor.values[0], 0.9486832981, 1e-9);
  EXPECT_NEAR(memory.prototypes()[0].descriptor.values[1], 0.3162277660, 1e-9);
  EXPECT_EQ(memory.prototypes()[0].update_count, 2U);
}

TEST(AppearanceMemory, DiverseConfirmedViewsCreateAtMostFourPrototypes)
{
  semantic_memory::AppearanceMemory memory(config());
  EXPECT_EQ(memory.update(observation({1.0, 0.0})).decision,
    semantic_memory::AppearanceUpdateDecision::kPrototypeCreated);
  EXPECT_EQ(memory.update(observation({0.0, 1.0})).decision,
    semantic_memory::AppearanceUpdateDecision::kPrototypeCreated);
  EXPECT_EQ(memory.update(observation({-1.0, 0.0})).decision,
    semantic_memory::AppearanceUpdateDecision::kPrototypeCreated);
  EXPECT_EQ(memory.update(observation({0.0, -1.0})).decision,
    semantic_memory::AppearanceUpdateDecision::kPrototypeCreated);

  const auto fifth = memory.update(observation({std::sqrt(0.5), std::sqrt(0.5)}));

  EXPECT_EQ(memory.prototypes().size(), 4U);
  EXPECT_EQ(
    fifth.decision, semantic_memory::AppearanceUpdateDecision::kPrototypeUpdated);
}

TEST(AppearanceMemory, RetainsBestQualityViewSeparatelyFromEma)
{
  semantic_memory::AppearanceMemory memory(config());
  memory.update(observation({1.0, 0.0}, 0.6));
  memory.update(observation({0.8, 0.6}, 0.95));
  memory.update(observation({1.0, 0.0}, 0.7));

  ASSERT_TRUE(memory.best_view().has_value());
  EXPECT_DOUBLE_EQ(memory.best_view()->quality, 0.95);
  EXPECT_DOUBLE_EQ(memory.best_view()->descriptor.values[0], 0.8);
  EXPECT_DOUBLE_EQ(memory.best_view()->descriptor.values[1], 0.6);
}

TEST(AppearanceMemory, RejectsUnsafeOrIncompatibleEvidenceWithoutMutation)
{
  semantic_memory::AppearanceMemory memory(config());
  EXPECT_EQ(memory.update(observation({1.0, 0.0})).decision,
    semantic_memory::AppearanceUpdateDecision::kPrototypeCreated);
  const auto original = memory.prototypes()[0].descriptor.values;

  auto ambiguous = observation({0.8, 0.6});
  ambiguous.ambiguous = true;
  EXPECT_EQ(memory.update(ambiguous).decision,
    semantic_memory::AppearanceUpdateDecision::kRejected);

  auto predicted = observation({0.8, 0.6});
  predicted.prediction_only = true;
  EXPECT_EQ(memory.update(predicted).decision,
    semantic_memory::AppearanceUpdateDecision::kRejected);

  auto incompatible = observation({0.8, 0.6});
  incompatible.descriptor.version = 2U;
  EXPECT_EQ(memory.update(incompatible).decision,
    semantic_memory::AppearanceUpdateDecision::kRejected);

  auto nonfinite = observation({0.8, 0.6});
  nonfinite.descriptor.values[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(memory.update(nonfinite).decision,
    semantic_memory::AppearanceUpdateDecision::kRejected);

  auto zero_norm = observation({0.0, 0.0});
  EXPECT_EQ(memory.update(zero_norm).decision,
    semantic_memory::AppearanceUpdateDecision::kRejected);

  auto low_quality = observation({0.8, 0.6}, 0.49);
  EXPECT_EQ(memory.update(low_quality).decision,
    semantic_memory::AppearanceUpdateDecision::kRejected);
  EXPECT_EQ(memory.prototypes().size(), 1U);
  EXPECT_EQ(memory.prototypes()[0].descriptor.values, original);
}

TEST(AppearanceMemory, AntipodalEmaIsRejectedTransactionallyWhenBankIsFull)
{
  auto single_config = config();
  single_config.maximum_prototypes = 1U;
  semantic_memory::AppearanceMemory memory(single_config);
  memory.update(observation({1.0, 0.0}, 0.9));
  const auto original = memory.prototypes()[0];

  const auto result = memory.update(observation({-1.0, 0.0}, 0.9));

  EXPECT_EQ(result.decision, semantic_memory::AppearanceUpdateDecision::kRejected);
  EXPECT_EQ(memory.prototypes()[0].descriptor.values, original.descriptor.values);
  EXPECT_EQ(memory.prototypes()[0].update_count, original.update_count);
  ASSERT_TRUE(memory.best_view().has_value());
  EXPECT_EQ(memory.best_view()->descriptor.values, original.descriptor.values);
}

TEST(AppearanceMemory, SummaryIdIsDeterministicAndChangesWithContent)
{
  semantic_memory::AppearanceMemory first(config());
  semantic_memory::AppearanceMemory second(config());
  EXPECT_TRUE(first.summary_id().empty());

  first.update(observation({1.0, 0.0}, 0.9));
  second.update(observation({1.0, 0.0}, 0.9));

  EXPECT_EQ(first.summary_id(), second.summary_id());
  EXPECT_EQ(first.summary_id().rfind("appearance-v1-", 0U), 0U);
  EXPECT_EQ(first.summary_id().size(), 30U);

  second.update(observation({0.0, 1.0}, 0.9));
  EXPECT_NE(first.summary_id(), second.summary_id());
}

TEST(AppearanceMemory, MergeUsesCompatibilityAndFourPrototypeBound)
{
  semantic_memory::AppearanceMemory target(config());
  semantic_memory::AppearanceMemory source(config());
  target.update(observation({1.0, 0.0}));
  source.update(observation({0.0, 1.0}));

  const auto merged = target.merge_from(source);

  EXPECT_EQ(merged.accepted, 1U);
  EXPECT_EQ(merged.rejected, 0U);
  EXPECT_EQ(target.prototypes().size(), 2U);
  EXPECT_LE(target.prototypes().size(), 4U);
}

TEST(AppearanceMemory, RejectsNormalizationToleranceThatCanAdmitZeroNorm)
{
  auto invalid = config();
  invalid.normalization_tolerance = 1.0;

  EXPECT_THROW(
    (void)semantic_memory::AppearanceMemory{invalid}, std::invalid_argument);
}
