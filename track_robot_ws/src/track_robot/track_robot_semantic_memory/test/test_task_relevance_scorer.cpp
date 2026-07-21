#include <vector>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/task_relevance_scorer.hpp"

namespace semantic_memory = track_robot_semantic_memory;

namespace
{

semantic_memory::AppearanceDescriptor descriptor(
  std::vector<double> values, std::uint32_t version = 1U)
{
  return semantic_memory::AppearanceDescriptor{
    "openclip", "checkpoint-a", version,
    static_cast<std::uint16_t>(values.size()), true, std::move(values)};
}

semantic_memory::AppearancePrototype prototype(
  std::vector<double> values, std::uint32_t version = 1U)
{
  return semantic_memory::AppearancePrototype{
    descriptor(std::move(values), version), 1.0, 1.0, 1U};
}

}  // namespace

TEST(TaskRelevanceScorer, UsesMaximumCompatiblePrototypeAndBoundedPermanentSemantics)
{
  semantic_memory::TaskRelevanceScorer scorer({0.75, 0.25, 1e-5, 16U});
  const semantic_memory::SemanticTaskEvidence task{
    {41U, 3U}, descriptor({1.0, 0.0})};
  const semantic_memory::ObjectTaskEvidence object{
    {7U, 9U}, semantic_memory::LifecycleState::kConfirmed,
    {prototype({0.0, 1.0}), prototype({0.8, 0.6}),
      prototype({1.0, 0.0}, 2U)},
    {{"red crate", 0.8, 0.5, true}, {"temporary hint", 1.0, 1.0, false}}};

  const auto result = scorer.score(task, object);

  ASSERT_TRUE(result.eligible);
  EXPECT_NEAR(result.appearance_similarity, 0.8, 1e-9);
  EXPECT_NEAR(result.semantic_similarity, 0.4, 1e-9);
  EXPECT_NEAR(result.relevance, 0.7, 1e-9);
}

TEST(TaskRelevanceScorer, FailsClosedForArchivedOrDescriptorIncompatibleObjects)
{
  semantic_memory::TaskRelevanceScorer scorer({0.75, 0.25, 1e-5, 16U});
  const semantic_memory::SemanticTaskEvidence task{
    {41U, 3U}, descriptor({1.0, 0.0})};
  auto archived = semantic_memory::ObjectTaskEvidence{
    {7U, 9U}, semantic_memory::LifecycleState::kArchived,
    {prototype({1.0, 0.0})}, {}};
  auto incompatible = archived;
  incompatible.lifecycle = semantic_memory::LifecycleState::kConfirmed;
  incompatible.prototypes = {prototype({1.0, 0.0}, 2U)};

  EXPECT_FALSE(scorer.score(task, archived).eligible);
  EXPECT_FALSE(scorer.score(task, incompatible).eligible);
}

TEST(TaskRelevanceOverlay, ChangingOrClearingTaskDoesNotMutatePermanentObjectEvidence)
{
  semantic_memory::TaskRelevanceOverlay overlay({0.75, 0.25, 1e-5, 16U});
  std::vector<semantic_memory::ObjectTaskEvidence> objects{{
    {7U, 9U}, semantic_memory::LifecycleState::kConfirmed,
    {prototype({1.0, 0.0})}, {{"crate", 0.9, 0.8, true}}}};
  const auto original = objects;

  EXPECT_EQ(
    overlay.recompute({{41U, 1U}, descriptor({1.0, 0.0})}, objects), 1U);
  ASSERT_TRUE(overlay.relevance({7U, 9U}).has_value());
  EXPECT_EQ(
    overlay.recompute({{41U, 2U}, descriptor({0.0, 1.0})}, objects), 1U);
  overlay.clear();

  EXPECT_FALSE(overlay.active_task().has_value());
  EXPECT_FALSE(overlay.relevance({7U, 9U}).has_value());
  ASSERT_EQ(objects.size(), original.size());
  EXPECT_EQ(objects[0].key, original[0].key);
  EXPECT_EQ(objects[0].prototypes[0].descriptor.values,
    original[0].prototypes[0].descriptor.values);
  EXPECT_EQ(objects[0].permanent_semantics[0].label,
    original[0].permanent_semantics[0].label);
}

TEST(TaskRelevanceScorer, RejectsInvalidTaskAndExcessSemanticEvidence)
{
  semantic_memory::TaskRelevanceScorer scorer({0.75, 0.25, 1e-5, 1U});
  const semantic_memory::ObjectTaskEvidence too_many{
    {7U, 9U}, semantic_memory::LifecycleState::kConfirmed,
    {prototype({1.0, 0.0})},
    {{"one", 1.0, 1.0, true}, {"two", 1.0, 1.0, true}}};

  EXPECT_FALSE(
    scorer.score({{0U, 0U}, descriptor({1.0, 0.0})}, too_many).eligible);
  EXPECT_FALSE(
    scorer.score({{1U, 1U}, descriptor({1.0, 0.0})}, too_many).eligible);
}

TEST(TaskRelevanceScorer, RejectsToleranceThatCouldAdmitZeroVectors)
{
  EXPECT_THROW(
    semantic_memory::TaskRelevanceScorer({0.75, 0.25, 1.0, 16U}),
    std::invalid_argument);
}

TEST(TaskRelevanceOverlay, InvalidReplacementTaskClearsThePreviousOverlay)
{
  semantic_memory::TaskRelevanceOverlay overlay({0.75, 0.25, 1e-5, 16U});
  const std::vector<semantic_memory::ObjectTaskEvidence> objects{{
    {7U, 9U}, semantic_memory::LifecycleState::kConfirmed,
    {prototype({1.0, 0.0})}, {}}};
  ASSERT_EQ(
    overlay.recompute({{41U, 1U}, descriptor({1.0, 0.0})}, objects), 1U);

  auto invalid_descriptor = descriptor({1.0, 0.0});
  invalid_descriptor.l2_normalized = false;
  EXPECT_EQ(
    overlay.recompute({{41U, 2U}, invalid_descriptor}, objects), 0U);
  EXPECT_FALSE(overlay.active_task().has_value());
  EXPECT_FALSE(overlay.relevance({7U, 9U}).has_value());
}
