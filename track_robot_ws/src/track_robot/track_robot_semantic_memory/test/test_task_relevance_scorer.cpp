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

semantic_memory::TaskRelevanceConfig relevance_config(
  std::size_t maximum_semantic_evidence = 16U)
{
  semantic_memory::TaskRelevanceConfig config;
  config.appearance_weight = 0.0;
  config.semantic_weight = 0.10;
  config.normalization_tolerance = 1e-5;
  config.maximum_semantic_evidence = maximum_semantic_evidence;
  config.grounding_weight = 0.70;
  config.stability_weight = 0.10;
  config.support_weight = 0.10;
  config.maximum_grounding_age_ns = 100;
  return config;
}

}  // namespace

TEST(TaskRelevanceScorer, UsesFreshGroundingAndBoundedPermanentSemantics)
{
  semantic_memory::TaskRelevanceScorer scorer(relevance_config());
  semantic_memory::SemanticTaskEvidence task{
    {41U, 3U}, descriptor({1.0, 0.0})};
  task.producer_epoch_id = 90U;
  task.source_stamp_ns = 100;
  task.evaluation_stamp_ns = 150;
  semantic_memory::ObjectTaskEvidence object{
    {7U, 9U}, semantic_memory::LifecycleState::kConfirmed,
    {prototype({0.0, 1.0}), prototype({0.8, 0.6}),
      prototype({1.0, 0.0}, 2U)},
    {{"red crate", 0.8, 0.5, true}, {"temporary hint", 1.0, 1.0, false}},
    std::nullopt, 0.0};
  object.active_grounding =
    semantic_memory::TaskConditionedGroundingEvidence{
    {41U, 3U}, 90U, 140, 0.8, 0.75};
  object.support_quality = 0.9;

  const auto result = scorer.score(task, object);

  ASSERT_TRUE(result.eligible);
  EXPECT_DOUBLE_EQ(result.appearance_similarity, 0.0);
  EXPECT_NEAR(result.semantic_similarity, 0.4, 1e-9);
  EXPECT_NEAR(result.relevance, 0.765, 1e-9);
}

TEST(TaskRelevanceScorer, FailsClosedForArchivedOrUnsupportedObjects)
{
  semantic_memory::TaskRelevanceScorer scorer(relevance_config());
  semantic_memory::SemanticTaskEvidence task{
    {41U, 3U}, descriptor({1.0, 0.0})};
  task.producer_epoch_id = 90U;
  task.source_stamp_ns = 100;
  task.evaluation_stamp_ns = 150;
  auto archived = semantic_memory::ObjectTaskEvidence{
    {7U, 9U}, semantic_memory::LifecycleState::kArchived,
    {prototype({1.0, 0.0})}, {}, std::nullopt, 0.0};
  archived.active_grounding =
    semantic_memory::TaskConditionedGroundingEvidence{
    {41U, 3U}, 90U, 140, 0.8, 1.0};
  auto unsupported = archived;
  unsupported.lifecycle = semantic_memory::LifecycleState::kConfirmed;
  unsupported.active_grounding.reset();

  EXPECT_FALSE(scorer.score(task, archived).eligible);
  EXPECT_FALSE(scorer.score(task, unsupported).eligible);
}

TEST(TaskRelevanceOverlay, ChangingOrClearingTaskDoesNotMutatePermanentObjectEvidence)
{
  semantic_memory::TaskRelevanceOverlay overlay(relevance_config());
  std::vector<semantic_memory::ObjectTaskEvidence> objects{{
    {7U, 9U}, semantic_memory::LifecycleState::kConfirmed,
    {prototype({1.0, 0.0})}, {{"crate", 0.9, 0.8, true}},
    std::nullopt, 0.0}};
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
  semantic_memory::TaskRelevanceScorer scorer(relevance_config(1U));
  const semantic_memory::ObjectTaskEvidence too_many{
    {7U, 9U}, semantic_memory::LifecycleState::kConfirmed,
    {prototype({1.0, 0.0})},
    {{"one", 1.0, 1.0, true}, {"two", 1.0, 1.0, true}},
    std::nullopt, 0.0};

  EXPECT_FALSE(
    scorer.score({{0U, 0U}, descriptor({1.0, 0.0})}, too_many).eligible);
  EXPECT_FALSE(
    scorer.score({{1U, 1U}, descriptor({1.0, 0.0})}, too_many).eligible);
}

TEST(TaskRelevanceScorer, RejectsToleranceThatCouldAdmitZeroVectors)
{
  auto config = relevance_config();
  config.normalization_tolerance = 1.0;
  EXPECT_THROW(
    semantic_memory::TaskRelevanceScorer{config},
    std::invalid_argument);
}

TEST(TaskRelevanceOverlay, InvalidReplacementTaskClearsThePreviousOverlay)
{
  semantic_memory::TaskRelevanceOverlay overlay(relevance_config());
  const std::vector<semantic_memory::ObjectTaskEvidence> objects{{
    {7U, 9U}, semantic_memory::LifecycleState::kConfirmed,
    {prototype({1.0, 0.0})}, {{"target", 1.0, 1.0, true}},
    std::nullopt, 0.0}};
  ASSERT_EQ(
    overlay.recompute({{41U, 1U}, descriptor({1.0, 0.0})}, objects), 1U);

  auto invalid_descriptor = descriptor({1.0, 0.0});
  invalid_descriptor.l2_normalized = false;
  EXPECT_EQ(
    overlay.recompute({{41U, 2U}, invalid_descriptor}, objects), 0U);
  EXPECT_FALSE(overlay.active_task().has_value());
  EXPECT_FALSE(overlay.relevance({7U, 9U}).has_value());
}

TEST(TaskRelevanceScorer, MatchingFreshGroundingIgnoresDinoClipSpaceMismatch)
{
  semantic_memory::TaskRelevanceConfig config;
  config.grounding_weight = 0.8;
  config.stability_weight = 0.1;
  config.support_weight = 0.1;
  config.semantic_weight = 0.0;
  config.maximum_grounding_age_ns = 100;
  semantic_memory::TaskRelevanceScorer scorer(config);
  semantic_memory::SemanticTaskEvidence task{
    {41U, 3U}, descriptor({1.0, 0.0})};
  task.producer_epoch_id = 90U;
  task.source_stamp_ns = 100;
  task.evaluation_stamp_ns = 150;
  auto dino = prototype({0.0, 1.0});
  dino.descriptor.encoder_id = "dinov3:vits16plus";
  dino.descriptor.checkpoint_id = "dino.pth";
  semantic_memory::ObjectTaskEvidence object{
    {7U, 9U}, semantic_memory::LifecycleState::kConfirmed,
    {dino}, {}, std::nullopt, 0.0};
  object.active_grounding =
    semantic_memory::TaskConditionedGroundingEvidence{
    {41U, 3U}, 90U, 140, 0.8, 0.75};
  object.support_quality = 0.9;

  const auto result = scorer.score(task, object);

  EXPECT_TRUE(result.eligible);
  EXPECT_DOUBLE_EQ(result.grounding_confidence, 0.8);
  EXPECT_DOUBLE_EQ(result.stability, 0.75);
  EXPECT_DOUBLE_EQ(result.support_quality, 0.9);
  EXPECT_DOUBLE_EQ(result.appearance_similarity, 0.0);
}

TEST(TaskRelevanceScorer, WrongVersionOrStaleGroundingIsIneligible)
{
  semantic_memory::TaskRelevanceConfig config;
  config.maximum_grounding_age_ns = 10;
  semantic_memory::TaskRelevanceScorer scorer(config);
  semantic_memory::SemanticTaskEvidence task{
    {41U, 3U}, descriptor({1.0, 0.0})};
  task.producer_epoch_id = 90U;
  task.source_stamp_ns = 100;
  task.evaluation_stamp_ns = 150;
  semantic_memory::ObjectTaskEvidence object{
    {7U, 9U}, semantic_memory::LifecycleState::kConfirmed, {}, {},
    std::nullopt, 0.0};
  object.active_grounding =
    semantic_memory::TaskConditionedGroundingEvidence{
    {41U, 2U}, 90U, 145, 0.8, 1.0};
  EXPECT_FALSE(scorer.score(task, object).eligible);

  object.active_grounding->key.query_version = 3U;
  object.active_grounding->source_stamp_ns = 130;
  EXPECT_FALSE(scorer.score(task, object).eligible);
}
