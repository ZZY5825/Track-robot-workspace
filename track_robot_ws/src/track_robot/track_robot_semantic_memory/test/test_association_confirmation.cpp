#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/association_confirmation.hpp"

namespace semantic_memory = track_robot_semantic_memory;

namespace
{

semantic_memory::ConfirmationConfig config()
{
  return semantic_memory::ConfirmationConfig{
    3U, 2U, 0.10, 0.05, 2U};
}

semantic_memory::LidarAssociationKey lidar(std::int64_t tracklet_id)
{
  return semantic_memory::LidarAssociationKey{9U, tracklet_id};
}

semantic_memory::VisualAssociationKey visual(std::uint64_t local_id = 42U)
{
  return {
    semantic_memory::VisualAssociationKind::kUpstreamProposal, 7U, local_id};
}

semantic_memory::ConfirmationInput candidate(
  std::uint64_t frame, std::int64_t tracklet_id,
  double best_score = 0.9, double second_score = 0.7)
{
  return semantic_memory::ConfirmationInput{
    visual(), lidar(tracklet_id), best_score, second_score, true, false, frame};
}

}  // namespace

TEST(AssociationConfirmation, NewAttachmentRequiresConsecutiveFrames)
{
  semantic_memory::AssociationConfirmation confirmation(config());

  EXPECT_EQ(
    confirmation.update(candidate(1U, 12)).decision,
    semantic_memory::ConfirmationDecision::kTentative);
  EXPECT_EQ(
    confirmation.update(candidate(2U, 12)).decision,
    semantic_memory::ConfirmationDecision::kTentative);
  const auto matched = confirmation.update(candidate(3U, 12));
  EXPECT_EQ(matched.decision, semantic_memory::ConfirmationDecision::kMatched);
  ASSERT_TRUE(matched.attached_lidar.has_value());
  EXPECT_EQ(*matched.attached_lidar, lidar(12));
}

TEST(AssociationConfirmation, AmbiguityAndSplitMergeNeverForceNewIdentity)
{
  semantic_memory::AssociationConfirmation confirmation(config());
  auto ambiguous = candidate(1U, 12, 0.80, 0.75);
  const auto first = confirmation.update(ambiguous);
  EXPECT_EQ(first.decision, semantic_memory::ConfirmationDecision::kAmbiguous);
  EXPECT_FALSE(first.attached_lidar.has_value());

  auto split_merge = candidate(2U, 12);
  split_merge.split_merge_hypothesis = true;
  const auto second = confirmation.update(split_merge);
  EXPECT_EQ(second.decision, semantic_memory::ConfirmationDecision::kAmbiguous);
  EXPECT_FALSE(second.attached_lidar.has_value());

  EXPECT_EQ(
    confirmation.update(candidate(3U, 12)).consecutive_hits, 1U);
}

TEST(AssociationConfirmation, ExistingAttachmentUsesHysteresisAndRejectsChallenger)
{
  semantic_memory::AssociationConfirmation confirmation(config());
  (void)confirmation.update(candidate(1U, 12));
  (void)confirmation.update(candidate(2U, 12));
  (void)confirmation.update(candidate(3U, 12));

  const auto retained = confirmation.update(candidate(4U, 12, 0.80, 0.74));
  EXPECT_EQ(retained.decision, semantic_memory::ConfirmationDecision::kMatched);
  EXPECT_EQ(retained.attached_lidar, lidar(12));

  const auto challenger = confirmation.update(candidate(5U, 99, 0.81, 0.80));
  EXPECT_EQ(challenger.decision, semantic_memory::ConfirmationDecision::kAmbiguous);
  EXPECT_EQ(challenger.attached_lidar, lidar(12));
}

TEST(AssociationConfirmation, MissesDetachThenEnforceCooldown)
{
  semantic_memory::AssociationConfirmation confirmation(config());
  (void)confirmation.update(candidate(1U, 12));
  (void)confirmation.update(candidate(2U, 12));
  (void)confirmation.update(candidate(3U, 12));

  auto missed = candidate(4U, 12);
  missed.assigned_lidar.reset();
  missed.gates_passed = false;
  EXPECT_EQ(
    confirmation.update(missed).decision,
    semantic_memory::ConfirmationDecision::kMatched);

  missed.frame_index = 5U;
  const auto detached = confirmation.update(missed);
  EXPECT_EQ(detached.decision, semantic_memory::ConfirmationDecision::kUnmatched);
  EXPECT_FALSE(detached.attached_lidar.has_value());

  EXPECT_EQ(
    confirmation.update(candidate(6U, 12)).decision,
    semantic_memory::ConfirmationDecision::kCooldown);
  EXPECT_EQ(
    confirmation.update(candidate(7U, 12)).decision,
    semantic_memory::ConfirmationDecision::kCooldown);
  EXPECT_EQ(
    confirmation.update(candidate(8U, 12)).decision,
    semantic_memory::ConfirmationDecision::kTentative);
}

TEST(AssociationConfirmation, ConfirmedChallengerSwitchesOnlyAfterMultipleFrames)
{
  semantic_memory::AssociationConfirmation confirmation(config());
  (void)confirmation.update(candidate(1U, 12));
  (void)confirmation.update(candidate(2U, 12));
  (void)confirmation.update(candidate(3U, 12));

  const auto first = confirmation.update(candidate(4U, 99, 0.95, 0.60));
  const auto second = confirmation.update(candidate(5U, 99, 0.95, 0.60));
  const auto switched = confirmation.update(candidate(6U, 99, 0.95, 0.60));

  EXPECT_EQ(first.decision, semantic_memory::ConfirmationDecision::kTentative);
  EXPECT_EQ(first.attached_lidar, lidar(12));
  EXPECT_EQ(second.attached_lidar, lidar(12));
  EXPECT_EQ(switched.decision, semantic_memory::ConfirmationDecision::kMatched);
  EXPECT_EQ(switched.attached_lidar, lidar(99));
}

TEST(AssociationConfirmation, RejectsNonMonotonicFramesAndNonFiniteScores)
{
  semantic_memory::AssociationConfirmation confirmation(config());
  (void)confirmation.update(candidate(2U, 12));
  EXPECT_THROW(
    static_cast<void>(confirmation.update(candidate(1U, 12))),
    std::invalid_argument);

  auto invalid = candidate(3U, 12);
  invalid.best_score = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    static_cast<void>(confirmation.update(invalid)), std::invalid_argument);
}

TEST(AssociationConfirmation, InvalidInputDoesNotCreateOrAdvanceState)
{
  semantic_memory::AssociationConfirmation confirmation(config());
  auto invalid = candidate(0U, 12);
  EXPECT_THROW(
    static_cast<void>(confirmation.update(invalid)), std::invalid_argument);
  invalid = candidate(1U, 12);
  invalid.best_score = 1.01;
  EXPECT_THROW(
    static_cast<void>(confirmation.update(invalid)), std::invalid_argument);
  invalid = candidate(1U, 12);
  invalid.assigned_lidar = semantic_memory::LidarAssociationKey{0U, -1};
  EXPECT_THROW(
    static_cast<void>(confirmation.update(invalid)), std::invalid_argument);

  const auto valid = confirmation.update(candidate(1U, 12));
  EXPECT_EQ(valid.decision, semantic_memory::ConfirmationDecision::kTentative);
  EXPECT_EQ(valid.consecutive_hits, 1U);
}

TEST(AssociationConfirmation, StableCompositeIdentityIsolatedByProducerEpoch)
{
  semantic_memory::AssociationConfirmation confirmation(config());
  auto first = candidate(1U, 12);
  first.visual_key = visual(99U);
  EXPECT_EQ(
    confirmation.update(first).decision,
    semantic_memory::ConfirmationDecision::kTentative);

  auto other_epoch = candidate(2U, 12);
  other_epoch.visual_key = visual(99U);
  other_epoch.visual_key.producer_epoch_id = 8U;
  const auto isolated = confirmation.update(other_epoch);

  EXPECT_EQ(isolated.decision, semantic_memory::ConfirmationDecision::kTentative);
  EXPECT_EQ(isolated.consecutive_hits, 1U);
}

TEST(AssociationConfirmation, BoundedStateEvictsOldestIdentityDeterministically)
{
  auto bounded = config();
  bounded.maximum_states = 2U;
  semantic_memory::AssociationConfirmation confirmation(bounded);
  auto first = candidate(1U, 12);
  first.visual_key = visual(1U);
  auto second = candidate(2U, 12);
  second.visual_key = visual(2U);
  auto third = candidate(3U, 12);
  third.visual_key = visual(3U);
  (void)confirmation.update(first);
  (void)confirmation.update(second);
  (void)confirmation.update(third);

  first.frame_index = 4U;
  const auto restarted = confirmation.update(first);

  EXPECT_EQ(restarted.decision, semantic_memory::ConfirmationDecision::kTentative);
  EXPECT_EQ(restarted.consecutive_hits, 1U);
}
