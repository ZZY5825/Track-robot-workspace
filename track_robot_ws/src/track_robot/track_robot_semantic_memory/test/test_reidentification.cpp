#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/reidentification.hpp"

namespace semantic_memory = track_robot_semantic_memory;

namespace
{

semantic_memory::ReidentificationConfig config()
{
  return semantic_memory::ReidentificationConfig{
    3.0, 5'000'000'000LL, 0.75, 0.70, 3U};
}

semantic_memory::ReidentificationEvidence evidence(std::uint64_t frame = 1U)
{
  semantic_memory::ReidentificationEvidence value;
  value.object_key = {7U, 11U};
  value.lifecycle = semantic_memory::LifecycleState::kLost;
  value.candidate_id = 42U;
  value.frame_index = frame;
  value.domain_compatible = true;
  value.age_ns = 1'000'000'000LL;
  value.spatial_distance_m = 1.0;
  value.appearance_similarity = 0.90;
  value.geometry_similarity = 0.80;
  value.semantic_similarity = 0.70;
  return value;
}

semantic_memory::RuntimeReidentificationPair runtime_pair(
  std::uint64_t lost_id,
  std::uint64_t candidate_id,
  double score)
{
  semantic_memory::RuntimeReidentificationPair pair;
  pair.lost_key = {7U, lost_id};
  pair.candidate_key = {7U, candidate_id};
  pair.expected_candidate_lidar_key = {80U, static_cast<std::int64_t>(candidate_id)};
  pair.expected_candidate_visual_key = semantic_memory::VisualAssociationKey{
    semantic_memory::VisualAssociationKind::kCameraTrack, 90U, candidate_id};
  pair.lost_lifecycle = semantic_memory::LifecycleState::kLost;
  pair.domain_compatible = true;
  pair.age_ns = 1'000'000'000LL;
  pair.spatial_distance_m = 1.0;
  pair.appearance_similarity = score;
  pair.geometry_similarity = score;
  pair.semantic_similarity = score;
  return pair;
}

semantic_memory::RuntimeReidentificationFrame runtime_frame(std::uint64_t frame_index)
{
  semantic_memory::RuntimeReidentificationFrame frame;
  frame.frame_index = frame_index;
  frame.memory_epoch_id = 7U;
  frame.candidates = {{7U, 21U}, {7U, 22U}};
  frame.lost_targets = {{7U, 11U}, {7U, 12U}};
  frame.pairs = {
    runtime_pair(11U, 21U, 0.95),
    runtime_pair(12U, 21U, 0.75),
    runtime_pair(11U, 22U, 0.74),
    runtime_pair(12U, 22U, 0.94)};
  return frame;
}

}  // namespace

TEST(Reidentification, HardSpatialDomainAndAgeGatesRejectBeforeConfirmation)
{
  semantic_memory::ReidentificationTracker tracker(config());

  auto wrong_domain = evidence();
  wrong_domain.domain_compatible = false;
  EXPECT_EQ(tracker.update(wrong_domain).decision,
    semantic_memory::ReidentificationDecision::kRejectedGate);

  auto too_old = evidence(2U);
  too_old.age_ns = 6'000'000'000LL;
  EXPECT_EQ(tracker.update(too_old).decision,
    semantic_memory::ReidentificationDecision::kRejectedGate);

  auto too_far = evidence(3U);
  too_far.spatial_distance_m = 3.1;
  EXPECT_EQ(tracker.update(too_far).decision,
    semantic_memory::ReidentificationDecision::kRejectedGate);
}

TEST(Reidentification, AppearanceGeometrySemanticsRequireMultipleFrames)
{
  semantic_memory::ReidentificationTracker tracker(config());

  const auto first = tracker.update(evidence(1U));
  const auto second = tracker.update(evidence(2U));
  const auto third = tracker.update(evidence(3U));

  EXPECT_EQ(first.decision, semantic_memory::ReidentificationDecision::kTentative);
  EXPECT_EQ(first.consecutive_hits, 1U);
  EXPECT_FALSE(first.event_emitted);
  EXPECT_EQ(second.consecutive_hits, 2U);
  EXPECT_EQ(third.decision, semantic_memory::ReidentificationDecision::kConfirmed);
  EXPECT_TRUE(third.event_emitted);
  EXPECT_EQ(third.object_key, (semantic_memory::GlobalObjectKey{7U, 11U}));
}

TEST(Reidentification, WeakAppearanceOrCombinedEvidenceCannotConfirm)
{
  semantic_memory::ReidentificationTracker tracker(config());
  auto weak_appearance = evidence();
  weak_appearance.appearance_similarity = 0.74;
  EXPECT_EQ(tracker.update(weak_appearance).decision,
    semantic_memory::ReidentificationDecision::kRejectedScore);

  auto weak_combined = evidence(2U);
  weak_combined.geometry_similarity = 0.0;
  weak_combined.semantic_similarity = 0.0;
  EXPECT_EQ(tracker.update(weak_combined).decision,
    semantic_memory::ReidentificationDecision::kRejectedScore);
}

TEST(Reidentification, CandidateChangeOrFrameGapRestartsConfirmation)
{
  semantic_memory::ReidentificationTracker tracker(config());
  EXPECT_EQ(tracker.update(evidence(1U)).consecutive_hits, 1U);
  auto changed = evidence(2U);
  changed.candidate_id = 99U;
  EXPECT_EQ(tracker.update(changed).consecutive_hits, 1U);
  changed.frame_index = 4U;
  EXPECT_EQ(tracker.update(changed).consecutive_hits, 1U);
}

TEST(Reidentification, ArchivedObjectsAreNeverResurrectedAutomatically)
{
  semantic_memory::ReidentificationTracker tracker(config());
  auto archived = evidence();
  archived.lifecycle = semantic_memory::LifecycleState::kArchived;

  const auto result = tracker.update(archived);

  EXPECT_EQ(result.decision,
    semantic_memory::ReidentificationDecision::kArchivedBlocked);
  EXPECT_FALSE(result.event_emitted);
}

TEST(Reidentification, RejectsInvalidIdsFramesAndNonfiniteScores)
{
  semantic_memory::ReidentificationTracker tracker(config());
  auto invalid = evidence();
  invalid.object_key = {};
  EXPECT_THROW(tracker.update(invalid), std::invalid_argument);

  invalid = evidence();
  invalid.appearance_similarity = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(tracker.update(invalid), std::invalid_argument);
}

TEST(RuntimeReidentification, CompleteGlobalAssignmentIsOneToOneAndDeterministic)
{
  auto runtime_config = config();
  runtime_config.ambiguity_margin = 0.005;
  semantic_memory::RuntimeReidentificationCoordinator first(runtime_config);
  semantic_memory::RuntimeReidentificationCoordinator permuted(runtime_config);

  const auto expected = first.process(runtime_frame(1U));
  auto shuffled = runtime_frame(1U);
  std::reverse(shuffled.candidates.begin(), shuffled.candidates.end());
  std::reverse(shuffled.lost_targets.begin(), shuffled.lost_targets.end());
  std::reverse(shuffled.pairs.begin(), shuffled.pairs.end());
  const auto actual = permuted.process(shuffled);

  ASSERT_EQ(expected.decisions.size(), 2U);
  EXPECT_EQ(actual.decisions, expected.decisions);
  EXPECT_EQ(expected.decisions[0].lost_key,
    (semantic_memory::GlobalObjectKey{7U, 11U}));
  EXPECT_EQ(expected.decisions[0].candidate_key,
    (semantic_memory::GlobalObjectKey{7U, 21U}));
  EXPECT_EQ(expected.decisions[1].lost_key,
    (semantic_memory::GlobalObjectKey{7U, 12U}));
  EXPECT_EQ(expected.decisions[1].candidate_key,
    (semantic_memory::GlobalObjectKey{7U, 22U}));
}

TEST(RuntimeReidentification, SamePairRequiresThreeConsecutiveCompleteFrames)
{
  auto runtime_config = config();
  runtime_config.ambiguity_margin = 0.005;
  semantic_memory::RuntimeReidentificationCoordinator coordinator(runtime_config);

  const auto first = coordinator.process(runtime_frame(1U));
  const auto second = coordinator.process(runtime_frame(2U));
  const auto third = coordinator.process(runtime_frame(3U));

  EXPECT_EQ(first.decisions[0].decision,
    semantic_memory::ReidentificationDecision::kTentative);
  EXPECT_EQ(first.decisions[0].consecutive_hits, 1U);
  EXPECT_EQ(second.decisions[0].consecutive_hits, 2U);
  EXPECT_EQ(third.decisions[0].decision,
    semantic_memory::ReidentificationDecision::kConfirmed);
}

TEST(RuntimeReidentification, InvalidFrameIsTransactionalAndCreatesAFrameGap)
{
  auto runtime_config = config();
  runtime_config.ambiguity_margin = 0.005;
  semantic_memory::RuntimeReidentificationCoordinator coordinator(runtime_config);
  EXPECT_EQ(coordinator.process(runtime_frame(1U)).decisions[0].consecutive_hits, 1U);

  auto incomplete = runtime_frame(2U);
  incomplete.pairs.pop_back();
  EXPECT_THROW(coordinator.process(incomplete), std::invalid_argument);

  const auto after_gap = coordinator.process(runtime_frame(3U));
  EXPECT_EQ(after_gap.decisions[0].consecutive_hits, 1U);
}

TEST(RuntimeReidentification, PrunesPairsThatAreAbsentFromTheCurrentFrame)
{
  auto runtime_config = config();
  runtime_config.ambiguity_margin = 0.005;
  semantic_memory::RuntimeReidentificationCoordinator coordinator(runtime_config);

  const auto first = coordinator.process(runtime_frame(1U));
  ASSERT_EQ(first.decisions[0].consecutive_hits, 1U);
  EXPECT_EQ(coordinator.confirmation_state_count(), 2U);

  semantic_memory::RuntimeReidentificationFrame empty;
  empty.frame_index = 2U;
  empty.memory_epoch_id = 7U;
  EXPECT_TRUE(coordinator.process(empty).decisions.empty());
  EXPECT_EQ(coordinator.confirmation_state_count(), 0U);

  const auto current_again = coordinator.process(runtime_frame(3U));
  EXPECT_EQ(current_again.decisions[0].consecutive_hits, 1U);
  EXPECT_EQ(coordinator.confirmation_state_count(), 2U);
}

TEST(RuntimeReidentification, ColumnAmbiguityAndBoundsFailClosed)
{
  auto runtime_config = config();
  runtime_config.ambiguity_margin = 0.01;
  semantic_memory::RuntimeReidentificationCoordinator coordinator(runtime_config);
  semantic_memory::RuntimeReidentificationFrame ambiguous;
  ambiguous.frame_index = 1U;
  ambiguous.memory_epoch_id = 7U;
  ambiguous.candidates = {{7U, 21U}, {7U, 22U}};
  ambiguous.lost_targets = {{7U, 11U}};
  ambiguous.pairs = {
    runtime_pair(11U, 21U, 0.900), runtime_pair(11U, 22U, 0.895)};

  const auto blocked = coordinator.process(ambiguous);
  ASSERT_EQ(blocked.decisions.size(), 1U);
  EXPECT_NE(blocked.decisions[0].decision,
    semantic_memory::ReidentificationDecision::kTentative);

  auto oversized = ambiguous;
  oversized.frame_index = 2U;
  oversized.candidates.clear();
  for (std::uint64_t id = 1U; id <= 65U; ++id) {
    oversized.candidates.push_back({7U, 100U + id});
  }
  oversized.lost_targets.clear();
  oversized.pairs.clear();
  EXPECT_THROW(coordinator.process(oversized), std::invalid_argument);
}

TEST(RuntimeReidentification, RejectsDuplicateIncompleteAndNonfiniteMatrices)
{
  auto runtime_config = config();
  runtime_config.ambiguity_margin = 0.005;

  auto duplicate_candidate = runtime_frame(1U);
  duplicate_candidate.candidates[1] = duplicate_candidate.candidates[0];
  semantic_memory::RuntimeReidentificationCoordinator first(runtime_config);
  EXPECT_THROW(first.process(duplicate_candidate), std::invalid_argument);

  auto duplicate_target = runtime_frame(1U);
  duplicate_target.lost_targets[1] = duplicate_target.lost_targets[0];
  semantic_memory::RuntimeReidentificationCoordinator second(runtime_config);
  EXPECT_THROW(second.process(duplicate_target), std::invalid_argument);

  auto duplicate_pair = runtime_frame(1U);
  duplicate_pair.pairs[3] = duplicate_pair.pairs[0];
  semantic_memory::RuntimeReidentificationCoordinator third(runtime_config);
  EXPECT_THROW(third.process(duplicate_pair), std::invalid_argument);

  auto nonfinite = runtime_frame(1U);
  nonfinite.pairs[0].appearance_similarity =
    std::numeric_limits<double>::quiet_NaN();
  semantic_memory::RuntimeReidentificationCoordinator fourth(runtime_config);
  EXPECT_THROW(fourth.process(nonfinite), std::invalid_argument);

  auto archived = runtime_frame(1U);
  archived.pairs[0].lost_lifecycle = semantic_memory::LifecycleState::kArchived;
  semantic_memory::RuntimeReidentificationCoordinator fifth(runtime_config);
  EXPECT_THROW(fifth.process(archived), std::invalid_argument);

  auto inconsistent_guard = runtime_frame(1U);
  inconsistent_guard.pairs[1].expected_candidate_lidar_key = {80U, 999};
  semantic_memory::RuntimeReidentificationCoordinator sixth(runtime_config);
  EXPECT_THROW(sixth.process(inconsistent_guard), std::invalid_argument);
}

TEST(RuntimeReidentification, EnforcesEveryConfiguredCardinalityBound)
{
  auto runtime_config = config();
  semantic_memory::RuntimeReidentificationCoordinator coordinator(runtime_config);

  semantic_memory::RuntimeReidentificationFrame too_many_targets;
  too_many_targets.frame_index = 1U;
  too_many_targets.memory_epoch_id = 7U;
  for (std::uint64_t id = 1U; id <= 257U; ++id) {
    too_many_targets.lost_targets.push_back({7U, id});
  }
  EXPECT_THROW(coordinator.process(too_many_targets), std::invalid_argument);

  semantic_memory::RuntimeReidentificationFrame too_many_pairs;
  too_many_pairs.frame_index = 1U;
  too_many_pairs.memory_epoch_id = 7U;
  for (std::uint64_t id = 1U; id <= 33U; ++id) {
    too_many_pairs.candidates.push_back({7U, 1000U + id});
  }
  for (std::uint64_t id = 1U; id <= 32U; ++id) {
    too_many_pairs.lost_targets.push_back({7U, 2000U + id});
  }
  EXPECT_THROW(coordinator.process(too_many_pairs), std::invalid_argument);
}

TEST(RuntimeReidentification, ThresholdEqualityIsAcceptedButBothAxesMustBeClear)
{
  auto runtime_config = config();
  runtime_config.ambiguity_margin = 0.0;
  semantic_memory::RuntimeReidentificationCoordinator threshold(runtime_config);
  semantic_memory::RuntimeReidentificationFrame exact;
  exact.frame_index = 1U;
  exact.memory_epoch_id = 7U;
  exact.candidates = {{7U, 21U}};
  exact.lost_targets = {{7U, 11U}};
  auto exact_pair = runtime_pair(11U, 21U, 0.75);
  exact_pair.geometry_similarity = 1.0;
  exact_pair.semantic_similarity = 0.0;
  exact.pairs = {exact_pair};
  const auto accepted = threshold.process(exact);
  ASSERT_EQ(accepted.decisions.size(), 1U);
  EXPECT_DOUBLE_EQ(accepted.decisions[0].combined_score, 0.70);
  EXPECT_EQ(accepted.decisions[0].decision,
    semantic_memory::ReidentificationDecision::kTentative);

  runtime_config.ambiguity_margin = 0.01;
  semantic_memory::RuntimeReidentificationCoordinator row(runtime_config);
  semantic_memory::RuntimeReidentificationFrame ambiguous;
  ambiguous.frame_index = 1U;
  ambiguous.memory_epoch_id = 7U;
  ambiguous.candidates = {{7U, 21U}};
  ambiguous.lost_targets = {{7U, 11U}, {7U, 12U}};
  ambiguous.pairs = {
    runtime_pair(11U, 21U, 0.900), runtime_pair(12U, 21U, 0.895)};
  const auto blocked = row.process(ambiguous);
  EXPECT_TRUE(std::none_of(
    blocked.decisions.begin(), blocked.decisions.end(),
    [](const auto & decision) {
      return decision.decision ==
             semantic_memory::ReidentificationDecision::kTentative;
    }));
}

TEST(RuntimeReidentification, CurrentDescriptorMismatchClearsPriorHit)
{
  auto runtime_config = config();
  runtime_config.ambiguity_margin = 0.005;
  semantic_memory::RuntimeReidentificationCoordinator coordinator(runtime_config);
  auto matching = runtime_frame(1U);
  matching.candidates.resize(1U);
  matching.lost_targets.resize(1U);
  matching.pairs = {runtime_pair(11U, 21U, 0.95)};
  EXPECT_EQ(coordinator.process(matching).decisions[0].consecutive_hits, 1U);

  auto current_descriptor_mismatch = matching;
  current_descriptor_mismatch.frame_index = 2U;
  current_descriptor_mismatch.pairs = {runtime_pair(11U, 21U, 0.50)};
  const auto rejected = coordinator.process(current_descriptor_mismatch);
  EXPECT_EQ(rejected.decisions[0].decision,
    semantic_memory::ReidentificationDecision::kRejectedScore);
  EXPECT_EQ(coordinator.confirmation_state_count(), 0U);

  matching.frame_index = 3U;
  EXPECT_EQ(coordinator.process(matching).decisions[0].consecutive_hits, 1U);
}

TEST(RuntimeReidentification, CandidateChangeAndResetRestartConfirmation)
{
  auto runtime_config = config();
  runtime_config.ambiguity_margin = 0.005;
  semantic_memory::RuntimeReidentificationCoordinator coordinator(runtime_config);
  auto first = runtime_frame(1U);
  first.candidates.resize(1U);
  first.lost_targets.resize(1U);
  first.pairs = {runtime_pair(11U, 21U, 0.95)};
  EXPECT_EQ(coordinator.process(first).decisions[0].consecutive_hits, 1U);

  auto changed = first;
  changed.frame_index = 2U;
  changed.candidates = {{7U, 22U}};
  changed.pairs = {runtime_pair(11U, 22U, 0.95)};
  EXPECT_EQ(coordinator.process(changed).decisions[0].consecutive_hits, 1U);

  coordinator.reset();
  changed.frame_index = 1U;
  EXPECT_EQ(coordinator.process(changed).decisions[0].consecutive_hits, 1U);
}
