#include <algorithm>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/runtime_association.hpp"

namespace semantic_memory = track_robot_semantic_memory;

namespace
{

semantic_memory::VisualAssociationKey visual_key(std::uint64_t id)
{
  return {
    semantic_memory::VisualAssociationKind::kUpstreamProposal, 70U, id};
}

semantic_memory::RuntimeAssociationFrame frame(
  std::uint64_t frame_index,
  std::uint64_t candidate_id,
  double score,
  bool stable = true)
{
  semantic_memory::RuntimeAssociationFrame value;
  value.frame_index = frame_index;
  value.visuals.push_back({
      candidate_id,
      stable ? std::optional<semantic_memory::VisualAssociationKey>(visual_key(5U)) :
      std::nullopt});
  value.lidars.push_back({9U, 12});
  value.pairs.push_back({candidate_id, {9U, 12}, score, true});
  return value;
}

semantic_memory::RuntimeAssociationConfig config()
{
  semantic_memory::RuntimeAssociationConfig value;
  value.match_threshold = 0.63;
  value.confirmation.confirmation_frames = 3U;
  value.confirmation.detach_after_misses = 2U;
  value.confirmation.ambiguity_margin = 0.05;
  value.confirmation.previous_association_hysteresis = 0.02;
  value.confirmation.cooldown_frames = 2U;
  return value;
}

}  // namespace

TEST(RuntimeAssociation, ChangingCandidateIdsConfirmOneStableVisualOnThirdFrame)
{
  semantic_memory::RuntimeAssociationCoordinator coordinator(config());

  const auto first = coordinator.process(frame(1U, 101U, 0.80));
  const auto second = coordinator.process(frame(2U, 202U, 0.81));
  const auto third = coordinator.process(frame(3U, 303U, 0.82));

  ASSERT_EQ(first.decisions.size(), 1U);
  ASSERT_EQ(second.decisions.size(), 1U);
  ASSERT_EQ(third.decisions.size(), 1U);
  EXPECT_EQ(first.decisions[0].decision,
    semantic_memory::ConfirmationDecision::kTentative);
  EXPECT_EQ(second.decisions[0].decision,
    semantic_memory::ConfirmationDecision::kTentative);
  EXPECT_EQ(third.decisions[0].decision,
    semantic_memory::ConfirmationDecision::kMatched);
  ASSERT_TRUE(third.decisions[0].attached_lidar.has_value());
  EXPECT_EQ(third.decisions[0].attached_lidar->tracklet_id, 12);
}

TEST(RuntimeAssociation, OneShotCandidateWithoutStableIdentityFailsClosed)
{
  semantic_memory::RuntimeAssociationCoordinator coordinator(config());

  const auto result = coordinator.process(frame(1U, 101U, 0.90, false));

  ASSERT_EQ(result.decisions.size(), 1U);
  EXPECT_EQ(result.decisions[0].decision,
    semantic_memory::ConfirmationDecision::kTentative);
  EXPECT_FALSE(result.decisions[0].attached_lidar.has_value());
  EXPECT_EQ(result.decisions[0].reason, "stable visual identity is unavailable");
}

TEST(RuntimeAssociation, GlobalAssignmentIsOneToOneAndPermutationDeterministic)
{
  semantic_memory::RuntimeAssociationFrame input;
  input.frame_index = 1U;
  input.visuals = {{20U, visual_key(2U)}, {10U, visual_key(1U)}};
  input.lidars = {{9U, 200}, {9U, 100}};
  input.pairs = {
    {20U, {9U, 200}, 0.70, true}, {20U, {9U, 100}, 0.80, true},
    {10U, {9U, 200}, 0.75, true}, {10U, {9U, 100}, 0.90, true}};
  auto permuted = input;
  std::reverse(permuted.visuals.begin(), permuted.visuals.end());
  std::reverse(permuted.lidars.begin(), permuted.lidars.end());
  std::reverse(permuted.pairs.begin(), permuted.pairs.end());

  semantic_memory::RuntimeAssociationCoordinator first(config());
  semantic_memory::RuntimeAssociationCoordinator second(config());
  const auto canonical_result = first.process(input);
  const auto permuted_result = second.process(permuted);

  ASSERT_EQ(canonical_result.decisions.size(), 2U);
  ASSERT_EQ(permuted_result.decisions.size(), 2U);
  EXPECT_EQ(canonical_result.decisions, permuted_result.decisions);
  ASSERT_TRUE(canonical_result.decisions[0].assigned_lidar.has_value());
  ASSERT_TRUE(canonical_result.decisions[1].assigned_lidar.has_value());
  EXPECT_NE(
    canonical_result.decisions[0].assigned_lidar,
    canonical_result.decisions[1].assigned_lidar);
}

TEST(RuntimeAssociation, ThresholdIsVirtualRunnerUpAndBoundaryIsAccepted)
{
  auto low_margin_config = config();
  low_margin_config.confirmation.confirmation_frames = 1U;
  semantic_memory::RuntimeAssociationCoordinator coordinator(low_margin_config);

  const auto boundary = coordinator.process(frame(1U, 1U, 0.63));
  const auto below = coordinator.process(frame(2U, 2U, 0.629));

  ASSERT_EQ(boundary.decisions.size(), 1U);
  EXPECT_EQ(boundary.decisions[0].decision,
    semantic_memory::ConfirmationDecision::kAmbiguous);
  EXPECT_DOUBLE_EQ(boundary.decisions[0].second_score, 0.63);
  ASSERT_EQ(below.decisions.size(), 1U);
  EXPECT_EQ(below.decisions[0].decision,
    semantic_memory::ConfirmationDecision::kUnmatched);
}

TEST(RuntimeAssociation, OversizedOrDuplicateFrameIsRejectedTransactionally)
{
  semantic_memory::RuntimeAssociationCoordinator coordinator(config());
  auto duplicate = frame(1U, 1U, 0.9);
  duplicate.visuals.push_back(duplicate.visuals.front());
  EXPECT_THROW(coordinator.process(duplicate), std::invalid_argument);

  auto valid = frame(1U, 1U, 0.9);
  EXPECT_NO_THROW(coordinator.process(valid));
}

TEST(RuntimeAssociation, DuplicateStableVisualKeyIsRejectedWithoutAdvancingConfirmation)
{
  semantic_memory::RuntimeAssociationCoordinator coordinator(config());
  auto duplicate = frame(1U, 101U, 0.90);
  duplicate.visuals.push_back({202U, visual_key(5U)});
  duplicate.pairs.push_back({202U, {9U, 12}, 0.80, true});

  EXPECT_THROW(coordinator.process(duplicate), std::invalid_argument);

  const auto first = coordinator.process(frame(1U, 101U, 0.90));
  const auto second = coordinator.process(frame(2U, 202U, 0.90));
  const auto third = coordinator.process(frame(3U, 303U, 0.90));

  ASSERT_EQ(first.decisions.size(), 1U);
  ASSERT_EQ(second.decisions.size(), 1U);
  ASSERT_EQ(third.decisions.size(), 1U);
  EXPECT_EQ(first.decisions[0].decision,
    semantic_memory::ConfirmationDecision::kTentative);
  EXPECT_EQ(second.decisions[0].decision,
    semantic_memory::ConfirmationDecision::kTentative);
  EXPECT_EQ(third.decisions[0].decision,
    semantic_memory::ConfirmationDecision::kMatched);
}

TEST(RuntimeAssociation, ShortlistDropsRejectedPairsAndIsDeterministic)
{
  std::vector<semantic_memory::RuntimePairCandidate> pairs{
    {10U, {9U, 30}, 0.95, false},
    {10U, {9U, 20}, 0.80, true},
    {10U, {9U, 10}, 0.80, true},
    {10U, {9U, 40}, 0.70, true},
    {11U, {9U, 50}, 0.99, true}};
  auto reversed = pairs;
  std::reverse(reversed.begin(), reversed.end());

  const auto first = semantic_memory::shortlist_visual_pairs(
    pairs, 10U, 2U);
  const auto second = semantic_memory::shortlist_visual_pairs(
    reversed, 10U, 2U);

  ASSERT_EQ(first.size(), 2U);
  EXPECT_EQ(first, second);
  EXPECT_EQ(first[0].lidar.tracklet_id, 10);
  EXPECT_EQ(first[1].lidar.tracklet_id, 20);
  EXPECT_TRUE(std::all_of(
    first.begin(), first.end(),
    [](const auto & pair) {return pair.gates_passed;}));
}

TEST(RuntimeAssociation, ShortlistRejectsInvalidBoundsAndDuplicateLidar)
{
  const std::vector<semantic_memory::RuntimePairCandidate> pairs{
    {10U, {9U, 10}, 0.80, true},
    {10U, {9U, 10}, 0.70, true}};

  EXPECT_THROW(
    semantic_memory::shortlist_visual_pairs(pairs, 10U, 0U),
    std::invalid_argument);
  EXPECT_THROW(
    semantic_memory::shortlist_visual_pairs(pairs, 10U, 2U),
    std::invalid_argument);
}
