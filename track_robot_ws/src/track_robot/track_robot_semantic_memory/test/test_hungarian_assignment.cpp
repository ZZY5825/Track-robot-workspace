#include <algorithm>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/hungarian_assignment.hpp"

namespace semantic_memory = track_robot_semantic_memory;

TEST(HungarianAssignment, FindsGlobalOneToOneMinimumInsteadOfGreedyReuse)
{
  const auto result = semantic_memory::hungarian_assignment(
    {10U, 20U}, {100U, 200U},
    {{0.1, 0.4}, {0.2, 0.3}}, 0.5);

  ASSERT_EQ(result.matches.size(), 2U);
  EXPECT_EQ(result.matches[0].row_id, 10U);
  EXPECT_EQ(result.matches[0].column_id, 100U);
  EXPECT_EQ(result.matches[1].row_id, 20U);
  EXPECT_EQ(result.matches[1].column_id, 200U);
  EXPECT_DOUBLE_EQ(result.total_cost, 0.4);
}

TEST(HungarianAssignment, EqualCostsUseStableIdsAndIgnoreInputPermutation)
{
  const auto canonical = semantic_memory::hungarian_assignment(
    {10U, 20U}, {100U, 200U},
    {{0.1, 0.1}, {0.1, 0.1}}, 0.5);
  const auto permuted = semantic_memory::hungarian_assignment(
    {20U, 10U}, {200U, 100U},
    {{0.1, 0.1}, {0.1, 0.1}}, 0.5);

  ASSERT_EQ(canonical.matches.size(), 2U);
  ASSERT_EQ(permuted.matches.size(), 2U);
  EXPECT_EQ(canonical.matches, permuted.matches);
  EXPECT_EQ(canonical.matches[0].row_id, 10U);
  EXPECT_EQ(canonical.matches[0].column_id, 100U);
  EXPECT_EQ(canonical.matches[1].row_id, 20U);
  EXPECT_EQ(canonical.matches[1].column_id, 200U);
}

TEST(HungarianAssignment, FalsePositiveAndMissedEvidenceRemainUnmatched)
{
  const auto result = semantic_memory::hungarian_assignment(
    {10U, 20U}, {100U, 200U},
    {{0.9, std::nullopt}, {std::nullopt, 0.2}}, 0.5);

  ASSERT_EQ(result.matches.size(), 1U);
  EXPECT_EQ(result.matches[0].row_id, 20U);
  EXPECT_EQ(result.matches[0].column_id, 200U);
  EXPECT_EQ(result.unmatched_rows, (std::vector<std::uint64_t>{10U}));
  EXPECT_EQ(result.unmatched_columns, (std::vector<std::uint64_t>{100U}));
}

TEST(HungarianAssignment, SplitMergeCompetitionDoesNotDuplicateTrackIdentity)
{
  const auto result = semantic_memory::hungarian_assignment(
    {10U, 20U}, {100U}, {{0.1}, {0.2}}, 0.5);

  ASSERT_EQ(result.matches.size(), 1U);
  EXPECT_EQ(result.matches[0].row_id, 10U);
  EXPECT_EQ(result.matches[0].column_id, 100U);
  EXPECT_EQ(result.unmatched_rows, (std::vector<std::uint64_t>{20U}));
}

TEST(HungarianAssignment, RejectsDuplicateIdsAndInvalidCosts)
{
  EXPECT_THROW(
    static_cast<void>(semantic_memory::hungarian_assignment(
      {10U, 10U}, {100U}, {{0.1}, {0.2}}, 0.5)),
    std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(semantic_memory::hungarian_assignment(
      {10U}, {100U}, {{-0.1}}, 0.5)),
    std::invalid_argument);
}
