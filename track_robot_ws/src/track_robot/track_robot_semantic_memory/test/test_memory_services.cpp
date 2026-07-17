#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/memory_services.hpp"

namespace semantic_memory = track_robot_semantic_memory;

namespace
{

semantic_memory::MemoryServiceRecord record(
  std::uint64_t epoch, std::uint64_t id, double relevance,
  semantic_memory::LifecycleState lifecycle =
  semantic_memory::LifecycleState::kConfirmed,
  semantic_memory::InspectionState inspection =
  semantic_memory::InspectionState::kNotInspected)
{
  return {{epoch, id}, lifecycle, inspection, relevance};
}

}  // namespace

TEST(MemoryServices, GetDistinguishesStaleEpochFromMissingObject)
{
  semantic_memory::MemoryServiceStore store(7U);
  store.upsert(record(7U, 1U, 0.8));

  EXPECT_EQ(store.get({6U, 1U}).reason, semantic_memory::ServiceReason::kStaleEpoch);
  EXPECT_EQ(store.get({7U, 2U}).reason, semantic_memory::ServiceReason::kNotFound);
  ASSERT_TRUE(store.get({7U, 1U}).record.has_value());
}

TEST(MemoryServices, QueryUsesBoundedPagesAndDeterministicTieBreaking)
{
  semantic_memory::MemoryServiceStore store(7U);
  store.upsert(record(7U, 2U, 0.9));
  store.upsert(record(7U, 3U, 0.2));
  store.upsert(record(7U, 1U, 0.9));

  auto first = store.query({2U, 0U, false, false, false, true});
  ASSERT_TRUE(first.accepted);
  ASSERT_EQ(first.records.size(), 2U);
  EXPECT_EQ(first.records[0].key.global_object_id, 1U);
  EXPECT_EQ(first.records[1].key.global_object_id, 2U);
  EXPECT_TRUE(first.has_more);

  auto second = store.query({2U, first.next_page_token, false, false, false, true});
  ASSERT_EQ(second.records.size(), 1U);
  EXPECT_EQ(second.records[0].key.global_object_id, 3U);
  EXPECT_FALSE(second.has_more);

  EXPECT_FALSE(store.query({65U, 0U, false, false, false, true}).accepted);
  EXPECT_FALSE(store.query({2U, 99U, false, false, false, true}).accepted);
}

TEST(MemoryServices, QueryAppliesLifecycleAndInspectionFilters)
{
  semantic_memory::MemoryServiceStore store(7U);
  store.upsert(record(7U, 1U, 0.9));
  store.upsert(record(7U, 2U, 0.8, semantic_memory::LifecycleState::kStale));
  store.upsert(record(7U, 3U, 0.7, semantic_memory::LifecycleState::kLost));
  store.upsert(record(7U, 4U, 0.6, semantic_memory::LifecycleState::kArchived));
  store.upsert(record(7U, 5U, 0.95, semantic_memory::LifecycleState::kConfirmed,
    semantic_memory::InspectionState::kComplete));

  const auto active_only = store.query({64U, 0U, false, false, false, false});
  ASSERT_EQ(active_only.records.size(), 1U);
  EXPECT_EQ(active_only.records[0].key.global_object_id, 1U);

  const auto all = store.query({64U, 0U, true, true, true, true});
  EXPECT_EQ(all.records.size(), 5U);
}

TEST(MemoryServices, InspectionMutationIsEventedAndIdempotent)
{
  semantic_memory::MemoryServiceStore store(7U);
  store.upsert(record(7U, 1U, 0.8));

  const auto changed = store.mark_inspected(
    {7U, 1U}, semantic_memory::InspectionState::kComplete);
  ASSERT_TRUE(changed.updated);
  ASSERT_EQ(store.events().size(), 1U);
  EXPECT_EQ(store.events()[0].type, semantic_memory::MemoryServiceEventType::kInspectionChanged);
  EXPECT_EQ(store.events()[0].key, (semantic_memory::GlobalObjectKey{7U, 1U}));

  const auto unchanged = store.mark_inspected(
    {7U, 1U}, semantic_memory::InspectionState::kComplete);
  EXPECT_TRUE(unchanged.updated);
  EXPECT_EQ(store.events().size(), 1U);
}

TEST(MemoryServices, EventHistoryKeepsOnlyTheNewestBoundedWindow)
{
  semantic_memory::MemoryServiceStore store(7U);
  store.upsert(record(7U, 1U, 0.8));

  for (std::size_t index = 0U; index < 130U; ++index) {
    const auto state = index % 2U == 0U ?
      semantic_memory::InspectionState::kRequested :
      semantic_memory::InspectionState::kComplete;
    ASSERT_TRUE(store.mark_inspected({7U, 1U}, state).updated);
  }

  ASSERT_EQ(store.events().size(), 64U);
  EXPECT_EQ(store.events().front().sequence, 67U);
  EXPECT_EQ(store.events().back().sequence, 130U);
}

TEST(MemoryServices, ResetAdvancesEpochBeforeClearingAndEventsTheNewEpoch)
{
  semantic_memory::MemoryServiceStore store(7U);
  store.upsert(record(7U, 1U, 0.8));

  const auto mismatch = store.reset(6U, true, "operator reset");
  EXPECT_FALSE(mismatch.reset);
  EXPECT_EQ(store.current_epoch(), 7U);

  const auto reset = store.reset(7U, true, std::string(300U, 'r'));
  ASSERT_TRUE(reset.reset);
  EXPECT_EQ(reset.new_epoch, 8U);
  EXPECT_EQ(store.current_epoch(), 8U);
  EXPECT_EQ(store.size(), 0U);
  EXPECT_EQ(store.get({7U, 1U}).reason, semantic_memory::ServiceReason::kStaleEpoch);
  ASSERT_EQ(store.events().size(), 1U);
  EXPECT_EQ(store.events()[0].type, semantic_memory::MemoryServiceEventType::kMemoryReset);
  EXPECT_EQ(store.events()[0].key.memory_epoch_id, 8U);
  EXPECT_LE(store.events()[0].reason.size(), 256U);
}

TEST(MemoryServices, BestCandidateFailsClosedUntilThresholdIsCalibrated)
{
  semantic_memory::MemoryServiceStore store(7U);
  store.upsert(record(7U, 2U, 0.95));
  store.upsert(record(7U, 1U, 0.95));
  store.upsert(record(7U, 3U, 0.99, semantic_memory::LifecycleState::kStale));
  store.upsert(record(7U, 4U, 0.98, semantic_memory::LifecycleState::kConfirmed,
    semantic_memory::InspectionState::kRequested));

  EXPECT_EQ(store.best_candidate({false, 0.8}).reason,
    semantic_memory::ServiceReason::kThresholdNotCalibrated);
  const auto winner = store.best_candidate({true, 0.9});
  ASSERT_TRUE(winner.record.has_value());
  EXPECT_EQ(winner.record->key.global_object_id, 1U);
  EXPECT_EQ(store.best_candidate({true, 0.96}).reason,
    semantic_memory::ServiceReason::kBelowThreshold);
}

TEST(MemoryServices, SynchronizePreservesInspectionAndPrunesMissingKeys)
{
  semantic_memory::MemoryServiceStore store(7U);
  store.synchronize(7U, {record(7U, 1U, 0.8), record(7U, 2U, 0.7)});
  ASSERT_TRUE(store.mark_inspected(
      {7U, 1U}, semantic_memory::InspectionState::kComplete).updated);

  store.synchronize(7U, {record(7U, 1U, 0.9)});

  const auto surviving = store.get({7U, 1U});
  ASSERT_TRUE(surviving.record.has_value());
  EXPECT_EQ(
    surviving.record->inspection,
    semantic_memory::InspectionState::kComplete);
  EXPECT_DOUBLE_EQ(surviving.record->task_relevance, 0.9);
  EXPECT_EQ(
    store.get({7U, 2U}).reason,
    semantic_memory::ServiceReason::kNotFound);

  store.synchronize(8U, {record(8U, 1U, 0.5)});
  const auto next_epoch = store.get({8U, 1U});
  ASSERT_TRUE(next_epoch.record.has_value());
  EXPECT_EQ(
    next_epoch.record->inspection,
    semantic_memory::InspectionState::kNotInspected);
  EXPECT_EQ(
    store.get({7U, 1U}).reason,
    semantic_memory::ServiceReason::kStaleEpoch);
}

TEST(MemoryServices, InvalidSynchronizationIsTransactionalAndBounded)
{
  semantic_memory::MemoryServiceStore store(7U);
  store.synchronize(7U, {record(7U, 1U, 0.8)});

  EXPECT_THROW(
    store.synchronize(7U, {record(7U, 2U, 0.7), record(7U, 2U, 0.6)}),
    std::invalid_argument);
  EXPECT_THROW(
    store.synchronize(7U, {record(8U, 2U, 0.7)}),
    std::invalid_argument);
  std::vector<semantic_memory::MemoryServiceRecord> oversized;
  for (std::uint64_t id = 1U; id <= 257U; ++id) {
    oversized.push_back(record(7U, id, 0.5));
  }
  EXPECT_THROW(store.synchronize(7U, oversized), std::invalid_argument);

  EXPECT_EQ(store.current_epoch(), 7U);
  EXPECT_EQ(store.size(), 1U);
  ASSERT_TRUE(store.get({7U, 1U}).record.has_value());
}

TEST(MemoryServices, IncrementalUpsertCannotExceedObjectBound)
{
  semantic_memory::MemoryServiceStore store(7U);
  for (std::uint64_t id = 1U; id <= 256U; ++id) {
    store.upsert(record(7U, id, 0.5));
  }

  EXPECT_THROW(store.upsert(record(7U, 257U, 0.5)), std::invalid_argument);
  EXPECT_EQ(store.size(), 256U);
  EXPECT_NO_THROW(store.upsert(record(7U, 256U, 0.8)));
  EXPECT_EQ(store.size(), 256U);
}
