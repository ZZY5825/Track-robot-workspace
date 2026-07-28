#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/memory_clock.hpp"
#include "track_robot_semantic_memory/source_time_buffer.hpp"

namespace semantic_memory = track_robot_semantic_memory;

TEST(SourceTimeBuffer, OrdersByStampThenProducerKeyRegardlessOfArrivalOrder)
{
  semantic_memory::SourceTimeBuffer<std::string> buffer(
    {4U, 1000, 200});

  EXPECT_EQ(buffer.push({200, 2U, 1}, "second"),
    semantic_memory::BufferPushResult::kInserted);
  EXPECT_EQ(buffer.push({100, 9U, 3}, "first-b"),
    semantic_memory::BufferPushResult::kInsertedOutOfOrder);
  EXPECT_EQ(buffer.push({100, 8U, 4}, "first-a"),
    semantic_memory::BufferPushResult::kInsertedOutOfOrder);

  ASSERT_EQ(buffer.entries().size(), 3U);
  EXPECT_EQ(buffer.entries()[0].value, "first-a");
  EXPECT_EQ(buffer.entries()[1].value, "first-b");
  EXPECT_EQ(buffer.entries()[2].value, "second");
}

TEST(SourceTimeBuffer, EvictsOldestByCountAndSourceAge)
{
  semantic_memory::SourceTimeBuffer<int> buffer({2U, 50, 10});

  buffer.push({100, 1U, 1}, 1);
  buffer.push({140, 1U, 2}, 2);
  buffer.push({160, 1U, 3}, 3);

  ASSERT_EQ(buffer.entries().size(), 2U);
  EXPECT_EQ(buffer.entries()[0].value, 2);
  EXPECT_EQ(buffer.entries()[1].value, 3);
  EXPECT_EQ(buffer.stats().age_evictions, 1U);

  buffer.push({170, 1U, 4}, 4);
  ASSERT_EQ(buffer.entries().size(), 2U);
  EXPECT_EQ(buffer.entries()[0].value, 3);
  EXPECT_EQ(buffer.stats().count_evictions, 1U);
}

TEST(SourceTimeBuffer, RollbackClearsOldDomainWhileSmallReorderingDoesNot)
{
  semantic_memory::SourceTimeBuffer<int> buffer({4U, 1000, 20});
  buffer.push({100, 1U, 1}, 1);
  buffer.push({200, 1U, 2}, 2);

  EXPECT_EQ(buffer.push({190, 1U, 3}, 3),
    semantic_memory::BufferPushResult::kInsertedOutOfOrder);
  EXPECT_EQ(buffer.stats().rollback_count, 0U);
  EXPECT_EQ(buffer.push({50, 1U, 4}, 4),
    semantic_memory::BufferPushResult::kInsertedAfterRollback);

  ASSERT_EQ(buffer.entries().size(), 1U);
  EXPECT_EQ(buffer.entries()[0].value, 4);
  EXPECT_EQ(buffer.stats().rollback_count, 1U);
  EXPECT_EQ(buffer.stats().rollback_drops, 3U);
}

TEST(SourceTimeBuffer, ReplacesAnExactProducerKeyDeterministically)
{
  semantic_memory::SourceTimeBuffer<int> buffer({4U, 100, 5});

  buffer.push({10, 7U, 8}, 1);
  EXPECT_EQ(buffer.push({10, 7U, 8}, 2),
    semantic_memory::BufferPushResult::kReplacedDuplicate);

  ASSERT_EQ(buffer.entries().size(), 1U);
  EXPECT_EQ(buffer.entries()[0].value, 2);
  EXPECT_EQ(buffer.stats().duplicate_replacements, 1U);
}

TEST(SourceTimeBuffer, NearestSelectsClosestEntryWithinBound)
{
  semantic_memory::SourceTimeBuffer<int> buffer({8U, 1000, 10});
  buffer.push({100, 1U, 1}, 1);
  buffer.push({160, 1U, 2}, 2);
  buffer.push({230, 1U, 3}, 3);

  const auto * nearest = buffer.nearest(175, 20);
  ASSERT_NE(nearest, nullptr);
  EXPECT_EQ(nearest->value, 2);
  EXPECT_EQ(buffer.nearest(175, 10), nullptr);
}

TEST(SourceTimeBuffer, NearestBreaksEqualDistanceByExistingKeyOrder)
{
  semantic_memory::SourceTimeBuffer<int> buffer({8U, 1000, 10});
  buffer.push({120, 2U, 4}, 2);
  buffer.push({80, 9U, 8}, 1);

  const auto * nearest = buffer.nearest(100, 20);
  ASSERT_NE(nearest, nullptr);
  EXPECT_EQ(nearest->key.source_stamp_ns, 80);
  EXPECT_EQ(nearest->value, 1);
}

TEST(SourceTimeBuffer, NearestHandlesEmptyBufferAndRejectsNegativeBounds)
{
  semantic_memory::SourceTimeBuffer<int> buffer({4U, 100, 5});
  EXPECT_EQ(buffer.nearest(10, 5), nullptr);
  EXPECT_THROW((void)buffer.nearest(-1, 5), std::invalid_argument);
  EXPECT_THROW((void)buffer.nearest(1, -1), std::invalid_argument);
}

TEST(SourceTimeBuffer, NearestCanBeRestrictedToOneProducerEpoch)
{
  semantic_memory::SourceTimeBuffer<int> buffer({8U, 1000, 10});
  buffer.push({100, 1U, 1}, 1);
  buffer.push({105, 2U, 1}, 2);

  const auto * epoch_one = buffer.nearest(105, 10, 1U);
  const auto * epoch_two = buffer.nearest(100, 10, 2U);

  ASSERT_NE(epoch_one, nullptr);
  ASSERT_NE(epoch_two, nullptr);
  EXPECT_EQ(epoch_one->value, 1);
  EXPECT_EQ(epoch_two->value, 2);
  EXPECT_EQ(buffer.nearest(100, 10, 3U), nullptr);
  EXPECT_THROW((void)buffer.nearest(100, 10, 0U), std::invalid_argument);
}

TEST(SourceTimeBuffer, ReportsNearestDeltaWithAndWithoutEpochRestriction)
{
  semantic_memory::SourceTimeBuffer<int> buffer({8U, 1000, 10});
  buffer.push({100, 1U, 1}, 1);
  buffer.push({135, 2U, 1}, 2);

  EXPECT_EQ(buffer.nearest_delta_ns(130), std::optional<std::int64_t>{5});
  EXPECT_EQ(buffer.nearest_delta_ns(130, 1U), std::optional<std::int64_t>{30});
  EXPECT_EQ(buffer.nearest_delta_ns(130, 2U), std::optional<std::int64_t>{5});
  EXPECT_EQ(buffer.nearest_delta_ns(130, 3U), std::nullopt);
  EXPECT_THROW((void)buffer.nearest_delta_ns(-1), std::invalid_argument);
  EXPECT_THROW((void)buffer.nearest_delta_ns(1, 0U), std::invalid_argument);
}
