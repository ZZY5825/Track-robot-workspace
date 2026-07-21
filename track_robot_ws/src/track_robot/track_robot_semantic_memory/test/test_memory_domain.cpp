#include <stdexcept>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/memory_domain.hpp"

namespace semantic_memory = track_robot_semantic_memory;

TEST(MemoryDomain, ModeEpochAndCanonicalFrameAllDefineIsolationBoundary)
{
  const semantic_memory::MemoryDomainKey local{
    semantic_memory::MemoryMode::kLocalSession, 4U, "odom"};

  EXPECT_EQ(local, (semantic_memory::MemoryDomainKey{
    semantic_memory::MemoryMode::kLocalSession, 4U, "odom"}));
  EXPECT_NE(local, (semantic_memory::MemoryDomainKey{
    semantic_memory::MemoryMode::kWorld, 4U, "map"}));
  EXPECT_NE(local, (semantic_memory::MemoryDomainKey{
    semantic_memory::MemoryMode::kLocalSession, 5U, "odom"}));
  EXPECT_NE(local, (semantic_memory::MemoryDomainKey{
    semantic_memory::MemoryMode::kLocalSession, 4U, "local_odom"}));
}

TEST(MemoryDomain, RejectsInvalidOrUnboundedCanonicalFrame)
{
  EXPECT_THROW(
    semantic_memory::MemoryDomainKey(
      semantic_memory::MemoryMode::kLocalSession, 1U, ""),
    std::invalid_argument);
  EXPECT_THROW(
    semantic_memory::MemoryDomainKey(
      semantic_memory::MemoryMode::kLocalSession, 1U, std::string(129U, 'x')),
    std::invalid_argument);
}

TEST(MemoryDomain, DomainChangesAdvanceEpochAndRejectOldPublicKeys)
{
  semantic_memory::MemoryDomainTracker tracker(40U);
  const semantic_memory::MemoryDomainKey local{
    semantic_memory::MemoryMode::kLocalSession, 4U, "odom"};
  const semantic_memory::MemoryDomainKey world{
    semantic_memory::MemoryMode::kWorld, 5U, "map"};

  const auto initial = tracker.update(local);
  EXPECT_TRUE(initial.changed);
  EXPECT_EQ(initial.memory_epoch_id, 40U);
  EXPECT_TRUE(tracker.accepts({40U, 1U}));

  const auto unchanged = tracker.update(local);
  EXPECT_FALSE(unchanged.changed);
  EXPECT_EQ(unchanged.memory_epoch_id, 40U);

  const auto promoted = tracker.update(world);
  EXPECT_TRUE(promoted.changed);
  EXPECT_EQ(promoted.memory_epoch_id, 41U);
  EXPECT_FALSE(tracker.accepts({40U, 1U}));
  EXPECT_TRUE(tracker.accepts({41U, 1U}));
}
