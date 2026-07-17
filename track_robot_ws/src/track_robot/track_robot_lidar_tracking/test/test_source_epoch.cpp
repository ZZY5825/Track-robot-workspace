#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "track_robot_lidar_tracking/source_epoch.hpp"

using track_robot_lidar_tracking::SourceEpoch;

TEST(SourceEpochTest, FixedSeedIsDeterministicAndAdvancesBeforeIdReuse) {
  SourceEpoch first(41U);
  SourceEpoch second(41U);

  EXPECT_EQ(first.value(), 41U);
  EXPECT_EQ(first.advance(), 42U);
  EXPECT_EQ(second.advance(), 42U);
}

TEST(SourceEpochTest, LiveSeedIsNeverZero) {
  SourceEpoch live(0U);
  EXPECT_NE(live.value(), 0U);
}

TEST(SourceEpochTest, WrapNeverUsesZero) {
  SourceEpoch epoch(std::numeric_limits<uint64_t>::max());
  EXPECT_EQ(epoch.advance(), 1U);
}
