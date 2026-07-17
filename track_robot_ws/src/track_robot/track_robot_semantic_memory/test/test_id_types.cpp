#include <cstdint>
#include <set>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/id_types.hpp"

namespace semantic_memory = track_robot_semantic_memory;

TEST(IdTypes, PublicObjectKeyIncludesMemoryEpoch)
{
  const semantic_memory::GlobalObjectKey first{7U, 11U};
  const semantic_memory::GlobalObjectKey same{7U, 11U};
  const semantic_memory::GlobalObjectKey next_epoch{8U, 11U};

  EXPECT_EQ(first, same);
  EXPECT_NE(first, next_epoch);
  EXPECT_TRUE(first.valid());
  EXPECT_FALSE((semantic_memory::GlobalObjectKey{0U, 11U}).valid());
  const std::set<semantic_memory::GlobalObjectKey> keys{first, next_epoch};
  EXPECT_EQ(keys.size(), 2U);
}

TEST(IdTypes, ProducerKeyKeepsEpochSeparateFromLocalId)
{
  const semantic_memory::ProducerObjectKey old_boot{100U, 3};
  const semantic_memory::ProducerObjectKey new_boot{101U, 3};

  EXPECT_NE(old_boot, new_boot);
  EXPECT_TRUE(old_boot.valid());
  EXPECT_FALSE((semantic_memory::ProducerObjectKey{0U, 3}).valid());
  EXPECT_FALSE((semantic_memory::ProducerObjectKey{100U, -1}).valid());
}
