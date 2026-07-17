#include <gtest/gtest.h>

#include "track_robot_semantic_memory/lifecycle_policy.hpp"

namespace semantic_memory = track_robot_semantic_memory;

TEST(LifecyclePolicy, RequiresRepeatedEvidenceBeforeConfirmation)
{
  const semantic_memory::LifecyclePolicy policy({3U, 100, 300, 1000});

  EXPECT_EQ(policy.evaluate(
      semantic_memory::LifecycleState::kTentative, 2U, 0),
    semantic_memory::LifecycleState::kTentative);
  EXPECT_EQ(policy.evaluate(
      semantic_memory::LifecycleState::kTentative, 3U, 0),
    semantic_memory::LifecycleState::kConfirmed);
}

TEST(LifecyclePolicy, LossStagesAreDeterministicAndRecoveryIsConservative)
{
  const semantic_memory::LifecyclePolicy policy({3U, 100, 300, 1000});

  EXPECT_EQ(policy.evaluate(
      semantic_memory::LifecycleState::kConfirmed, 3U, 101),
    semantic_memory::LifecycleState::kStale);
  EXPECT_EQ(policy.evaluate(
      semantic_memory::LifecycleState::kStale, 3U, 301),
    semantic_memory::LifecycleState::kLost);
  EXPECT_EQ(policy.evaluate(
      semantic_memory::LifecycleState::kLost, 3U, 1001),
    semantic_memory::LifecycleState::kArchived);
  EXPECT_EQ(policy.evaluate(
      semantic_memory::LifecycleState::kStale, 3U, 0),
    semantic_memory::LifecycleState::kConfirmed);
  EXPECT_EQ(policy.evaluate(
      semantic_memory::LifecycleState::kLost, 3U, 0),
    semantic_memory::LifecycleState::kConfirmed);
  EXPECT_EQ(policy.evaluate(
      semantic_memory::LifecycleState::kArchived, 10U, 0),
    semantic_memory::LifecycleState::kArchived);
}

TEST(LifecyclePolicy, SupportStateDoesNotOverwriteLifecycle)
{
  const semantic_memory::LifecyclePolicy policy({2U, 100, 300, 1000});

  EXPECT_EQ(policy.freshness(0), semantic_memory::EvidenceFreshness::kObserved);
  EXPECT_EQ(policy.freshness(50), semantic_memory::EvidenceFreshness::kPredicted);
  EXPECT_EQ(policy.freshness(301), semantic_memory::EvidenceFreshness::kUnsupported);
}
