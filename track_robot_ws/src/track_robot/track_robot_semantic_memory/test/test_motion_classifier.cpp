#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/motion_classifier.hpp"

namespace semantic_memory = track_robot_semantic_memory;

TEST(MotionClassifier, UsesHysteresisBetweenStaticAndDynamic)
{
  const semantic_memory::MotionClassifier classifier(0.15, 0.35);

  EXPECT_EQ(classifier.classify(semantic_memory::MotionState::kUnknown, 0.10),
    semantic_memory::MotionState::kStatic);
  EXPECT_EQ(classifier.classify(semantic_memory::MotionState::kStatic, 0.25),
    semantic_memory::MotionState::kStatic);
  EXPECT_EQ(classifier.classify(semantic_memory::MotionState::kStatic, 0.40),
    semantic_memory::MotionState::kDynamic);
  EXPECT_EQ(classifier.classify(semantic_memory::MotionState::kDynamic, 0.25),
    semantic_memory::MotionState::kDynamic);
  EXPECT_EQ(classifier.classify(semantic_memory::MotionState::kDynamic, 0.10),
    semantic_memory::MotionState::kStatic);
}

TEST(MotionClassifier, RejectsInvalidThresholdsAndSpeed)
{
  EXPECT_THROW(semantic_memory::MotionClassifier(0.4, 0.2), std::invalid_argument);
  const semantic_memory::MotionClassifier classifier(0.15, 0.35);
  EXPECT_THROW((void)classifier.classify(
      semantic_memory::MotionState::kUnknown,
      std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW((void)classifier.classify(
      semantic_memory::MotionState::kUnknown, -0.1), std::invalid_argument);
}
