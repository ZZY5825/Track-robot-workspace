#include <vector>

#include "gtest/gtest.h"
#include "track_robot_safety/rotation_collision.hpp"

namespace
{

struct Point
{
  double x;
  double y;
};

constexpr double kHalfLength = 0.73;
constexpr double kHalfWidth = 0.63;
constexpr double kAngularDeceleration = 0.80;
constexpr double kResponseLatency = 0.25;
constexpr double kFixedMargin = 0.05;
constexpr double kSampleAngle = 0.005;

TEST(RotationCollision, ZeroAngularSpeedHasNoStoppingSweep)
{
  EXPECT_DOUBLE_EQ(
    track_robot_safety::rotationStopAngle(
      0.0, kAngularDeceleration, kResponseLatency, kFixedMargin),
    0.0);
}

TEST(RotationCollision, StoppingAngleUsesBrakingLatencyAndMargin)
{
  EXPECT_NEAR(
    track_robot_safety::rotationStopAngle(
      -0.026, kAngularDeceleration, kResponseLatency, kFixedMargin),
    0.0569225,
    1e-7);
  EXPECT_GT(
    track_robot_safety::rotationStopAngle(
      0.40, kAngularDeceleration, kResponseLatency, kFixedMargin),
    track_robot_safety::rotationStopAngle(
      0.026, kAngularDeceleration, kResponseLatency, kFixedMargin));
}

TEST(RotationCollision, SmallMeasuredCorrectionDoesNotSweepAFullCircle)
{
  const std::vector<Point> points{{0.315839, 0.871163}};

  const auto result = track_robot_safety::evaluateRotationCollision(
    points,
    -0.026,
    kHalfLength,
    kHalfWidth,
    kAngularDeceleration,
    kResponseLatency,
    kFixedMargin,
    kSampleAngle);

  EXPECT_FALSE(result.collision);
}

TEST(RotationCollision, ObstacleInsideFiniteStoppingSweepStillBlocks)
{
  const std::vector<Point> points{{0.75, 0.55}};

  const auto result = track_robot_safety::evaluateRotationCollision(
    points,
    -0.40,
    kHalfLength,
    kHalfWidth,
    kAngularDeceleration,
    kResponseLatency,
    kFixedMargin,
    kSampleAngle);

  EXPECT_TRUE(result.collision);
  EXPECT_LT(result.collision_angle, 0.0);
  EXPECT_GT(result.time_to_collision, 0.0);
}

}  // namespace
