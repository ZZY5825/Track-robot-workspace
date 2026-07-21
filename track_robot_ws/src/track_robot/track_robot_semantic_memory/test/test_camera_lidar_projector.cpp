#include <array>
#include <cstdint>
#include <stdexcept>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/camera_lidar_projector.hpp"

namespace semantic_memory = track_robot_semantic_memory;

namespace
{

constexpr std::int64_t kSourceStampNs = 1'234'567'890LL;

semantic_memory::CameraModel test_camera()
{
  return semantic_memory::CameraModel{
    640U, 480U, 100.0, 100.0, 320.0, 240.0, 0.05};
}

semantic_memory::RigidTransform identity_transform()
{
  return semantic_memory::RigidTransform{
    std::array<double, 9U>{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0},
    semantic_memory::Point3d{0.0, 0.0, 0.0}};
}

semantic_memory::TransformEvidence available_transform()
{
  return semantic_memory::TransformEvidence{
    true, kSourceStampNs, identity_transform()};
}

semantic_memory::LidarBox3d box_at(
  const semantic_memory::Point3d & centroid,
  const semantic_memory::Point3d & minimum,
  const semantic_memory::Point3d & maximum)
{
  return semantic_memory::LidarBox3d{
    kSourceStampNs, centroid, minimum, maximum};
}

}  // namespace

TEST(CameraLidarProjector, ProjectsCentroidAndCornersWithFixedCalibration)
{
  semantic_memory::CameraLidarProjector projector;
  const auto box = box_at(
    {1.0, 0.0, 10.0}, {0.0, -1.0, 9.0}, {2.0, 1.0, 11.0});
  auto transform = available_transform();
  transform.source_to_camera.translation = {-1.0, 0.0, 0.0};

  const auto result = projector.project(test_camera(), box, transform);

  ASSERT_EQ(result.status, semantic_memory::ProjectionStatus::kVisible);
  EXPECT_DOUBLE_EQ(result.centroid_pixel.u, 320.0);
  EXPECT_DOUBLE_EQ(result.centroid_pixel.v, 240.0);
  EXPECT_NEAR(result.projected_box.left, 308.8888888889, 1e-9);
  EXPECT_NEAR(result.projected_box.top, 228.8888888889, 1e-9);
  EXPECT_NEAR(result.projected_box.right, 331.1111111111, 1e-9);
  EXPECT_NEAR(result.projected_box.bottom, 251.1111111111, 1e-9);
  EXPECT_DOUBLE_EQ(result.image_inside_fraction, 1.0);
  EXPECT_EQ(result.projected_corner_count, 8U);
}

TEST(CameraLidarProjector, RejectsCentroidBehindCamera)
{
  semantic_memory::CameraLidarProjector projector;
  const auto box = box_at(
    {0.0, 0.0, -2.0}, {-1.0, -1.0, -3.0}, {1.0, 1.0, -1.0});

  const auto result = projector.project(
    test_camera(), box, available_transform());

  EXPECT_EQ(result.status, semantic_memory::ProjectionStatus::kBehindCamera);
}

TEST(CameraLidarProjector, ReportsBoxOutsideImageFieldOfView)
{
  semantic_memory::CameraLidarProjector projector;
  const auto box = box_at(
    {100.0, 0.0, 10.0}, {99.0, -1.0, 9.0}, {101.0, 1.0, 11.0});

  const auto result = projector.project(
    test_camera(), box, available_transform());

  EXPECT_EQ(result.status, semantic_memory::ProjectionStatus::kOutOfFieldOfView);
  EXPECT_DOUBLE_EQ(result.image_inside_fraction, 0.0);
}

TEST(CameraLidarProjector, ClipsPartialProjectionAndPreservesInsideFraction)
{
  semantic_memory::CameraLidarProjector projector;
  const auto box = box_at(
    {-30.0, 0.0, 10.0}, {-35.0, -1.0, 10.0}, {-25.0, 1.0, 10.0});

  const auto result = projector.project(
    test_camera(), box, available_transform());

  ASSERT_EQ(result.status, semantic_memory::ProjectionStatus::kVisible);
  EXPECT_DOUBLE_EQ(result.projected_box.left, -30.0);
  EXPECT_DOUBLE_EQ(result.projected_box.right, 70.0);
  EXPECT_DOUBLE_EQ(result.clipped_box.left, 0.0);
  EXPECT_DOUBLE_EQ(result.clipped_box.right, 70.0);
  EXPECT_NEAR(result.image_inside_fraction, 0.7, 1e-12);
}

TEST(CameraLidarProjector, DistinguishesMissingTransformFromWrongQueryStamp)
{
  semantic_memory::CameraLidarProjector projector;
  const auto box = box_at(
    {0.0, 0.0, 10.0}, {-1.0, -1.0, 9.0}, {1.0, 1.0, 11.0});

  auto unavailable = available_transform();
  unavailable.available = false;
  EXPECT_EQ(
    projector.project(test_camera(), box, unavailable).status,
    semantic_memory::ProjectionStatus::kTransformUnavailable);

  auto wrong_stamp = available_transform();
  wrong_stamp.query_stamp_ns += 1;
  EXPECT_EQ(
    projector.project(test_camera(), box, wrong_stamp).status,
    semantic_memory::ProjectionStatus::kSourceStampMismatch);
}

TEST(CameraLidarProjector, CalculatesReusableRoiGeometryTerms)
{
  const semantic_memory::Box2d projected{10.0, 10.0, 30.0, 30.0};
  const semantic_memory::Box2d roi{20.0, 10.0, 40.0, 30.0};

  EXPECT_NEAR(semantic_memory::intersection_over_union(projected, roi), 1.0 / 3.0, 1e-12);
  EXPECT_DOUBLE_EQ(semantic_memory::inside_fraction(projected, roi), 0.5);
  EXPECT_DOUBLE_EQ(semantic_memory::center_distance_pixels(projected, roi), 10.0);
}

TEST(CameraLidarProjector, RejectsMalformedCalibrationAndNonFiniteGeometry)
{
  semantic_memory::CameraLidarProjector projector;
  auto invalid_camera = test_camera();
  invalid_camera.fx = 0.0;
  const auto box = box_at(
    {0.0, 0.0, 10.0}, {-1.0, -1.0, 9.0}, {1.0, 1.0, 11.0});

  EXPECT_THROW(
    static_cast<void>(projector.project(
      invalid_camera, box, available_transform())),
    std::invalid_argument);
}
