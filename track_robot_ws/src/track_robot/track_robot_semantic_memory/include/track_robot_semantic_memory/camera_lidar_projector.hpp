#pragma once

#include <array>
#include <cstdint>

namespace track_robot_semantic_memory
{

struct Point2d
{
  double u{0.0};
  double v{0.0};
};

struct Point3d
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Box2d
{
  double left{0.0};
  double top{0.0};
  double right{0.0};
  double bottom{0.0};
};

struct CameraModel
{
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  double minimum_depth_m{0.05};
};

struct RigidTransform
{
  // Row-major source-to-camera rotation followed by camera-frame translation.
  std::array<double, 9U> rotation{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  Point3d translation{};
};

struct TransformEvidence
{
  bool available{false};
  std::int64_t query_stamp_ns{0};
  RigidTransform source_to_camera{};
};

struct LidarBox3d
{
  std::int64_t source_stamp_ns{0};
  Point3d centroid{};
  Point3d minimum{};
  Point3d maximum{};
};

enum class ProjectionStatus : std::uint8_t
{
  kVisible = 0U,
  kTransformUnavailable = 1U,
  kSourceStampMismatch = 2U,
  kBehindCamera = 3U,
  kOutOfFieldOfView = 4U,
  kNoProjectableCorners = 5U,
};

struct ProjectionResult
{
  ProjectionStatus status{ProjectionStatus::kTransformUnavailable};
  Point2d centroid_pixel{};
  Box2d projected_box{};
  Box2d clipped_box{};
  double image_inside_fraction{0.0};
  std::uint32_t projected_corner_count{0U};
};

[[nodiscard]] double intersection_over_union(
  const Box2d & first, const Box2d & second);

// Fraction of the first box covered by the second box.
[[nodiscard]] double inside_fraction(
  const Box2d & first, const Box2d & second);

[[nodiscard]] double center_distance_pixels(
  const Box2d & first, const Box2d & second);

class CameraLidarProjector
{
public:
  [[nodiscard]] ProjectionResult project(
    const CameraModel & camera,
    const LidarBox3d & lidar_box,
    const TransformEvidence & transform) const;
};

}  // namespace track_robot_semantic_memory
