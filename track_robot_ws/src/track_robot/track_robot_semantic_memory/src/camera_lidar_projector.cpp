#include "track_robot_semantic_memory/camera_lidar_projector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace track_robot_semantic_memory
{
namespace
{

bool finite(const Point3d & point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

bool finite(const Box2d & box) noexcept
{
  return std::isfinite(box.left) && std::isfinite(box.top) &&
         std::isfinite(box.right) && std::isfinite(box.bottom);
}

void validate_box(const Box2d & box)
{
  if (!finite(box) || box.right < box.left || box.bottom < box.top) {
    throw std::invalid_argument("2D box must be finite and ordered");
  }
}

double area(const Box2d & box) noexcept
{
  return std::max(0.0, box.right - box.left) *
         std::max(0.0, box.bottom - box.top);
}

Box2d intersection(const Box2d & first, const Box2d & second) noexcept
{
  return Box2d{
    std::max(first.left, second.left),
    std::max(first.top, second.top),
    std::min(first.right, second.right),
    std::min(first.bottom, second.bottom)};
}

Point3d apply_transform(
  const RigidTransform & transform, const Point3d & source) noexcept
{
  const auto & r = transform.rotation;
  return Point3d{
    r[0] * source.x + r[1] * source.y + r[2] * source.z +
      transform.translation.x,
    r[3] * source.x + r[4] * source.y + r[5] * source.z +
      transform.translation.y,
    r[6] * source.x + r[7] * source.y + r[8] * source.z +
      transform.translation.z};
}

Point2d project_point(const CameraModel & camera, const Point3d & point) noexcept
{
  return Point2d{
    camera.fx * point.x / point.z + camera.cx,
    camera.fy * point.y / point.z + camera.cy};
}

void validate_inputs(
  const CameraModel & camera,
  const LidarBox3d & box,
  const RigidTransform & transform)
{
  if (camera.width == 0U || camera.height == 0U ||
    !std::isfinite(camera.fx) || !std::isfinite(camera.fy) ||
    !std::isfinite(camera.cx) || !std::isfinite(camera.cy) ||
    !std::isfinite(camera.minimum_depth_m) || camera.fx <= 0.0 ||
    camera.fy <= 0.0 || camera.minimum_depth_m <= 0.0)
  {
    throw std::invalid_argument("camera calibration must be finite and positive");
  }

  if (!finite(box.centroid) || !finite(box.minimum) || !finite(box.maximum) ||
    box.minimum.x > box.maximum.x || box.minimum.y > box.maximum.y ||
    box.minimum.z > box.maximum.z)
  {
    throw std::invalid_argument("3D box must be finite and ordered");
  }

  for (const double value : transform.rotation) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("transform rotation must be finite");
    }
  }
  if (!finite(transform.translation)) {
    throw std::invalid_argument("transform translation must be finite");
  }
}

}  // namespace

double intersection_over_union(const Box2d & first, const Box2d & second)
{
  validate_box(first);
  validate_box(second);
  const double intersection_area = area(intersection(first, second));
  const double union_area = area(first) + area(second) - intersection_area;
  return union_area > 0.0 ? intersection_area / union_area : 0.0;
}

double inside_fraction(const Box2d & first, const Box2d & second)
{
  validate_box(first);
  validate_box(second);
  const double first_area = area(first);
  return first_area > 0.0 ? area(intersection(first, second)) / first_area : 0.0;
}

double center_distance_pixels(const Box2d & first, const Box2d & second)
{
  validate_box(first);
  validate_box(second);
  const double first_u = 0.5 * (first.left + first.right);
  const double first_v = 0.5 * (first.top + first.bottom);
  const double second_u = 0.5 * (second.left + second.right);
  const double second_v = 0.5 * (second.top + second.bottom);
  return std::hypot(first_u - second_u, first_v - second_v);
}

ProjectionResult CameraLidarProjector::project(
  const CameraModel & camera,
  const LidarBox3d & lidar_box,
  const TransformEvidence & transform) const
{
  validate_inputs(camera, lidar_box, transform.source_to_camera);

  ProjectionResult result;
  if (!transform.available) {
    result.status = ProjectionStatus::kTransformUnavailable;
    return result;
  }
  if (transform.query_stamp_ns != lidar_box.source_stamp_ns) {
    result.status = ProjectionStatus::kSourceStampMismatch;
    return result;
  }

  const Point3d camera_centroid = apply_transform(
    transform.source_to_camera, lidar_box.centroid);
  if (camera_centroid.z <= camera.minimum_depth_m) {
    result.status = ProjectionStatus::kBehindCamera;
    return result;
  }
  result.centroid_pixel = project_point(camera, camera_centroid);

  const std::array<Point3d, 8U> corners{
    Point3d{lidar_box.minimum.x, lidar_box.minimum.y, lidar_box.minimum.z},
    Point3d{lidar_box.minimum.x, lidar_box.minimum.y, lidar_box.maximum.z},
    Point3d{lidar_box.minimum.x, lidar_box.maximum.y, lidar_box.minimum.z},
    Point3d{lidar_box.minimum.x, lidar_box.maximum.y, lidar_box.maximum.z},
    Point3d{lidar_box.maximum.x, lidar_box.minimum.y, lidar_box.minimum.z},
    Point3d{lidar_box.maximum.x, lidar_box.minimum.y, lidar_box.maximum.z},
    Point3d{lidar_box.maximum.x, lidar_box.maximum.y, lidar_box.minimum.z},
    Point3d{lidar_box.maximum.x, lidar_box.maximum.y, lidar_box.maximum.z}};

  double left = std::numeric_limits<double>::infinity();
  double top = std::numeric_limits<double>::infinity();
  double right = -std::numeric_limits<double>::infinity();
  double bottom = -std::numeric_limits<double>::infinity();
  for (const auto & source_corner : corners) {
    const auto camera_corner = apply_transform(
      transform.source_to_camera, source_corner);
    if (camera_corner.z <= camera.minimum_depth_m) {
      continue;
    }
    const auto pixel = project_point(camera, camera_corner);
    left = std::min(left, pixel.u);
    top = std::min(top, pixel.v);
    right = std::max(right, pixel.u);
    bottom = std::max(bottom, pixel.v);
    ++result.projected_corner_count;
  }

  if (result.projected_corner_count == 0U) {
    result.status = ProjectionStatus::kNoProjectableCorners;
    return result;
  }

  result.projected_box = Box2d{left, top, right, bottom};
  const Box2d image_box{
    0.0, 0.0, static_cast<double>(camera.width),
    static_cast<double>(camera.height)};
  const Box2d overlap = intersection(result.projected_box, image_box);
  const double overlap_area = area(overlap);
  if (overlap_area <= 0.0) {
    result.status = ProjectionStatus::kOutOfFieldOfView;
    return result;
  }

  result.clipped_box = overlap;
  result.image_inside_fraction = inside_fraction(result.projected_box, image_box);
  result.status = ProjectionStatus::kVisible;
  return result;
}

}  // namespace track_robot_semantic_memory
