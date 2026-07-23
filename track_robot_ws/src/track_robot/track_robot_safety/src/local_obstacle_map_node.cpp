#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace {

struct Point3f
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

struct GroundPlane
{
  double a{0.0};
  double b{0.0};
  double c{0.0};
  std::size_t inliers{0};
  bool valid{false};

  double heightAt(const double x, const double y) const
  {
    return a * x + b * y + c;
  }
};

struct VoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto h1 = std::hash<int>{}(key.x);
    const auto h2 = std::hash<int>{}(key.y);
    const auto h3 = std::hash<int>{}(key.z);
    return h1 ^ (h2 << 1U) ^ (h3 << 2U);
  }
};

double rectangleClearance(
  const Point3f & point, const double half_length, const double half_width)
{
  const double dx = std::max(std::abs(static_cast<double>(point.x)) - half_length, 0.0);
  const double dy = std::max(std::abs(static_cast<double>(point.y)) - half_width, 0.0);
  return std::hypot(dx, dy);
}

}  // namespace

class LocalObstacleMapNode : public rclcpp::Node
{
public:
  LocalObstacleMapNode()
  : Node("local_obstacle_map_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/rslidar_points");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    grid_topic_ = declare_parameter<std::string>(
      "grid_topic", "/safety/local_obstacle_grid");
    filtered_cloud_topic_ = declare_parameter<std::string>(
      "filtered_cloud_topic", "/safety/filtered_obstacle_points");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/safety/obstacle_markers");
    debug_topic_ = declare_parameter<std::string>(
      "debug_topic", "/safety/obstacle_map_debug");
    lidar_qos_reliability_ = declare_parameter<std::string>(
      "lidar_qos_reliability", "reliable");

    min_range_ = declare_parameter<double>("min_range", 0.20);
    max_range_ = declare_parameter<double>("max_range", 8.0);
    ground_z_ = declare_parameter<double>("ground_z", 0.0);
    estimate_ground_plane_ = declare_parameter<bool>("estimate_ground_plane", true);
    ground_search_tolerance_ = declare_parameter<double>("ground_search_tolerance", 0.30);
    ground_inlier_tolerance_ = declare_parameter<double>("ground_inlier_tolerance", 0.06);
    ground_histogram_bin_ = declare_parameter<double>("ground_histogram_bin", 0.03);
    ground_min_range_ = declare_parameter<double>("ground_min_range", 0.8);
    ground_max_range_ = declare_parameter<double>("ground_max_range", 6.0);
    min_ground_inliers_ = declare_parameter<int>("min_ground_inliers", 100);
    max_ground_slope_ = declare_parameter<double>("max_ground_slope", 0.35);
    min_obstacle_height_ = declare_parameter<double>("min_obstacle_height", 0.08);
    max_obstacle_height_ = declare_parameter<double>("max_obstacle_height", 1.80);
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.07);
    footprint_length_ = declare_parameter<double>("footprint_length", 1.20);
    footprint_width_ = declare_parameter<double>("footprint_width", 1.00);
    self_filter_margin_ = declare_parameter<double>("self_filter_margin", 0.03);
    safety_inflation_ = declare_parameter<double>("safety_inflation", 0.20);
    grid_size_ = declare_parameter<double>("grid_size", 12.0);
    grid_resolution_ = declare_parameter<double>("grid_resolution", 0.05);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);
    allow_latest_tf_fallback_ = declare_parameter<bool>("allow_latest_tf_fallback", false);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
    if (lidar_qos_reliability_ == "best_effort" ||
      lidar_qos_reliability_ == "sensor_data")
    {
      qos.best_effort();
    } else {
      qos.reliable();
    }

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, qos,
      std::bind(&LocalObstacleMapNode::cloudCallback, this, std::placeholders::_1));
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, 5);
    filtered_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      filtered_cloud_topic_, 5);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 5);
    debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, 5);

    RCLCPP_INFO(
      get_logger(),
      "Local obstacle map: lidar=%s frame=%s footprint=%.3fx%.3f inflation=%.2f",
      lidar_topic_.c_str(), base_frame_.c_str(), footprint_length_, footprint_width_,
      safety_inflation_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto processing_start = std::chrono::steady_clock::now();
    std::vector<Point3f> raw_points;
    std::string status;
    if (!readCloud(*msg, raw_points, status)) {
      publishDebug(status, raw_points.size(), 0, GroundPlane(), Point3f(), 0.0, 0.0);
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    if (!lookupTransform(*msg, transform, status)) {
      publishDebug(status, raw_points.size(), 0, GroundPlane(), Point3f(), 0.0, 0.0);
      return;
    }

    std::vector<Point3f> transformed_points;
    transformed_points.reserve(raw_points.size());
    for (const auto & raw : raw_points) {
      transformed_points.push_back(applyTransform(raw, transform));
    }
    const GroundPlane ground = estimateGroundPlane(transformed_points);

    std::vector<Point3f> obstacles;
    obstacles.reserve(raw_points.size());
    std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels;
    const double half_length = footprint_length_ * 0.5;
    const double half_width = footprint_width_ * 0.5;
    const double leaf = std::max(voxel_leaf_size_, 0.01);
    double closest_clearance = std::numeric_limits<double>::infinity();
    Point3f closest_point;
    closest_point.x = std::numeric_limits<float>::quiet_NaN();
    closest_point.y = std::numeric_limits<float>::quiet_NaN();
    closest_point.z = std::numeric_limits<float>::quiet_NaN();

    for (const auto & point : transformed_points) {
      const double range = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
      const double point_height = static_cast<double>(point.z) -
        ground.heightAt(point.x, point.y);
      if (range < min_range_ || range > max_range_ ||
        point_height < min_obstacle_height_ || point_height > max_obstacle_height_)
      {
        continue;
      }
      if (std::abs(static_cast<double>(point.x)) <= half_length + self_filter_margin_ &&
        std::abs(static_cast<double>(point.y)) <= half_width + self_filter_margin_)
      {
        continue;
      }

      const VoxelKey key{
        static_cast<int>(std::floor(point.x / leaf)),
        static_cast<int>(std::floor(point.y / leaf)),
        static_cast<int>(std::floor(point.z / leaf))};
      if (!occupied_voxels.insert(key).second) {
        continue;
      }
      obstacles.push_back(point);
      const double clearance = rectangleClearance(point, half_length, half_width);
      if (clearance < closest_clearance) {
        closest_clearance = clearance;
        closest_point = point;
      }
    }

    publishFilteredCloud(obstacles, msg->header.stamp);
    publishGrid(obstacles, msg->header.stamp);
    publishMarkers(closest_clearance);

    const double processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - processing_start).count();
    publishDebug(
      status, raw_points.size(), obstacles.size(), ground, closest_point,
      closest_clearance, processing_ms);
  }

  GroundPlane estimateGroundPlane(const std::vector<Point3f> & points) const
  {
    GroundPlane fallback;
    fallback.c = ground_z_;
    if (!estimate_ground_plane_) {
      fallback.valid = true;
      return fallback;
    }

    const double bin_size = std::max(ground_histogram_bin_, 0.01);
    const int bin_count = std::max(
      1, static_cast<int>(std::ceil(2.0 * ground_search_tolerance_ / bin_size)));
    std::vector<int> histogram(static_cast<std::size_t>(bin_count), 0);
    for (const auto & point : points) {
      const double range = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
      if (range < ground_min_range_ || range > ground_max_range_ ||
        point.z < ground_z_ - ground_search_tolerance_ ||
        point.z > ground_z_ + ground_search_tolerance_)
      {
        continue;
      }
      const int bin = static_cast<int>(std::floor(
        (point.z - (ground_z_ - ground_search_tolerance_)) / bin_size));
      if (bin >= 0 && bin < bin_count) {
        ++histogram[static_cast<std::size_t>(bin)];
      }
    }
    const auto mode_it = std::max_element(histogram.begin(), histogram.end());
    if (mode_it == histogram.end() || *mode_it < min_ground_inliers_) {
      return fallback;
    }
    const int mode_bin = static_cast<int>(std::distance(histogram.begin(), mode_it));
    const double mode_z = ground_z_ - ground_search_tolerance_ +
      (static_cast<double>(mode_bin) + 0.5) * bin_size;

    GroundPlane plane;
    plane.c = mode_z;
    for (int iteration = 0; iteration < 2; ++iteration) {
      double normal[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
      double rhs[3] = {0.0, 0.0, 0.0};
      std::size_t count = 0;
      for (const auto & point : points) {
        const double range = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
        if (range < ground_min_range_ || range > ground_max_range_) {
          continue;
        }
        const double predicted = plane.heightAt(point.x, point.y);
        if (std::abs(static_cast<double>(point.z) - predicted) > ground_inlier_tolerance_) {
          continue;
        }
        const double values[3] = {point.x, point.y, 1.0};
        for (int row = 0; row < 3; ++row) {
          rhs[row] += values[row] * point.z;
          for (int col = 0; col < 3; ++col) {
            normal[row][col] += values[row] * values[col];
          }
        }
        ++count;
      }
      double solution[3] = {0.0, 0.0, mode_z};
      if (count < static_cast<std::size_t>(min_ground_inliers_) ||
        !solveThreeByThree(normal, rhs, solution))
      {
        return fallback;
      }
      plane.a = solution[0];
      plane.b = solution[1];
      plane.c = solution[2];
      plane.inliers = count;
    }

    if (std::hypot(plane.a, plane.b) > max_ground_slope_ ||
      std::abs(plane.c - ground_z_) > ground_search_tolerance_)
    {
      return fallback;
    }
    plane.valid = true;
    return plane;
  }

  static bool solveThreeByThree(double matrix[3][3], double rhs[3], double solution[3])
  {
    double augmented[3][4];
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        augmented[row][col] = matrix[row][col];
      }
      augmented[row][3] = rhs[row];
    }
    for (int pivot = 0; pivot < 3; ++pivot) {
      int best_row = pivot;
      for (int row = pivot + 1; row < 3; ++row) {
        if (std::abs(augmented[row][pivot]) > std::abs(augmented[best_row][pivot])) {
          best_row = row;
        }
      }
      if (std::abs(augmented[best_row][pivot]) < 1e-9) {
        return false;
      }
      for (int col = pivot; col < 4; ++col) {
        std::swap(augmented[pivot][col], augmented[best_row][col]);
      }
      const double divisor = augmented[pivot][pivot];
      for (int col = pivot; col < 4; ++col) {
        augmented[pivot][col] /= divisor;
      }
      for (int row = 0; row < 3; ++row) {
        if (row == pivot) {
          continue;
        }
        const double factor = augmented[row][pivot];
        for (int col = pivot; col < 4; ++col) {
          augmented[row][col] -= factor * augmented[pivot][col];
        }
      }
    }
    for (int row = 0; row < 3; ++row) {
      solution[row] = augmented[row][3];
    }
    return true;
  }

  bool readCloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    std::vector<Point3f> & points,
    std::string & status) const
  {
    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;
    for (const auto & field : cloud.fields) {
      if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        x_offset = static_cast<int>(field.offset);
      } else if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        y_offset = static_cast<int>(field.offset);
      } else if (field.name == "z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        z_offset = static_cast<int>(field.offset);
      }
    }
    if (x_offset < 0 || y_offset < 0 || z_offset < 0 || cloud.is_bigendian) {
      status = "invalid_xyz_cloud";
      return false;
    }

    const std::size_t point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
    points.reserve(point_count);
    for (uint32_t row = 0; row < cloud.height; ++row) {
      for (uint32_t col = 0; col < cloud.width; ++col) {
        const std::size_t offset = static_cast<std::size_t>(row) * cloud.row_step +
          static_cast<std::size_t>(col) * cloud.point_step;
        if (offset + cloud.point_step > cloud.data.size()) {
          continue;
        }
        Point3f point;
        std::memcpy(&point.x, cloud.data.data() + offset + x_offset, sizeof(float));
        std::memcpy(&point.y, cloud.data.data() + offset + y_offset, sizeof(float));
        std::memcpy(&point.z, cloud.data.data() + offset + z_offset, sizeof(float));
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
          points.push_back(point);
        }
      }
    }
    status = "ok";
    return true;
  }

  bool lookupTransform(
    const sensor_msgs::msg::PointCloud2 & cloud,
    geometry_msgs::msg::TransformStamped & transform,
    std::string & status)
  {
    const std::string source_frame = cloud.header.frame_id.empty() ? base_frame_ :
      cloud.header.frame_id;
    if (source_frame == base_frame_) {
      transform.header.frame_id = base_frame_;
      transform.child_frame_id = base_frame_;
      transform.transform.rotation.w = 1.0;
      status = "already_base_frame";
      return true;
    }

    try {
      transform = tf_buffer_.lookupTransform(
        base_frame_, source_frame, rclcpp::Time(cloud.header.stamp),
        rclcpp::Duration::from_seconds(tf_timeout_sec_));
      status = "tf_at_cloud_stamp";
      return true;
    } catch (const std::exception & exact_error) {
      if (!allow_latest_tf_fallback_) {
        status = std::string("missing_timestamped_tf:") + exact_error.what();
        return false;
      }
      try {
        transform = tf_buffer_.lookupTransform(base_frame_, source_frame, tf2::TimePointZero, 30ms);
        status = "latest_tf_fallback";
        return true;
      } catch (const std::exception & latest_error) {
        status = std::string("missing_tf:") + latest_error.what();
        return false;
      }
    }
  }

  static Point3f applyTransform(
    const Point3f & point, const geometry_msgs::msg::TransformStamped & transform)
  {
    const auto & t = transform.transform.translation;
    const auto & q = transform.transform.rotation;
    const double xx = q.x * q.x;
    const double yy = q.y * q.y;
    const double zz = q.z * q.z;
    const double xy = q.x * q.y;
    const double xz = q.x * q.z;
    const double yz = q.y * q.z;
    const double wx = q.w * q.x;
    const double wy = q.w * q.y;
    const double wz = q.w * q.z;
    Point3f output;
    output.x = static_cast<float>(
      (1.0 - 2.0 * (yy + zz)) * point.x + 2.0 * (xy - wz) * point.y +
      2.0 * (xz + wy) * point.z + t.x);
    output.y = static_cast<float>(
      2.0 * (xy + wz) * point.x + (1.0 - 2.0 * (xx + zz)) * point.y +
      2.0 * (yz - wx) * point.z + t.y);
    output.z = static_cast<float>(
      2.0 * (xz - wy) * point.x + 2.0 * (yz + wx) * point.y +
      (1.0 - 2.0 * (xx + yy)) * point.z + t.z);
    return output;
  }

  void publishFilteredCloud(
    const std::vector<Point3f> & points, const builtin_interfaces::msg::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = base_frame_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(3);
    const char * names[3] = {"x", "y", "z"};
    for (std::size_t i = 0; i < 3; ++i) {
      cloud.fields[i].name = names[i];
      cloud.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
      cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud.fields[i].count = 1;
    }
    cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
    for (std::size_t i = 0; i < points.size(); ++i) {
      std::memcpy(cloud.data.data() + i * cloud.point_step, &points[i].x, sizeof(float));
      std::memcpy(cloud.data.data() + i * cloud.point_step + 4, &points[i].y, sizeof(float));
      std::memcpy(cloud.data.data() + i * cloud.point_step + 8, &points[i].z, sizeof(float));
    }
    filtered_cloud_pub_->publish(cloud);
  }

  void publishGrid(
    const std::vector<Point3f> & points, const builtin_interfaces::msg::Time & stamp)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = base_frame_;
    grid.info.resolution = static_cast<float>(grid_resolution_);
    grid.info.width = static_cast<uint32_t>(std::ceil(grid_size_ / grid_resolution_));
    grid.info.height = grid.info.width;
    grid.info.origin.position.x = -0.5 * grid_size_;
    grid.info.origin.position.y = -0.5 * grid_size_;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<std::size_t>(grid.info.width) * grid.info.height, 0);
    for (const auto & point : points) {
      const int gx = static_cast<int>(std::floor(
        (point.x - grid.info.origin.position.x) / grid_resolution_));
      const int gy = static_cast<int>(std::floor(
        (point.y - grid.info.origin.position.y) / grid_resolution_));
      if (gx < 0 || gy < 0 || gx >= static_cast<int>(grid.info.width) ||
        gy >= static_cast<int>(grid.info.height))
      {
        continue;
      }
      grid.data[static_cast<std::size_t>(gy) * grid.info.width + gx] = 100;
    }
    grid_pub_->publish(grid);
  }

  visualization_msgs::msg::Marker rectangleMarker(
    const int id, const std::string & ns, const double half_length,
    const double half_width, const float red, const float green, const float blue) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = base_frame_;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.035;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 0.9F;
    marker.lifetime = rclcpp::Duration::from_seconds(0.25);
    const double xs[5] = {half_length, half_length, -half_length, -half_length, half_length};
    const double ys[5] = {half_width, -half_width, -half_width, half_width, half_width};
    for (int i = 0; i < 5; ++i) {
      geometry_msgs::msg::Point point;
      point.x = xs[i];
      point.y = ys[i];
      point.z = 0.08;
      marker.points.push_back(point);
    }
    return marker;
  }

  void publishMarkers(const double closest_clearance)
  {
    visualization_msgs::msg::MarkerArray markers;
    const double half_length = footprint_length_ * 0.5;
    const double half_width = footprint_width_ * 0.5;
    markers.markers.push_back(rectangleMarker(
      0, "robot_footprint", half_length, half_width, 0.2F, 0.7F, 1.0F));
    markers.markers.push_back(rectangleMarker(
      1, "inflated_footprint", half_length + safety_inflation_,
      half_width + safety_inflation_, 1.0F, 0.65F, 0.05F));

    visualization_msgs::msg::Marker text;
    text.header.stamp = now();
    text.header.frame_id = base_frame_;
    text.ns = "obstacle_clearance";
    text.id = 2;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.z = 1.2;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.16;
    text.color.r = 1.0F;
    text.color.g = 1.0F;
    text.color.b = 1.0F;
    text.color.a = 0.9F;
    text.lifetime = rclcpp::Duration::from_seconds(0.25);
    std::ostringstream label;
    label << "nearest obstacle clearance: ";
    if (std::isfinite(closest_clearance)) {
      label << closest_clearance << " m";
    } else {
      label << "none";
    }
    text.text = label.str();
    markers.markers.push_back(text);
    marker_pub_->publish(markers);
  }

  void publishDebug(
    const std::string & status, const std::size_t raw_count,
    const std::size_t obstacle_count, const GroundPlane & ground,
    const Point3f & closest_point, const double closest_clearance,
    const double processing_ms)
  {
    std_msgs::msg::String msg;
    std::ostringstream data;
    data << "{\"status\":\"" << status << "\","
         << "\"raw_points\":" << raw_count << ","
         << "\"obstacle_points\":" << obstacle_count << ","
         << "\"closest_point\":[";
    if (std::isfinite(closest_point.x)) {
      data << closest_point.x << "," << closest_point.y << "," << closest_point.z;
    } else {
      data << "null,null,null";
    }
    data << "],"
         << "\"closest_clearance\":";
    if (std::isfinite(closest_clearance)) {
      data << closest_clearance;
    } else {
      data << "null";
    }
    data << ",\"ground_plane\":{\"valid\":" << (ground.valid ? "true" : "false")
         << ",\"a\":" << ground.a << ",\"b\":" << ground.b
         << ",\"c\":" << ground.c << ",\"inliers\":" << ground.inliers << "},"
         << "\"processing_ms\":" << processing_ms << "}";
    msg.data = data.str();
    debug_pub_->publish(msg);
  }

  std::string lidar_topic_;
  std::string base_frame_;
  std::string grid_topic_;
  std::string filtered_cloud_topic_;
  std::string marker_topic_;
  std::string debug_topic_;
  std::string lidar_qos_reliability_;
  double min_range_{0.2};
  double max_range_{8.0};
  double ground_z_{0.0};
  bool estimate_ground_plane_{true};
  double ground_search_tolerance_{0.30};
  double ground_inlier_tolerance_{0.06};
  double ground_histogram_bin_{0.03};
  double ground_min_range_{0.8};
  double ground_max_range_{6.0};
  int min_ground_inliers_{100};
  double max_ground_slope_{0.35};
  double min_obstacle_height_{0.08};
  double max_obstacle_height_{1.80};
  double voxel_leaf_size_{0.07};
  double footprint_length_{1.20};
  double footprint_width_{1.00};
  double self_filter_margin_{0.03};
  double safety_inflation_{0.20};
  double grid_size_{12.0};
  double grid_resolution_{0.05};
  double tf_timeout_sec_{0.05};
  bool allow_latest_tf_fallback_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalObstacleMapNode>());
  rclcpp::shutdown();
  return 0;
}
