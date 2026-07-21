#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <track_robot_interfaces/msg/lidar_cluster.hpp>
#include <track_robot_interfaces/msg/lidar_cluster_array.hpp>
#include <track_robot_interfaces/msg/lidar_tracklet.hpp>
#include <track_robot_interfaces/msg/lidar_tracklet_array.hpp>
#include <track_robot_interfaces/msg/semantic_lidar_tracklet_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "track_robot_lidar_tracking/source_epoch.hpp"

using namespace std::chrono_literals;

namespace {

struct PointXYZI {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
};

struct ClusterDetection {
  std::vector<int> indices;
  PointXYZI centroid;
  PointXYZI minimum;
  PointXYZI maximum;
  double distance{0.0};
  double bearing{0.0};
  double dynamic_score{0.0};
  double observation_quality{0.0};
  double measurement_variance{0.25};
  bool oversized{false};
  bool too_sparse{false};
  bool too_large{false};
  bool near_static{false};
  bool tracklet_eligible{false};
};

struct Tracklet {
  int id{-1};
  PointXYZI position;
  PointXYZI velocity;
  Eigen::Vector4d state{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d covariance{Eigen::Matrix4d::Identity()};
  PointXYZI minimum;
  PointXYZI maximum;
  PointXYZI last_detection_position;
  size_t point_count{0};
  double confidence{0.0};
  double dynamic_score{0.0};
  double observation_quality{0.0};
  uint32_t age_frames{0};
  uint32_t hit_count{0};
  uint32_t miss_count{0};
  uint32_t stationary_count{0};
  rclcpp::Time last_update;
  rclcpp::Time last_seen;
  bool confirmed{false};
};

struct VoxelKey {
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &key) const {
    const auto h1 = std::hash<int>{}(key.x * 73856093);
    const auto h2 = std::hash<int>{}(key.y * 19349663);
    const auto h3 = std::hash<int>{}(key.z * 83492791);
    return h1 ^ h2 ^ h3;
  }
};

struct GridKey {
  int x{0};
  int y{0};

  bool operator==(const GridKey &other) const {
    return x == other.x && y == other.y;
  }
};

struct GridKeyHash {
  std::size_t operator()(const GridKey &key) const {
    const auto h1 = std::hash<int>{}(key.x * 73856093);
    const auto h2 = std::hash<int>{}(key.y * 19349663);
    return h1 ^ h2;
  }
};

geometry_msgs::msg::Point toPoint(const PointXYZI &point) {
  geometry_msgs::msg::Point msg;
  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;
  return msg;
}

geometry_msgs::msg::Vector3 toVector(const PointXYZI &point) {
  geometry_msgs::msg::Vector3 msg;
  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;
  return msg;
}

double distanceXY(const PointXYZI &a, const PointXYZI &b) {
  return std::hypot(static_cast<double>(a.x - b.x), static_cast<double>(a.y - b.y));
}

}  // namespace

class LidarTrackletManagerNode : public rclcpp::Node {
 public:
  LidarTrackletManagerNode()
  : Node("lidar_tracklet_manager_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_) {
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/rslidar_points");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/human_tracking/lidar_tracklets");
    semantic_output_topic_ = declare_parameter<std::string>(
      "semantic_output_topic", "");
    const int64_t source_epoch_seed = declare_parameter<int64_t>("source_epoch_seed", 0);
    if (source_epoch_seed < 0) {
      throw std::invalid_argument("source_epoch_seed must be non-negative");
    }
    source_epoch_ = std::make_unique<track_robot_lidar_tracking::SourceEpoch>(
      static_cast<uint64_t>(source_epoch_seed));
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/human_tracking/lidar_tracklet_markers");
    candidate_clusters_topic_ = declare_parameter<std::string>(
      "candidate_clusters_topic", "/human_tracking/lidar_candidate_clusters");
    candidate_marker_topic_ = declare_parameter<std::string>(
      "candidate_marker_topic", "/human_tracking/lidar_candidate_cluster_markers");
    debug_topic_ = declare_parameter<std::string>(
      "debug_topic", "/human_tracking/lidar_tracklet_debug");
    lidar_qos_reliability_ = declare_parameter<std::string>("lidar_qos_reliability", "reliable");
    tracking_frame_ = declare_parameter<std::string>("tracking_frame", "base_link");
    const auto tracking_frame_override = declare_parameter<std::string>(
      "tracking_frame_override", "");
    if (!tracking_frame_override.empty()) {
      tracking_frame_ = tracking_frame_override;
    }
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");

    min_range_ = declare_parameter<double>("min_range", 0.5);
    max_range_ = declare_parameter<double>("max_range", 8.0);
    min_z_ = declare_parameter<double>("min_z", -0.25);
    max_z_ = declare_parameter<double>("max_z", 2.2);
    ground_z_threshold_ = declare_parameter<double>("ground_z_threshold", -0.15);
    remove_ground_ = declare_parameter<bool>("remove_ground", true);
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.10);
    max_points_before_clustering_ = declare_parameter<int>("max_points_before_clustering", 12000);
    process_every_n_clouds_ = declare_parameter<int>("process_every_n_clouds", 2);

    cluster_tolerance_ = declare_parameter<double>("cluster_tolerance", 0.38);
    const int legacy_min_cluster_points = declare_parameter<int>("min_cluster_points", 5);
    cluster_core_min_points_ = declare_parameter<int>("cluster_core_min_points", 3);
    generic_tracklet_min_points_ = declare_parameter<int>(
      "generic_tracklet_min_points", legacy_min_cluster_points);
    camera_associated_min_points_ = declare_parameter<int>("camera_associated_min_points", 2);
    max_cluster_points_ = declare_parameter<int>("max_cluster_points", 2500);
    near_range_limit_ = declare_parameter<double>("near_range_limit", 2.0);
    mid_range_limit_ = declare_parameter<double>("mid_range_limit", 6.0);
    near_min_cluster_points_ = declare_parameter<int>("near_min_cluster_points", 3);
    mid_min_cluster_points_ = declare_parameter<int>("mid_min_cluster_points", 6);
    far_min_cluster_points_ = declare_parameter<int>("far_min_cluster_points", 2);
    near_min_bbox_height_ = declare_parameter<double>("near_min_bbox_height", 0.10);
    mid_min_bbox_height_ = declare_parameter<double>("mid_min_bbox_height", 0.25);
    far_min_bbox_height_ = declare_parameter<double>("far_min_bbox_height", 0.05);
    impossible_candidate_width_ = declare_parameter<double>("impossible_candidate_width", 3.0);
    impossible_candidate_depth_ = declare_parameter<double>("impossible_candidate_depth", 3.0);
    impossible_candidate_height_ = declare_parameter<double>("impossible_candidate_height", 2.8);
    min_bbox_height_ = declare_parameter<double>("min_bbox_height", 0.15);
    max_bbox_height_ = declare_parameter<double>("max_bbox_height", 2.2);
    min_bbox_width_ = declare_parameter<double>("min_bbox_width", 0.05);
    max_bbox_width_ = declare_parameter<double>("max_bbox_width", 1.6);
    max_bbox_depth_ = declare_parameter<double>("max_bbox_depth", 1.6);
    show_impossible_raw_clusters_ = declare_parameter<bool>("show_impossible_raw_clusters", false);
    max_visual_candidate_width_ = declare_parameter<double>("max_visual_candidate_width", 3.0);
    max_visual_candidate_depth_ = declare_parameter<double>("max_visual_candidate_depth", 3.0);
    max_visual_candidate_height_ = declare_parameter<double>("max_visual_candidate_height", 2.8);

    tracklet_gating_distance_ = declare_parameter<double>("tracklet_gating_distance", 1.2);
    tracklet_confirm_hits_ = declare_parameter<int>("tracklet_confirm_hits", 3);
    max_tracklet_missed_frames_ = declare_parameter<int>("max_tracklet_missed_frames", 12);
    max_tracklet_age_sec_ = declare_parameter<double>("max_tracklet_age_sec", 1.5);
    max_tracklet_speed_mps_ = declare_parameter<double>("max_tracklet_speed_mps", 2.2);
    tracklet_process_noise_accel_std_ = declare_parameter<double>(
      "tracklet_process_noise_accel_std", 1.5);
    tracklet_maneuver_noise_accel_std_ = declare_parameter<double>(
      "tracklet_maneuver_noise_accel_std", 3.0);
    tracklet_measurement_variance_ = declare_parameter<double>(
      "tracklet_measurement_variance", 0.12);
    tracklet_nis_gate_ = declare_parameter<double>("tracklet_nis_gate", 9.21);
    stationary_displacement_m_ = declare_parameter<double>(
      "stationary_displacement_m", 0.10);
    stationary_confirm_frames_ = declare_parameter<int>("stationary_confirm_frames", 3);
    stationary_velocity_decay_ = declare_parameter<double>("stationary_velocity_decay", 0.5);
    stationary_speed_threshold_mps_ = declare_parameter<double>(
      "stationary_speed_threshold_mps", 0.08);
    stationary_measurement_speed_mps_ = declare_parameter<double>(
      "stationary_measurement_speed_mps", 0.25);
    confidence_hit_increment_ = declare_parameter<double>("confidence_hit_increment", 0.15);
    confidence_miss_decay_ = declare_parameter<double>("confidence_miss_decay", 0.06);
    min_publish_confidence_ = declare_parameter<double>("min_publish_confidence", 0.10);

    marker_publish_rate_ = declare_parameter<double>("marker_publish_rate", 10.0);
    candidate_marker_publish_rate_ = declare_parameter<double>("candidate_marker_publish_rate", 5.0);
    debug_publish_rate_ = declare_parameter<double>("debug_publish_rate", 2.0);

    // A depth-one subscription makes overload behavior deterministic: process the newest scan.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    if (lidar_qos_reliability_ == "best_effort" || lidar_qos_reliability_ == "sensor_data") {
      qos.best_effort();
    } else {
      qos.reliable();
    }

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, qos,
      std::bind(&LidarTrackletManagerNode::cloudCallback, this, std::placeholders::_1));
    tracklet_pub_ = create_publisher<track_robot_interfaces::msg::LidarTrackletArray>(
      output_topic_, 5);
    if (!semantic_output_topic_.empty()) {
      semantic_tracklet_pub_ = create_publisher<
        track_robot_interfaces::msg::SemanticLidarTrackletArray>(
        semantic_output_topic_, 5);
    }
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 5);
    clusters_pub_ = create_publisher<track_robot_interfaces::msg::LidarClusterArray>(
      candidate_clusters_topic_, 5);
    candidate_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      candidate_marker_topic_, 5);
    debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, 5);

    marker_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(marker_publish_rate_, 1.0))),
      std::bind(&LidarTrackletManagerNode::publishMarkers, this));

    RCLCPP_INFO(
      get_logger(),
      "LiDAR tracklet manager: lidar=%s output=%s frame=%s range=%.1f-%.1fm",
      lidar_topic_.c_str(), output_topic_.c_str(), tracking_frame_.c_str(),
      min_range_, max_range_);
  }

 private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    ++cloud_count_;
    if (process_every_n_clouds_ > 1 && cloud_count_ % process_every_n_clouds_ != 0) {
      return;
    }

    const auto start = std::chrono::steady_clock::now();
    rclcpp::Time measurement_time(msg->header.stamp);
    if (!measurement_time.nanoseconds()) {
      measurement_time = get_clock()->now();
    }
    if (last_measurement_stamp_.nanoseconds() &&
      (measurement_time - last_measurement_stamp_).seconds() < -0.10) {
      tracklets_.clear();
      tf_buffer_.clear();
      next_tracklet_id_ = 0;
      source_epoch_->advance();
      last_candidate_marker_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      RCLCPP_INFO(get_logger(), "LiDAR timestamp reset detected; cleared generic tracklets");
    }
    last_measurement_stamp_ = measurement_time;

    std::vector<PointXYZI> downsampled;
    size_t raw_point_count = 0;
    size_t cropped_point_count = 0;
    std::string tf_status;
    if (!preprocessCloud(
        *msg, measurement_time, downsampled, raw_point_count,
        cropped_point_count, tf_status)) {
      publishDebug(measurement_time, tf_status, raw_point_count, 0, 0, 0.0);
      return;
    }
    if (static_cast<int>(downsampled.size()) > max_points_before_clustering_) {
      downsampled.resize(static_cast<size_t>(max_points_before_clustering_));
    }

    auto detections = clusterDetections(downsampled);
    updateTracklets(detections, measurement_time);

    publishTracklets(measurement_time);
    publishCandidateClusters(detections, measurement_time);

    const auto end = std::chrono::steady_clock::now();
    const double processing_ms =
      std::chrono::duration<double, std::milli>(end - start).count();
    publishDebug(
      measurement_time, tf_status, raw_point_count, cropped_point_count, downsampled.size(),
      detections.size(), processing_ms);
  }

  bool preprocessCloud(
    const sensor_msgs::msg::PointCloud2 &cloud,
    const rclcpp::Time &stamp,
    std::vector<PointXYZI> &output,
    size_t &raw_count,
    size_t &cropped_count,
    std::string &status) {
    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;
    int intensity_offset = -1;
    for (const auto &field : cloud.fields) {
      if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        x_offset = static_cast<int>(field.offset);
      } else if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        y_offset = static_cast<int>(field.offset);
      } else if (field.name == "z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        z_offset = static_cast<int>(field.offset);
      } else if (field.name == "intensity" &&
        field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        intensity_offset = static_cast<int>(field.offset);
      }
    }
    if (x_offset < 0 || y_offset < 0 || z_offset < 0 || cloud.is_bigendian) {
      status = x_offset < 0 || y_offset < 0 || z_offset < 0 ?
        "missing_xyz_fields" : "bigendian_not_supported";
      return false;
    }

    const std::string source_frame = cloud.header.frame_id.empty() ? tracking_frame_ :
      cloud.header.frame_id;
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    if (source_frame != tracking_frame_) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          tracking_frame_, source_frame, stamp, 30ms);
        const auto &q = transform.transform.rotation;
        Eigen::Quaterniond quaternion(q.w, q.x, q.y, q.z);
        quaternion.normalize();
        rotation = quaternion.toRotationMatrix();
        const auto &t = transform.transform.translation;
        translation = Eigen::Vector3d(t.x, t.y, t.z);
        status = "tf";
      } catch (const std::exception &ex) {
        status = std::string("missing_tf_") + source_frame + "_to_" + tracking_frame_ +
          ":" + ex.what();
        return false;
      }
    } else {
      status = "already_tracking_frame";
    }
    updateRobotOrigin(stamp);

    std::unordered_map<VoxelKey, std::pair<PointXYZI, int>, VoxelKeyHash> voxels;
    const size_t point_count = static_cast<size_t>(cloud.width) * cloud.height;
    voxels.reserve(std::min<size_t>(point_count, static_cast<size_t>(max_points_before_clustering_ * 2)));
    if (voxel_leaf_size_ <= 0.0) {
      output.reserve(std::min<size_t>(point_count, static_cast<size_t>(max_points_before_clustering_)));
    }
    for (uint32_t row = 0; row < cloud.height; ++row) {
      for (uint32_t col = 0; col < cloud.width; ++col) {
        const size_t offset = static_cast<size_t>(row) * cloud.row_step +
          static_cast<size_t>(col) * cloud.point_step;
        if (offset + cloud.point_step > cloud.data.size()) {
          continue;
        }
        PointXYZI raw;
        std::memcpy(&raw.x, cloud.data.data() + offset + x_offset, sizeof(float));
        std::memcpy(&raw.y, cloud.data.data() + offset + y_offset, sizeof(float));
        std::memcpy(&raw.z, cloud.data.data() + offset + z_offset, sizeof(float));
        if (intensity_offset >= 0) {
          std::memcpy(&raw.intensity, cloud.data.data() + offset + intensity_offset, sizeof(float));
        }
        if (!std::isfinite(raw.x) || !std::isfinite(raw.y) || !std::isfinite(raw.z)) {
          continue;
        }
        ++raw_count;
        const Eigen::Vector3d transformed =
          rotation * Eigen::Vector3d(raw.x, raw.y, raw.z) + translation;
        PointXYZI point{
          static_cast<float>(transformed.x()), static_cast<float>(transformed.y()),
          static_cast<float>(transformed.z()), raw.intensity};
        const double range = std::hypot(
          static_cast<double>(point.x - current_robot_origin_.x),
          static_cast<double>(point.y - current_robot_origin_.y));
        if (range < min_range_ || range > max_range_ || point.z < min_z_ ||
          point.z > max_z_ || (remove_ground_ && point.z < ground_z_threshold_)) {
          continue;
        }
        ++cropped_count;
        if (voxel_leaf_size_ <= 0.0) {
          if (output.size() < static_cast<size_t>(max_points_before_clustering_)) {
            output.emplace_back(point);
          }
          continue;
        }
        const VoxelKey key{
          static_cast<int>(std::floor(point.x / voxel_leaf_size_)),
          static_cast<int>(std::floor(point.y / voxel_leaf_size_)),
          static_cast<int>(std::floor(point.z / voxel_leaf_size_))};
        auto &entry = voxels[key];
        entry.first.x += point.x;
        entry.first.y += point.y;
        entry.first.z += point.z;
        entry.first.intensity += point.intensity;
        ++entry.second;
      }
    }
    if (voxel_leaf_size_ > 0.0) {
      output.reserve(std::min<size_t>(voxels.size(), static_cast<size_t>(max_points_before_clustering_)));
      for (const auto &item : voxels) {
        if (output.size() >= static_cast<size_t>(max_points_before_clustering_)) {
          break;
        }
        const float count = static_cast<float>(std::max(1, item.second.second));
        const auto &sum = item.second.first;
        output.emplace_back(PointXYZI{
          sum.x / count, sum.y / count, sum.z / count, sum.intensity / count});
      }
    }
    return true;
  }

  bool readCloud(
    const sensor_msgs::msg::PointCloud2 &cloud,
    std::vector<PointXYZI> &points,
    std::string &status) const {
    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;
    int intensity_offset = -1;
    for (const auto &field : cloud.fields) {
      if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        x_offset = static_cast<int>(field.offset);
      } else if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        y_offset = static_cast<int>(field.offset);
      } else if (field.name == "z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        z_offset = static_cast<int>(field.offset);
      } else if (
        field.name == "intensity" &&
        field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        intensity_offset = static_cast<int>(field.offset);
      }
    }
    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
      status = "missing_xyz_fields";
      return false;
    }
    if (cloud.is_bigendian) {
      status = "bigendian_not_supported";
      return false;
    }

    const size_t point_count = static_cast<size_t>(cloud.width) * cloud.height;
    points.reserve(point_count);
    for (uint32_t row = 0; row < cloud.height; ++row) {
      for (uint32_t col = 0; col < cloud.width; ++col) {
        const size_t offset =
          static_cast<size_t>(row) * cloud.row_step +
          static_cast<size_t>(col) * cloud.point_step;
        if (offset + cloud.point_step > cloud.data.size()) {
          continue;
        }
        PointXYZI point;
        std::memcpy(&point.x, cloud.data.data() + offset + x_offset, sizeof(float));
        std::memcpy(&point.y, cloud.data.data() + offset + y_offset, sizeof(float));
        std::memcpy(&point.z, cloud.data.data() + offset + z_offset, sizeof(float));
        if (intensity_offset >= 0) {
          std::memcpy(
            &point.intensity, cloud.data.data() + offset + intensity_offset, sizeof(float));
        }
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
          points.emplace_back(point);
        }
      }
    }
    status = "ok";
    return true;
  }

  bool transformToTrackingFrame(
    const std::vector<PointXYZI> &input,
    const std::string &source_frame,
    const rclcpp::Time &stamp,
    std::vector<PointXYZI> &output,
    std::string &status) {
    const auto normalized_source = source_frame.empty() ? tracking_frame_ : source_frame;
    if (normalized_source == tracking_frame_) {
      output = input;
      updateRobotOrigin(stamp);
      status = "already_tracking_frame";
      return true;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        tracking_frame_, normalized_source, stamp, 30ms);
    } catch (const std::exception &ex) {
      status = std::string("missing_tf_") + normalized_source + "_to_" + tracking_frame_ +
        ":" + ex.what();
      return false;
    }

    const auto &t = transform.transform.translation;
    const auto &q = transform.transform.rotation;
    const double xx = q.x * q.x;
    const double yy = q.y * q.y;
    const double zz = q.z * q.z;
    const double xy = q.x * q.y;
    const double xz = q.x * q.z;
    const double yz = q.y * q.z;
    const double wx = q.w * q.x;
    const double wy = q.w * q.y;
    const double wz = q.w * q.z;
    const double r00 = 1.0 - 2.0 * (yy + zz);
    const double r01 = 2.0 * (xy - wz);
    const double r02 = 2.0 * (xz + wy);
    const double r10 = 2.0 * (xy + wz);
    const double r11 = 1.0 - 2.0 * (xx + zz);
    const double r12 = 2.0 * (yz - wx);
    const double r20 = 2.0 * (xz - wy);
    const double r21 = 2.0 * (yz + wx);
    const double r22 = 1.0 - 2.0 * (xx + yy);
    output.reserve(input.size());
    for (const auto &point : input) {
      PointXYZI transformed;
      transformed.x = static_cast<float>(r00 * point.x + r01 * point.y + r02 * point.z + t.x);
      transformed.y = static_cast<float>(r10 * point.x + r11 * point.y + r12 * point.z + t.y);
      transformed.z = static_cast<float>(r20 * point.x + r21 * point.y + r22 * point.z + t.z);
      transformed.intensity = point.intensity;
      output.emplace_back(transformed);
    }
    status = "tf";
    updateRobotOrigin(stamp);
    return true;
  }

  void updateRobotOrigin(const rclcpp::Time &stamp) {
    current_robot_origin_ = PointXYZI{};
    if (tracking_frame_ == base_frame_) {
      return;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        tracking_frame_, base_frame_, stamp, 30ms);
      current_robot_origin_.x = static_cast<float>(transform.transform.translation.x);
      current_robot_origin_.y = static_cast<float>(transform.transform.translation.y);
      current_robot_origin_.z = static_cast<float>(transform.transform.translation.z);
    } catch (const std::exception &) {
      // Cloud transformation may still be valid through a different chain. Debug output
      // continues to expose the TF status; origin zero is the stationary fallback.
    }
  }

  std::vector<PointXYZI> cropPoints(const std::vector<PointXYZI> &points) const {
    std::vector<PointXYZI> output;
    output.reserve(points.size());
    for (const auto &point : points) {
      const double range = std::hypot(
        static_cast<double>(point.x - current_robot_origin_.x),
        static_cast<double>(point.y - current_robot_origin_.y));
      if (range < min_range_ || range > max_range_) {
        continue;
      }
      if (point.z < min_z_ || point.z > max_z_) {
        continue;
      }
      if (remove_ground_ && point.z < ground_z_threshold_) {
        continue;
      }
      output.emplace_back(point);
    }
    return output;
  }

  std::vector<PointXYZI> voxelDownsample(const std::vector<PointXYZI> &points) const {
    if (voxel_leaf_size_ <= 0.0 || points.empty()) {
      return points;
    }
    std::unordered_map<VoxelKey, std::pair<PointXYZI, int>, VoxelKeyHash> voxels;
    voxels.reserve(points.size());
    for (const auto &point : points) {
      const VoxelKey key{
        static_cast<int>(std::floor(point.x / voxel_leaf_size_)),
        static_cast<int>(std::floor(point.y / voxel_leaf_size_)),
        static_cast<int>(std::floor(point.z / voxel_leaf_size_))};
      auto &entry = voxels[key];
      entry.first.x += point.x;
      entry.first.y += point.y;
      entry.first.z += point.z;
      entry.first.intensity += point.intensity;
      entry.second += 1;
    }

    std::vector<PointXYZI> output;
    output.reserve(voxels.size());
    for (const auto &item : voxels) {
      const auto &sum = item.second.first;
      const float count = static_cast<float>(std::max(item.second.second, 1));
      output.emplace_back(PointXYZI{
        sum.x / count, sum.y / count, sum.z / count, sum.intensity / count});
    }
    return output;
  }

  std::vector<ClusterDetection> clusterDetections(const std::vector<PointXYZI> &points) const {
    std::vector<ClusterDetection> detections;
    if (points.size() < static_cast<size_t>(cluster_core_min_points_)) {
      return detections;
    }

    std::vector<int> labels(points.size(), -1);
    int cluster_id = 0;
    const double tolerance2 = cluster_tolerance_ * cluster_tolerance_;
    const auto grid = buildClusterGrid(points);

    for (size_t seed = 0; seed < points.size(); ++seed) {
      if (labels[seed] >= 0) {
        continue;
      }
      std::vector<int> neighbors = radiusNeighbors(points, grid, seed, tolerance2);
      if (neighbors.size() < static_cast<size_t>(cluster_core_min_points_)) {
        continue;
      }

      labels[seed] = cluster_id;
      std::queue<int> queue;
      for (const auto index : neighbors) {
        if (labels[static_cast<size_t>(index)] == -1) {
          labels[static_cast<size_t>(index)] = -2;
          queue.push(index);
        }
      }

      while (!queue.empty()) {
        const int index = queue.front();
        queue.pop();
        if (labels[static_cast<size_t>(index)] >= 0) {
          continue;
        }
        labels[static_cast<size_t>(index)] = cluster_id;
        auto more = radiusNeighbors(points, grid, static_cast<size_t>(index), tolerance2);
        if (more.size() >= static_cast<size_t>(cluster_core_min_points_)) {
          for (const auto next : more) {
            if (labels[static_cast<size_t>(next)] == -1) {
              labels[static_cast<size_t>(next)] = -2;
              queue.push(next);
            }
          }
        }
      }
      ++cluster_id;
    }

    for (int id = 0; id < cluster_id; ++id) {
      ClusterDetection detection;
      for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] == id) {
          detection.indices.emplace_back(static_cast<int>(i));
        }
      }
      fillDetection(points, detection);
      classifyDetection(detection);
      detections.emplace_back(detection);
    }
    return detections;
  }

  std::unordered_map<GridKey, std::vector<int>, GridKeyHash> buildClusterGrid(
    const std::vector<PointXYZI> &points) const {
    std::unordered_map<GridKey, std::vector<int>, GridKeyHash> grid;
    grid.reserve(points.size());
    const double cell = std::max(cluster_tolerance_, 1e-3);
    for (size_t i = 0; i < points.size(); ++i) {
      const GridKey key{
        static_cast<int>(std::floor(points[i].x / cell)),
        static_cast<int>(std::floor(points[i].y / cell))};
      grid[key].emplace_back(static_cast<int>(i));
    }
    return grid;
  }

  std::vector<int> radiusNeighbors(
    const std::vector<PointXYZI> &points,
    const std::unordered_map<GridKey, std::vector<int>, GridKeyHash> &grid,
    const size_t seed,
    const double tolerance2) const {
    std::vector<int> neighbors;
    const auto &origin = points[seed];
    const double cell = std::max(cluster_tolerance_, 1e-3);
    const int cx = static_cast<int>(std::floor(origin.x / cell));
    const int cy = static_cast<int>(std::floor(origin.y / cell));
    for (int dx_cell = -1; dx_cell <= 1; ++dx_cell) {
      for (int dy_cell = -1; dy_cell <= 1; ++dy_cell) {
        const GridKey key{cx + dx_cell, cy + dy_cell};
        const auto iter = grid.find(key);
        if (iter == grid.end()) {
          continue;
        }
        for (const auto index : iter->second) {
          const auto &candidate = points[static_cast<size_t>(index)];
          const double dx = static_cast<double>(candidate.x - origin.x);
          const double dy = static_cast<double>(candidate.y - origin.y);
          if (dx * dx + dy * dy <= tolerance2) {
            neighbors.emplace_back(index);
          }
        }
      }
    }
    return neighbors;
  }

  void fillDetection(const std::vector<PointXYZI> &points, ClusterDetection &detection) const {
    detection.minimum = PointXYZI{std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0.0F};
    detection.maximum = PointXYZI{std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), 0.0F};
    PointXYZI sum;
    for (const auto index : detection.indices) {
      const auto &point = points[static_cast<size_t>(index)];
      sum.x += point.x;
      sum.y += point.y;
      sum.z += point.z;
      detection.minimum.x = std::min(detection.minimum.x, point.x);
      detection.minimum.y = std::min(detection.minimum.y, point.y);
      detection.minimum.z = std::min(detection.minimum.z, point.z);
      detection.maximum.x = std::max(detection.maximum.x, point.x);
      detection.maximum.y = std::max(detection.maximum.y, point.y);
      detection.maximum.z = std::max(detection.maximum.z, point.z);
    }
    const float count = static_cast<float>(std::max<size_t>(detection.indices.size(), 1));
    detection.centroid = PointXYZI{sum.x / count, sum.y / count, sum.z / count, 0.0F};
    detection.distance = distanceXY(detection.centroid, current_robot_origin_);
    detection.bearing = std::atan2(
      detection.centroid.y - current_robot_origin_.y,
      detection.centroid.x - current_robot_origin_.x);
    detection.dynamic_score = 0.0;
  }

  void classifyDetection(ClusterDetection &detection) const {
    const double width = std::abs(detection.maximum.x - detection.minimum.x);
    const double depth = std::abs(detection.maximum.y - detection.minimum.y);
    const double height = std::abs(detection.maximum.z - detection.minimum.z);
    const int adaptive_min_points = minClusterPointsForRange(detection.distance);
    const double adaptive_min_height = minHeightForRange(detection.distance);
    detection.too_sparse =
      detection.indices.size() < static_cast<size_t>(adaptive_min_points);
    detection.too_large = detection.indices.size() > static_cast<size_t>(max_cluster_points_);
    detection.oversized =
      width > max_bbox_width_ || depth > max_bbox_depth_ || height > max_bbox_height_;
    detection.near_static = false;
    const bool too_small =
      width < min_bbox_width_ || depth < min_bbox_width_ || height < adaptive_min_height;
    const bool impossible_size =
      width > impossible_candidate_width_ ||
      depth > impossible_candidate_depth_ ||
      height > impossible_candidate_height_;
    detection.tracklet_eligible =
      !impossible_size &&
      !detection.too_sparse && !detection.too_large && !detection.oversized && !too_small;
    const double point_reference = std::max(3.0, 30.0 / std::max(1.0, detection.distance));
    const double point_score = std::min(1.0, detection.indices.size() / point_reference);
    const double height_score = std::min(1.0, height / std::max(0.15, adaptive_min_height));
    const double geometry_score = detection.oversized ? 0.2 : (too_small ? 0.4 : 1.0);
    detection.observation_quality =
      std::max(0.05, std::min(1.0, 0.45 * point_score + 0.35 * height_score +
      0.20 * geometry_score));
    detection.measurement_variance =
      tracklet_measurement_variance_ / std::max(0.15, detection.observation_quality);
  }

  int minClusterPointsForRange(const double range) const {
    if (range < near_range_limit_) {
      return std::max(1, near_min_cluster_points_);
    }
    if (range < mid_range_limit_) {
      return std::max(1, mid_min_cluster_points_);
    }
    return std::max(1, far_min_cluster_points_);
  }

  double minHeightForRange(const double range) const {
    if (range < near_range_limit_) {
      return std::max(0.0, near_min_bbox_height_);
    }
    if (range < mid_range_limit_) {
      return std::max(0.0, mid_min_bbox_height_);
    }
    return std::max(0.0, far_min_bbox_height_);
  }

  void updateTracklets(const std::vector<ClusterDetection> &detections, const rclcpp::Time &now) {
    std::vector<ClusterDetection> eligible_detections;
    eligible_detections.reserve(detections.size());
    for (const auto &detection : detections) {
      if (detection.tracklet_eligible) {
        eligible_detections.emplace_back(detection);
      }
    }

    for (auto &track : tracklets_) {
      predictTrack(track, now);
    }

    constexpr double invalid_cost = 1.0e5;
    std::vector<std::vector<double>> costs(
      tracklets_.size(), std::vector<double>(eligible_detections.size(), invalid_cost));
    for (size_t ti = 0; ti < tracklets_.size(); ++ti) {
      for (size_t di = 0; di < eligible_detections.size(); ++di) {
        costs[ti][di] = associationCost(tracklets_[ti], eligible_detections[di], invalid_cost);
      }
    }
    const auto matched_track = globalAssignment(costs, invalid_cost);
    std::vector<int> matched_detection(eligible_detections.size(), -1);

    for (size_t ti = 0; ti < tracklets_.size(); ++ti) {
      const int di = ti < matched_track.size() ? matched_track[ti] : -1;
      if (di >= 0 && static_cast<size_t>(di) < eligible_detections.size() &&
        costs[ti][static_cast<size_t>(di)] < invalid_cost) {
        matched_detection[static_cast<size_t>(di)] = static_cast<int>(ti);
        updateMatchedTrack(tracklets_[ti], eligible_detections[static_cast<size_t>(di)], now);
      } else {
        markMissed(tracklets_[ti]);
      }
    }

    for (size_t di = 0; di < eligible_detections.size(); ++di) {
      if (matched_detection[di] < 0) {
        createTracklet(eligible_detections[di], now);
      }
    }

    tracklets_.erase(
      std::remove_if(
        tracklets_.begin(), tracklets_.end(),
        [&](const Tracklet &track) {
          const double age_sec = (now - track.last_seen).seconds();
          return (
            track.miss_count > static_cast<uint32_t>(max_tracklet_missed_frames_) ||
            age_sec > max_tracklet_age_sec_ ||
            track.confidence <= 0.0);
        }),
      tracklets_.end());
  }

  double associationCost(
    const Tracklet &track,
    const ClusterDetection &detection,
    const double invalid_cost) const {
    const Eigen::Vector2d measurement(detection.centroid.x, detection.centroid.y);
    const Eigen::Vector2d residual = measurement - track.state.head<2>();
    Eigen::Matrix2d innovation = track.covariance.block<2, 2>(0, 0);
    innovation.diagonal().array() += detection.measurement_variance;
    const double nis = residual.transpose() * innovation.inverse() * residual;
    const double distance = residual.norm();
    if (distance > tracklet_gating_distance_ || nis > tracklet_nis_gate_) {
      return invalid_cost;
    }
    const double size_change = std::min(1.0, sizeDistance(track, detection) / 1.5);
    const double point_ratio = std::abs(std::log(
      (static_cast<double>(detection.indices.size()) + 1.0) /
      (static_cast<double>(track.point_count) + 1.0)));
    return
      0.65 * std::min(1.0, nis / std::max(1e-6, tracklet_nis_gate_)) +
      0.20 * size_change +
      0.15 * std::min(1.0, point_ratio);
  }

  std::vector<int> globalAssignment(
    const std::vector<std::vector<double>> &costs,
    const double invalid_cost) const {
    const size_t rows = costs.size();
    const size_t cols = rows ? costs.front().size() : 0;
    if (rows == 0) {
      return {};
    }
    if (cols == 0) {
      return std::vector<int>(rows, -1);
    }

    const size_t n = rows + cols;
    const double unmatched_cost = 1.25;
    std::vector<std::vector<double>> square(n + 1, std::vector<double>(n + 1, 0.0));
    for (size_t i = 0; i < rows; ++i) {
      for (size_t j = 0; j < cols; ++j) {
        square[i + 1][j + 1] = costs[i][j];
      }
      for (size_t j = cols; j < n; ++j) {
        square[i + 1][j + 1] = unmatched_cost;
      }
    }
    for (size_t i = rows; i < n; ++i) {
      for (size_t j = 0; j < cols; ++j) {
        square[i + 1][j + 1] = unmatched_cost;
      }
    }

    std::vector<double> u(n + 1, 0.0);
    std::vector<double> v(n + 1, 0.0);
    std::vector<size_t> p(n + 1, 0);
    std::vector<size_t> way(n + 1, 0);
    for (size_t i = 1; i <= n; ++i) {
      p[0] = i;
      size_t j0 = 0;
      std::vector<double> minv(n + 1, invalid_cost);
      std::vector<bool> used(n + 1, false);
      do {
        used[j0] = true;
        const size_t i0 = p[j0];
        double delta = invalid_cost;
        size_t j1 = 0;
        for (size_t j = 1; j <= n; ++j) {
          if (used[j]) {
            continue;
          }
          const double current = square[i0][j] - u[i0] - v[j];
          if (current < minv[j]) {
            minv[j] = current;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
        for (size_t j = 0; j <= n; ++j) {
          if (used[j]) {
            u[p[j]] += delta;
            v[j] -= delta;
          } else {
            minv[j] -= delta;
          }
        }
        j0 = j1;
      } while (p[j0] != 0);
      do {
        const size_t j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
      } while (j0 != 0);
    }

    std::vector<int> assignment(rows, -1);
    for (size_t j = 1; j <= n; ++j) {
      if (p[j] > 0 && p[j] <= rows && j <= cols &&
        costs[p[j] - 1][j - 1] < invalid_cost) {
        assignment[p[j] - 1] = static_cast<int>(j - 1);
      }
    }
    return assignment;
  }

  void predictTrack(Tracklet &track, const rclcpp::Time &now) {
    const double dt = std::min(0.5, std::max(0.0, (now - track.last_update).seconds()));
    const Eigen::Vector2d previous_position = track.state.head<2>();
    Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
    transition(0, 2) = dt;
    transition(1, 3) = dt;
    track.state = transition * track.state;
    track.covariance =
      transition * track.covariance * transition.transpose() +
      processNoise(dt, tracklet_process_noise_accel_std_);
    const Eigen::Vector2d shift = track.state.head<2>() - previous_position;
    track.minimum.x += static_cast<float>(shift.x());
    track.maximum.x += static_cast<float>(shift.x());
    track.minimum.y += static_cast<float>(shift.y());
    track.maximum.y += static_cast<float>(shift.y());
    syncTrackState(track);
    track.last_update = now;
    ++track.age_frames;
  }

  void updateMatchedTrack(
    Tracklet &track,
    const ClusterDetection &detection,
    const rclcpp::Time &now) {
    const double dt = std::max(1e-3, (now - track.last_seen).seconds());
    const Eigen::Vector2d measurement(detection.centroid.x, detection.centroid.y);
    const Eigen::Vector2d residual = measurement - track.state.head<2>();
    const Eigen::Vector2d measured_motion(
      detection.centroid.x - track.last_detection_position.x,
      detection.centroid.y - track.last_detection_position.y);
    const Eigen::Vector2d predicted_velocity = track.state.tail<2>();
    const bool maneuver =
      measured_motion.norm() > stationary_displacement_m_ &&
      predicted_velocity.norm() > stationary_speed_threshold_mps_ &&
      measured_motion.dot(predicted_velocity) <
        0.5 * measured_motion.norm() * predicted_velocity.norm();
    if (maneuver) {
      track.covariance += processNoise(dt, tracklet_maneuver_noise_accel_std_);
    }

    Eigen::Matrix<double, 2, 4> observation;
    observation.setZero();
    observation(0, 0) = 1.0;
    observation(1, 1) = 1.0;
    const Eigen::Matrix2d measurement_covariance =
      Eigen::Matrix2d::Identity() * detection.measurement_variance;
    const Eigen::Matrix2d innovation_covariance =
      observation * track.covariance * observation.transpose() + measurement_covariance;
    const Eigen::Matrix<double, 4, 2> gain =
      track.covariance * observation.transpose() * innovation_covariance.inverse();
    track.state += gain * residual;
    const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
    const Eigen::Matrix4d kh = gain * observation;
    track.covariance =
      (identity - kh) * track.covariance * (identity - kh).transpose() +
      gain * measurement_covariance * gain.transpose();

    const double measured_speed = measured_motion.norm() / dt;
    if (measured_motion.norm() < stationary_displacement_m_ &&
      measured_speed < stationary_measurement_speed_mps_) {
      ++track.stationary_count;
    } else {
      track.stationary_count = 0;
    }
    if (track.stationary_count >= static_cast<uint32_t>(stationary_confirm_frames_)) {
      track.state.tail<2>() *= stationary_velocity_decay_;
      if (track.state.tail<2>().norm() < stationary_speed_threshold_mps_) {
        track.state.tail<2>().setZero();
      }
    }
    clampStateVelocity(track.state);
    syncTrackState(track);
    track.minimum = detection.minimum;
    track.maximum = detection.maximum;
    track.last_detection_position = detection.centroid;
    track.point_count = detection.indices.size();
    track.confidence = std::min(1.0, track.confidence + confidence_hit_increment_);
    track.dynamic_score = detection.dynamic_score;
    track.observation_quality = detection.observation_quality;
    track.hit_count += 1;
    track.miss_count = 0;
    track.confirmed = track.hit_count >= static_cast<uint32_t>(tracklet_confirm_hits_);
    track.last_seen = now;
    track.last_update = now;
  }

  void markMissed(Tracklet &track) {
    track.miss_count += 1;
    track.confidence = std::max(0.0, track.confidence - confidence_miss_decay_);
  }

  void createTracklet(const ClusterDetection &detection, const rclcpp::Time &now) {
    Tracklet track;
    track.id = next_tracklet_id_++;
    track.position = detection.centroid;
    track.velocity = PointXYZI{};
    track.state <<
      detection.centroid.x, detection.centroid.y, 0.0, 0.0;
    track.covariance.setZero();
    track.covariance.diagonal() << 0.25, 0.25, 1.0, 1.0;
    track.minimum = detection.minimum;
    track.maximum = detection.maximum;
    track.last_detection_position = detection.centroid;
    track.point_count = detection.indices.size();
    track.confidence = std::min(1.0, confidence_hit_increment_);
    track.dynamic_score = detection.dynamic_score;
    track.observation_quality = detection.observation_quality;
    track.age_frames = 1;
    track.hit_count = 1;
    track.miss_count = 0;
    track.confirmed = track.hit_count >= static_cast<uint32_t>(tracklet_confirm_hits_);
    track.last_update = now;
    track.last_seen = now;
    tracklets_.emplace_back(track);
  }

  Eigen::Matrix4d processNoise(const double dt, const double acceleration_std) const {
    Eigen::Matrix4d noise = Eigen::Matrix4d::Zero();
    const double variance = acceleration_std * acceleration_std;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    for (int axis = 0; axis < 2; ++axis) {
      noise(axis, axis) = 0.25 * dt4 * variance;
      noise(axis, axis + 2) = 0.5 * dt3 * variance;
      noise(axis + 2, axis) = 0.5 * dt3 * variance;
      noise(axis + 2, axis + 2) = dt2 * variance;
    }
    return noise;
  }

  void syncTrackState(Tracklet &track) const {
    track.position.x = static_cast<float>(track.state(0));
    track.position.y = static_cast<float>(track.state(1));
    track.velocity.x = static_cast<float>(track.state(2));
    track.velocity.y = static_cast<float>(track.state(3));
  }

  void clampStateVelocity(Eigen::Vector4d &state) const {
    const double speed = state.tail<2>().norm();
    if (speed <= max_tracklet_speed_mps_ || speed <= 1e-6) {
      return;
    }
    state.tail<2>() *= max_tracklet_speed_mps_ / speed;
  }

  double sizeDistance(const Tracklet &track, const ClusterDetection &detection) const {
    const double tx = std::abs(track.maximum.x - track.minimum.x);
    const double ty = std::abs(track.maximum.y - track.minimum.y);
    const double tz = std::abs(track.maximum.z - track.minimum.z);
    const double dx = std::abs(detection.maximum.x - detection.minimum.x);
    const double dy = std::abs(detection.maximum.y - detection.minimum.y);
    const double dz = std::abs(detection.maximum.z - detection.minimum.z);
    return std::abs(tx - dx) + std::abs(ty - dy) + std::abs(tz - dz);
  }

  void publishTracklets(const rclcpp::Time &now) {
    track_robot_interfaces::msg::LidarTrackletArray msg;
    msg.header.stamp = now;
    msg.header.frame_id = tracking_frame_;
    for (const auto &track : tracklets_) {
      if (!track.confirmed || track.confidence < min_publish_confidence_) {
        continue;
      }
      msg.tracklets.emplace_back(toTrackletMsg(track));
    }
    latest_tracklet_msg_ = msg;
    tracklet_pub_->publish(msg);

    if (semantic_tracklet_pub_) {
      track_robot_interfaces::msg::SemanticLidarTrackletArray semantic_msg;
      semantic_msg.header = msg.header;
      semantic_msg.source_epoch_id = source_epoch_->value();
      const size_t keep_count = std::min<size_t>(msg.tracklets.size(), 256U);
      semantic_msg.dropped_tracklet_count = static_cast<uint32_t>(
        msg.tracklets.size() - keep_count);
      for (size_t index = 0; index < keep_count; ++index) {
        semantic_msg.tracklets.emplace_back(msg.tracklets[index]);
      }
      latest_semantic_drop_count_ = semantic_msg.dropped_tracklet_count;
      semantic_tracklet_pub_->publish(semantic_msg);
    }
  }

  track_robot_interfaces::msg::LidarTracklet toTrackletMsg(const Tracklet &track) const {
    track_robot_interfaces::msg::LidarTracklet msg;
    msg.tracklet_id = track.id;
    msg.position = toPoint(track.position);
    msg.position_base = toPoint(track.position);
    msg.position_map_valid = false;
    msg.velocity = toVector(track.velocity);
    msg.minimum = toPoint(track.minimum);
    msg.maximum = toPoint(track.maximum);
    msg.size.x = std::abs(track.maximum.x - track.minimum.x);
    msg.size.y = std::abs(track.maximum.y - track.minimum.y);
    msg.size.z = std::abs(track.maximum.z - track.minimum.z);
    msg.point_count = static_cast<uint32_t>(track.point_count);
    msg.distance = static_cast<float>(distanceXY(track.position, current_robot_origin_));
    msg.bearing = static_cast<float>(std::atan2(
      track.position.y - current_robot_origin_.y,
      track.position.x - current_robot_origin_.x));
    msg.confidence = static_cast<float>(track.confidence);
    msg.dynamic_score = static_cast<float>(track.dynamic_score);
    msg.age_frames = track.age_frames;
    msg.hit_count = track.hit_count;
    msg.miss_count = track.miss_count;
    msg.confirmed = track.confirmed;
    msg.active = track.miss_count == 0;
    msg.observation_quality = static_cast<float>(track.observation_quality);
    msg.position_covariance_xy = {
      static_cast<float>(track.covariance(0, 0)),
      static_cast<float>(track.covariance(0, 1)),
      static_cast<float>(track.covariance(1, 0)),
      static_cast<float>(track.covariance(1, 1))};
    const int64_t stamp_ns = track.last_seen.nanoseconds();
    msg.last_measurement_stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    msg.last_measurement_stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    return msg;
  }

  void publishCandidateClusters(
    const std::vector<ClusterDetection> &detections,
    const rclcpp::Time &now) {
    track_robot_interfaces::msg::LidarClusterArray msg;
    msg.header.stamp = now;
    msg.header.frame_id = tracking_frame_;
    for (size_t i = 0; i < detections.size(); ++i) {
      const auto &detection = detections[i];
      track_robot_interfaces::msg::LidarCluster cluster;
      cluster.cluster_id = static_cast<int>(i);
      cluster.centroid = toPoint(detection.centroid);
      cluster.minimum = toPoint(detection.minimum);
      cluster.maximum = toPoint(detection.maximum);
      cluster.size.x = std::abs(detection.maximum.x - detection.minimum.x);
      cluster.size.y = std::abs(detection.maximum.y - detection.minimum.y);
      cluster.size.z = std::abs(detection.maximum.z - detection.minimum.z);
      cluster.point_count = static_cast<uint32_t>(detection.indices.size());
      cluster.distance = static_cast<float>(detection.distance);
      cluster.confidence = detection.tracklet_eligible ? 0.5F : 0.15F;
      cluster.human_candidate = detection.tracklet_eligible;
      cluster.oversized = detection.oversized;
      cluster.too_sparse = detection.too_sparse;
      cluster.too_large = detection.too_large;
      cluster.near_static = detection.near_static;
      cluster.dynamic_score = static_cast<float>(detection.dynamic_score);
      msg.clusters.emplace_back(cluster);
    }
    latest_cluster_count_ = detections.size();
    updateLatestClusterStats(detections);
    clusters_pub_->publish(msg);
    if (!last_candidate_marker_stamp_.nanoseconds() ||
      (now - last_candidate_marker_stamp_).seconds() >=
      1.0 / std::max(0.1, candidate_marker_publish_rate_)) {
      last_candidate_marker_stamp_ = now;
      publishCandidateMarkers(detections, now);
    }
  }

  void updateLatestClusterStats(const std::vector<ClusterDetection> &detections) {
    latest_eligible_cluster_count_ = 0;
    latest_weak_sparse_cluster_count_ = 0;
    latest_oversized_cluster_count_ = 0;
    latest_hidden_impossible_cluster_count_ = 0;
    for (const auto &detection : detections) {
      if (detection.tracklet_eligible) {
        ++latest_eligible_cluster_count_;
      }
      if (detection.too_sparse) {
        ++latest_weak_sparse_cluster_count_;
      }
      if (detection.oversized || detection.too_large) {
        ++latest_oversized_cluster_count_;
      }
      if (hideImpossibleCandidateMarker(detection)) {
        ++latest_hidden_impossible_cluster_count_;
      }
    }
  }

  void publishCandidateMarkers(
    const std::vector<ClusterDetection> &detections,
    const rclcpp::Time &now) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header.stamp = now;
    clear.header.frame_id = tracking_frame_;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.emplace_back(clear);

    int marker_id = 1;
    for (size_t i = 0; i < detections.size(); ++i) {
      const auto &detection = detections[i];
      if (hideImpossibleCandidateMarker(detection)) {
        continue;
      }
      visualization_msgs::msg::Marker box;
      box.header.stamp = now;
      box.header.frame_id = tracking_frame_;
      box.ns = "lidar_candidate_clusters";
      box.id = marker_id++;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose.position.x = 0.5 * (detection.minimum.x + detection.maximum.x);
      box.pose.position.y = 0.5 * (detection.minimum.y + detection.maximum.y);
      box.pose.position.z = 0.5 * (detection.minimum.z + detection.maximum.z);
      box.pose.orientation.w = 1.0;
      box.scale.x = std::max(0.05F, std::abs(detection.maximum.x - detection.minimum.x));
      box.scale.y = std::max(0.05F, std::abs(detection.maximum.y - detection.minimum.y));
      box.scale.z = std::max(0.05F, std::abs(detection.maximum.z - detection.minimum.z));
      if (detection.tracklet_eligible) {
        box.color.r = 0.1F;
        box.color.g = 0.8F;
        box.color.b = 0.3F;
        box.color.a = 0.18F;
      } else if (detection.oversized || detection.too_large) {
        box.color.r = 1.0F;
        box.color.g = 0.55F;
        box.color.b = 0.05F;
        box.color.a = 0.18F;
      } else {
        box.color.r = 0.65F;
        box.color.g = 0.65F;
        box.color.b = 0.65F;
        box.color.a = 0.12F;
      }
      box.lifetime = rclcpp::Duration::from_seconds(0.25);
      markers.markers.emplace_back(box);

      visualization_msgs::msg::Marker text;
      text.header = box.header;
      text.ns = "lidar_candidate_cluster_labels";
      text.id = marker_id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position = toPoint(detection.centroid);
      text.pose.position.z += 0.35;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.14;
      text.color.r = 1.0F;
      text.color.g = 1.0F;
      text.color.b = 1.0F;
      text.color.a = 0.9F;
      text.text = "c=" + std::to_string(i) +
        (detection.tracklet_eligible ? " eligible" : " raw") +
        (detection.oversized ? " oversized" : "") +
        (detection.too_large ? " too_large" : "") +
        (detection.too_sparse ? " sparse" : "") +
        " pts=" + std::to_string(detection.indices.size());
      text.lifetime = rclcpp::Duration::from_seconds(0.25);
      markers.markers.emplace_back(text);
    }
    candidate_marker_pub_->publish(markers);
  }

  bool hideImpossibleCandidateMarker(const ClusterDetection &detection) const {
    if (show_impossible_raw_clusters_ || detection.tracklet_eligible || detection.too_sparse) {
      return false;
    }
    const double width = std::abs(detection.maximum.x - detection.minimum.x);
    const double depth = std::abs(detection.maximum.y - detection.minimum.y);
    const double height = std::abs(detection.maximum.z - detection.minimum.z);
    return (
      width > max_visual_candidate_width_ ||
      depth > max_visual_candidate_depth_ ||
      height > max_visual_candidate_height_);
  }

  void publishMarkers() {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = latest_tracklet_msg_.header;
    clear.header.frame_id = tracking_frame_;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.emplace_back(clear);

    int marker_id = 1;
    for (const auto &track : tracklets_) {
      if (!track.confirmed || track.confidence < min_publish_confidence_) {
        continue;
      }
      visualization_msgs::msg::Marker box;
      box.header = latest_tracklet_msg_.header;
      box.header.frame_id = tracking_frame_;
      box.ns = "lidar_tracklets";
      box.id = marker_id++;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose.position.x = 0.5 * (track.minimum.x + track.maximum.x);
      box.pose.position.y = 0.5 * (track.minimum.y + track.maximum.y);
      box.pose.position.z = 0.5 * (track.minimum.z + track.maximum.z);
      box.pose.orientation.w = 1.0;
      box.scale.x = std::max(0.05F, std::abs(track.maximum.x - track.minimum.x));
      box.scale.y = std::max(0.05F, std::abs(track.maximum.y - track.minimum.y));
      box.scale.z = std::max(0.05F, std::abs(track.maximum.z - track.minimum.z));
      box.color.r = track.miss_count == 0 ? 0.1F : 0.6F;
      box.color.g = track.miss_count == 0 ? 0.7F : 0.6F;
      box.color.b = track.miss_count == 0 ? 1.0F : 0.6F;
      box.color.a = track.miss_count == 0 ? 0.28F : 0.14F;
      box.lifetime = rclcpp::Duration::from_seconds(0.35);
      markers.markers.emplace_back(box);

      visualization_msgs::msg::Marker text;
      text.header = box.header;
      text.ns = "lidar_tracklet_labels";
      text.id = marker_id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position = toPoint(track.position);
      text.pose.position.z += 0.45;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.18;
      text.color.r = 1.0F;
      text.color.g = 1.0F;
      text.color.b = 1.0F;
      text.color.a = 1.0F;
      text.text = "id=" + std::to_string(track.id) +
        " c=" + shortFloat(track.confidence) +
        " h/m=" + std::to_string(track.hit_count) + "/" + std::to_string(track.miss_count) +
        (track.miss_count == 0 ? " measured" : " predicted");
      text.lifetime = rclcpp::Duration::from_seconds(0.35);
      markers.markers.emplace_back(text);
    }
    marker_pub_->publish(markers);
  }

  std::string shortFloat(const double value) const {
    char buffer[16];
    std::snprintf(buffer, sizeof(buffer), "%.2f", value);
    return std::string(buffer);
  }

  void publishDebug(
    const rclcpp::Time &now,
    const std::string &status,
    const size_t raw_points,
    const size_t cropped_points,
    const size_t downsampled_points,
    const size_t detection_count,
    const double processing_ms = 0.0) {
    if ((now - last_debug_time_).seconds() < 1.0 / std::max(debug_publish_rate_, 0.1)) {
      return;
    }
    last_debug_time_ = now;

    size_t confirmed_count = 0;
    for (const auto &track : tracklets_) {
      if (track.confirmed && track.confidence >= min_publish_confidence_) {
        ++confirmed_count;
      }
    }

    std_msgs::msg::String msg;
    msg.data =
      std::string("{\"status\":\"") + status + "\"" +
      ",\"raw_points\":" + std::to_string(raw_points) +
      ",\"cropped_points\":" + std::to_string(cropped_points) +
      ",\"downsampled_points\":" + std::to_string(downsampled_points) +
      ",\"cluster_count\":" + std::to_string(detection_count) +
      ",\"eligible_cluster_count\":" + std::to_string(latest_eligible_cluster_count_) +
      ",\"weak_sparse_cluster_count\":" + std::to_string(latest_weak_sparse_cluster_count_) +
      ",\"oversized_cluster_count\":" + std::to_string(latest_oversized_cluster_count_) +
      ",\"hidden_impossible_cluster_count\":" +
        std::to_string(latest_hidden_impossible_cluster_count_) +
      ",\"tracklet_count\":" + std::to_string(tracklets_.size()) +
      ",\"confirmed_tracklet_count\":" + std::to_string(confirmed_count) +
      ",\"source_epoch_id\":" + std::to_string(source_epoch_->value()) +
      ",\"semantic_tracklet_drop_count\":" +
        std::to_string(latest_semantic_drop_count_) +
      ",\"processing_ms\":" + shortFloat(processing_ms) +
      "}";
    debug_pub_->publish(msg);
  }

  std::string lidar_topic_;
  std::string output_topic_;
  std::string semantic_output_topic_;
  std::string marker_topic_;
  std::string candidate_clusters_topic_;
  std::string candidate_marker_topic_;
  std::string debug_topic_;
  std::string lidar_qos_reliability_;
  std::string tracking_frame_;
  std::string base_frame_;
  std::string map_frame_;

  double min_range_{0.5};
  double max_range_{8.0};
  double min_z_{-0.25};
  double max_z_{2.2};
  double ground_z_threshold_{-0.15};
  bool remove_ground_{true};
  double voxel_leaf_size_{0.10};
  int max_points_before_clustering_{12000};
  int process_every_n_clouds_{2};

  double cluster_tolerance_{0.38};
  int cluster_core_min_points_{3};
  int generic_tracklet_min_points_{5};
  int camera_associated_min_points_{2};
  int max_cluster_points_{2500};
  double near_range_limit_{2.0};
  double mid_range_limit_{6.0};
  int near_min_cluster_points_{3};
  int mid_min_cluster_points_{6};
  int far_min_cluster_points_{2};
  double near_min_bbox_height_{0.10};
  double mid_min_bbox_height_{0.25};
  double far_min_bbox_height_{0.05};
  double impossible_candidate_width_{3.0};
  double impossible_candidate_depth_{3.0};
  double impossible_candidate_height_{2.8};
  double min_bbox_height_{0.15};
  double max_bbox_height_{2.2};
  double min_bbox_width_{0.05};
  double max_bbox_width_{1.6};
  double max_bbox_depth_{1.6};
  bool show_impossible_raw_clusters_{false};
  double max_visual_candidate_width_{3.0};
  double max_visual_candidate_depth_{3.0};
  double max_visual_candidate_height_{2.8};

  double tracklet_gating_distance_{1.2};
  int tracklet_confirm_hits_{3};
  int max_tracklet_missed_frames_{12};
  double max_tracklet_age_sec_{1.5};
  double max_tracklet_speed_mps_{2.2};
  double tracklet_process_noise_accel_std_{1.5};
  double tracklet_maneuver_noise_accel_std_{3.0};
  double tracklet_measurement_variance_{0.12};
  double tracklet_nis_gate_{9.21};
  double stationary_displacement_m_{0.10};
  int stationary_confirm_frames_{3};
  double stationary_velocity_decay_{0.5};
  double stationary_speed_threshold_mps_{0.08};
  double stationary_measurement_speed_mps_{0.25};
  double confidence_hit_increment_{0.15};
  double confidence_miss_decay_{0.06};
  double min_publish_confidence_{0.10};
  double marker_publish_rate_{10.0};
  double candidate_marker_publish_rate_{5.0};
  double debug_publish_rate_{2.0};

  uint64_t cloud_count_{0};
  int next_tracklet_id_{1};
  size_t latest_cluster_count_{0};
  size_t latest_eligible_cluster_count_{0};
  size_t latest_weak_sparse_cluster_count_{0};
  size_t latest_oversized_cluster_count_{0};
  size_t latest_hidden_impossible_cluster_count_{0};
  uint32_t latest_semantic_drop_count_{0};
  rclcpp::Time last_debug_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_candidate_marker_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_measurement_stamp_{0, 0, RCL_ROS_TIME};
  PointXYZI current_robot_origin_;

  std::vector<Tracklet> tracklets_;
  track_robot_interfaces::msg::LidarTrackletArray latest_tracklet_msg_;
  std::unique_ptr<track_robot_lidar_tracking::SourceEpoch> source_epoch_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<track_robot_interfaces::msg::LidarTrackletArray>::SharedPtr tracklet_pub_;
  rclcpp::Publisher<track_robot_interfaces::msg::SemanticLidarTrackletArray>::SharedPtr
    semantic_tracklet_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_marker_pub_;
  rclcpp::Publisher<track_robot_interfaces::msg::LidarClusterArray>::SharedPtr clusters_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarTrackletManagerNode>());
  rclcpp::shutdown();
  return 0;
}
