#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <track_robot_interfaces/msg/human_detection2_d.hpp>
#include <track_robot_interfaces/msg/human_detection2_d_array.hpp>
#include <track_robot_interfaces/msg/lidar_cluster_array.hpp>
#include <track_robot_interfaces/msg/lidar_tracklet.hpp>
#include <track_robot_interfaces/msg/lidar_tracklet_array.hpp>
#include <track_robot_interfaces/msg/selected_lidar_tracklet.hpp>
#include <track_robot_interfaces/msg/target_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

namespace {

struct PointXYZI {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
};

struct BBox2D {
  double x1{0.0};
  double y1{0.0};
  double x2{0.0};
  double y2{0.0};
};

struct Projection {
  bool valid{false};
  double u{0.0};
  double v{0.0};
  BBox2D box;
};

struct AnchorMeasurement {
  bool valid{false};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d marker_center{Eigen::Vector3d::Zero()};
  Eigen::Vector3d marker_size{Eigen::Vector3d(0.55, 0.55, 1.35)};
  double quality{0.0};
  size_t point_count{0};
  double depth_spread{0.0};
  double prediction_distance{0.0};
  std::string roi_type{"none"};
  std::string status{"not_run"};
};

struct ProcessingTiming {
  double cloud_parse_ms{0.0};
  double projection_ms{0.0};
  double association_ms{0.0};
  double kalman_ms{0.0};
  double publish_ms{0.0};
  bool fresh_cloud_processed{false};
};

struct TrackletMatch {
  bool valid{false};
  track_robot_interfaces::msg::LidarTracklet tracklet;
  double score{0.0};
  double nis_xy{0.0};
  std::string reason{"none"};
};

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d &value) {
  geometry_msgs::msg::Point point;
  point.x = value.x();
  point.y = value.y();
  point.z = value.z();
  return point;
}

geometry_msgs::msg::Vector3 toVector(const Eigen::Vector3d &value) {
  geometry_msgs::msg::Vector3 vector;
  vector.x = value.x();
  vector.y = value.y();
  vector.z = value.z();
  return vector;
}

Eigen::Vector3d fromPoint(const geometry_msgs::msg::Point &point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

double normXY(const Eigen::Vector3d &point) {
  return std::hypot(point.x(), point.y());
}

double clamp01(const double value) {
  return std::max(0.0, std::min(1.0, value));
}

double iou(const BBox2D &a, const BBox2D &b) {
  const double x1 = std::max(a.x1, b.x1);
  const double y1 = std::max(a.y1, b.y1);
  const double x2 = std::min(a.x2, b.x2);
  const double y2 = std::min(a.y2, b.y2);
  const double inter = std::max(0.0, x2 - x1) * std::max(0.0, y2 - y1);
  const double area_a = std::max(0.0, a.x2 - a.x1) * std::max(0.0, a.y2 - a.y1);
  const double area_b = std::max(0.0, b.x2 - b.x1) * std::max(0.0, b.y2 - b.y1);
  const double denom = area_a + area_b - inter;
  return denom > 0.0 ? inter / denom : 0.0;
}

bool inside(const double u, const double v, const BBox2D &box) {
  return u >= box.x1 && u <= box.x2 && v >= box.y1 && v <= box.y2;
}

double percentile(std::vector<double> values, const double percent) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double clamped = std::max(0.0, std::min(100.0, percent));
  const double index = (clamped / 100.0) * static_cast<double>(values.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(index));
  const size_t hi = static_cast<size_t>(std::ceil(index));
  const double alpha = index - static_cast<double>(lo);
  return values[lo] * (1.0 - alpha) + values[hi] * alpha;
}

std::string shortFloat(const double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", value);
  return std::string(buffer);
}

}  // namespace

class SelectedHumanTargetTrackerNode : public rclcpp::Node {
 public:
  SelectedHumanTargetTrackerNode()
  : Node("selected_human_target_tracker_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_) {
    camera_target_topic_ = declare_parameter<std::string>(
      "camera_target_topic", "/human_tracking/camera_target");
    detections_topic_ = declare_parameter<std::string>(
      "detections_topic", "/human_tracking/detections");
    lidar_tracklets_topic_ = declare_parameter<std::string>(
      "lidar_tracklets_topic", "/human_tracking/lidar_tracklets");
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/rslidar_points");
    candidate_clusters_topic_ = declare_parameter<std::string>(
      "candidate_clusters_topic", "/human_tracking/lidar_candidate_clusters");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/zed/zed_node/left/camera_info");

    selected_tracklet_topic_ = declare_parameter<std::string>(
      "selected_tracklet_topic", "/human_tracking/selected_lidar_tracklet");
    fused_target_topic_ = declare_parameter<std::string>(
      "fused_target_topic", "/human_tracking/fused_target_state");
    compat_target_topic_ = declare_parameter<std::string>(
      "compat_target_topic", "/human_tracking/target_state");
    selected_tracklet_marker_topic_ = declare_parameter<std::string>(
      "selected_tracklet_marker_topic", "/human_tracking/selected_tracklet_marker");
    selected_target_marker_topic_ = declare_parameter<std::string>(
      "selected_target_marker_topic", "/human_tracking/selected_target_marker");
    prediction_gate_marker_topic_ = declare_parameter<std::string>(
      "prediction_gate_marker_topic", "/human_tracking/target_prediction_gate_marker");
    fused_marker_topic_ = declare_parameter<std::string>(
      "fused_marker_topic", "/human_tracking/fused_target_marker");
    camera_guided_points_topic_ = declare_parameter<std::string>(
      "camera_guided_points_topic", "/human_tracking/camera_guided_target_points");
    debug_topic_ = declare_parameter<std::string>(
      "debug_topic", "/human_tracking/target_tracker_debug");

    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "rslidar");
    camera_frame_ = declare_parameter<std::string>(
      "camera_frame", "zed_left_camera_optical_frame");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");

    direct_optical_parent_frame_ = declare_parameter<std::string>(
      "direct_camera_lidar_optical_parent_frame", camera_frame_);
    direct_optical_child_frame_ = declare_parameter<std::string>(
      "direct_camera_lidar_optical_child_frame", lidar_frame_);
    direct_optical_translation_ = Eigen::Vector3d(
      declare_parameter<double>("direct_camera_lidar_optical_x", 0.06),
      declare_parameter<double>("direct_camera_lidar_optical_y", -0.065),
      declare_parameter<double>("direct_camera_lidar_optical_z", -0.26));
    direct_optical_qx_ = declare_parameter<double>("direct_camera_lidar_optical_qx", 0.5);
    direct_optical_qy_ = declare_parameter<double>("direct_camera_lidar_optical_qy", -0.5);
    direct_optical_qz_ = declare_parameter<double>("direct_camera_lidar_optical_qz", 0.5);
    direct_optical_qw_ = declare_parameter<double>("direct_camera_lidar_optical_qw", 0.5);
    direct_optical_rotation_ = quaternionToRotation(
      direct_optical_qx_, direct_optical_qy_, direct_optical_qz_, direct_optical_qw_);

    camera_visible_min_confidence_ = declare_parameter<double>(
      "camera_visible_min_confidence", 0.35);
    min_association_score_ = declare_parameter<double>("min_association_score", 0.55);
    association_confirm_frames_ = declare_parameter<int>("association_confirm_frames", 2);
    max_projection_center_error_px_ = declare_parameter<double>(
      "max_projection_center_error_px", 220.0);
    camera_target_timeout_sec_ = declare_parameter<double>("camera_target_timeout_sec", 1.0);
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 1.0);
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);
    debug_rate_ = declare_parameter<double>("debug_rate", 2.0);

    min_range_ = declare_parameter<double>("min_range", 0.5);
    max_range_ = declare_parameter<double>("max_range", 10.0);
    min_z_ = declare_parameter<double>("min_z", -0.25);
    max_z_ = declare_parameter<double>("max_z", 2.2);
    max_cloud_points_for_projection_ = declare_parameter<int>(
      "max_cloud_points_for_projection", 15000);
    debug_cloud_max_points_ = declare_parameter<int>("debug_cloud_max_points", 3000);
    camera_guided_min_points_ = declare_parameter<int>("camera_guided_min_points", 3);
    camera_guided_roi_center_width_fraction_ = declare_parameter<double>(
      "camera_guided_roi_center_width_fraction", 0.50);
    camera_guided_roi_y_min_fraction_ = declare_parameter<double>(
      "camera_guided_roi_y_min_fraction", 0.20);
    camera_guided_roi_y_max_fraction_ = declare_parameter<double>(
      "camera_guided_roi_y_max_fraction", 0.70);
    camera_guided_depth_percentile_low_ = declare_parameter<double>(
      "camera_guided_depth_percentile_low", 20.0);
    camera_guided_depth_percentile_high_ = declare_parameter<double>(
      "camera_guided_depth_percentile_high", 55.0);
    min_keypoint_confidence_ = declare_parameter<double>("min_keypoint_confidence", 0.35);
    camera_guided_max_depth_spread_ = declare_parameter<double>(
      "camera_guided_max_depth_spread", 1.25);
    camera_guided_prediction_gate_m_ = declare_parameter<double>(
      "camera_guided_prediction_gate_m", 1.5);

    process_noise_accel_std_ = declare_parameter<double>("process_noise_accel_std", 1.2);
    initial_position_variance_ = declare_parameter<double>("initial_position_variance", 0.35);
    initial_velocity_variance_ = declare_parameter<double>("initial_velocity_variance", 1.0);
    camera_anchor_xy_variance_ = declare_parameter<double>("camera_anchor_xy_variance", 0.08);
    tracklet_xy_variance_ = declare_parameter<double>("tracklet_xy_variance", 0.18);
    z_variance_ = declare_parameter<double>("z_variance", 0.35);
    max_camera_anchor_nis_xy_ = declare_parameter<double>("max_camera_anchor_nis_xy", 25.0);
    max_tracklet_nis_xy_ = declare_parameter<double>("max_tracklet_nis_xy", 9.21);
    prediction_gate_radius_m_ = declare_parameter<double>("prediction_gate_radius_m", 1.2);
    max_prediction_gate_radius_m_ = declare_parameter<double>(
      "max_prediction_gate_radius_m", 2.0);
    selected_exact_id_grace_sec_ = declare_parameter<double>(
      "selected_exact_id_grace_sec", 0.5);
    selected_relink_confirm_frames_ = declare_parameter<int>(
      "selected_relink_confirm_frames", 3);
    selected_relink_timeout_sec_ = declare_parameter<double>(
      "selected_relink_timeout_sec", 1.5);
    selected_relink_absolute_gate_m_ = declare_parameter<double>(
      "selected_relink_absolute_gate_m", 1.2);
    selected_relink_min_score_ = declare_parameter<double>(
      "selected_relink_min_score", 0.55);
    prediction_velocity_damping_delay_sec_ = declare_parameter<double>(
      "prediction_velocity_damping_delay_sec", 0.4);
    prediction_velocity_damping_rate_ = declare_parameter<double>(
      "prediction_velocity_damping_rate", 0.8);
    prediction_only_timeout_sec_ = declare_parameter<double>(
      "prediction_only_timeout_sec", 2.0);
    max_target_speed_mps_ = declare_parameter<double>("max_target_speed_mps", 2.2);
    camera_projection_max_rate_hz_ = declare_parameter<double>(
      "camera_projection_max_rate_hz", 10.0);

    camera_target_sub_ = create_subscription<track_robot_interfaces::msg::TargetState>(
      camera_target_topic_, 5,
      std::bind(&SelectedHumanTargetTrackerNode::cameraTargetCallback, this, std::placeholders::_1));
    detections_sub_ = create_subscription<track_robot_interfaces::msg::HumanDetection2DArray>(
      detections_topic_, 5,
      std::bind(&SelectedHumanTargetTrackerNode::detectionsCallback, this, std::placeholders::_1));
    tracklets_sub_ = create_subscription<track_robot_interfaces::msg::LidarTrackletArray>(
      lidar_tracklets_topic_, 5,
      std::bind(&SelectedHumanTargetTrackerNode::trackletsCallback, this, std::placeholders::_1));
    candidates_sub_ = create_subscription<track_robot_interfaces::msg::LidarClusterArray>(
      candidate_clusters_topic_, 5,
      std::bind(&SelectedHumanTargetTrackerNode::candidatesCallback, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 5,
      std::bind(&SelectedHumanTargetTrackerNode::cameraInfoCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SelectedHumanTargetTrackerNode::cloudCallback, this, std::placeholders::_1));

    selected_pub_ = create_publisher<track_robot_interfaces::msg::SelectedLidarTracklet>(
      selected_tracklet_topic_, 5);
    fused_pub_ = create_publisher<track_robot_interfaces::msg::TargetState>(
      fused_target_topic_, 5);
    compat_pub_ = create_publisher<track_robot_interfaces::msg::TargetState>(
      compat_target_topic_, 5);
    selected_tracklet_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(selected_tracklet_marker_topic_, 5);
    selected_target_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(selected_target_marker_topic_, 5);
    prediction_gate_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(prediction_gate_marker_topic_, 5);
    fused_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(fused_marker_topic_, 5);
    camera_guided_points_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(camera_guided_points_topic_, 3);
    debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, 5);

    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_))),
      std::bind(&SelectedHumanTargetTrackerNode::periodicPublish, this));

    RCLCPP_INFO(
      get_logger(),
      "selected_human_target_tracker_node: camera_target=%s tracklets=%s output=%s frame=%s",
      camera_target_topic_.c_str(), lidar_tracklets_topic_.c_str(),
      fused_target_topic_.c_str(), base_frame_.c_str());
  }

 private:
  void cameraTargetCallback(const track_robot_interfaces::msg::TargetState::SharedPtr msg) {
    latest_camera_target_ = *msg;
    last_camera_target_time_ = now();
    if (msg->target_id < 0 ||
      msg->lock_state == track_robot_interfaces::msg::TargetState::LOCK_NO_TARGET ||
      msg->lock_state == track_robot_interfaces::msg::TargetState::LOCK_CANDIDATE_VISIBLE) {
      clearTarget();
    }
  }

  void detectionsCallback(
    const track_robot_interfaces::msg::HumanDetection2DArray::SharedPtr msg) {
    latest_detections_ = *msg;
    last_detections_time_ = now();
  }

  void trackletsCallback(const track_robot_interfaces::msg::LidarTrackletArray::SharedPtr msg) {
    latest_tracklets_ = *msg;
    last_tracklets_time_ = now();
    update("tracklet_event");
  }

  void candidatesCallback(const track_robot_interfaces::msg::LidarClusterArray::SharedPtr msg) {
    latest_candidates_ = *msg;
    last_candidates_time_ = now();
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    latest_camera_info_ = *msg;
    have_camera_info_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    latest_cloud_ = *msg;
    have_cloud_ = true;
    last_cloud_time_ = now();
    const double minimum_period = 1.0 / std::max(1.0, camera_projection_max_rate_hz_);
    if ((!last_camera_projection_time_.nanoseconds() ||
      (last_cloud_time_ - last_camera_projection_time_).seconds() >= minimum_period) &&
      cameraVisible(last_cloud_time_)) {
      last_camera_projection_time_ = last_cloud_time_;
      update("cloud_event");
    }
  }

  void periodicPublish() {
    update("timer");
  }

  void update(const std::string &trigger) {
    const auto update_start = std::chrono::steady_clock::now();
    const rclcpp::Time stamp = now();
    predictFilter(stamp);

    AnchorMeasurement anchor;
    TrackletMatch match;
    std::string measurement_source = "none";
    std::string rejection_reason = "none";
    bool measurement_accepted = false;
    double selected_score = 0.0;
    double nis_xy = 0.0;
    ProcessingTiming timing;
    const bool identity_active = cameraTargetValid(stamp);
    const bool visible = cameraVisible(stamp);

    if (visible) {
      last_camera_seen_time_ = stamp;
      if (trigger == "tracklet_event") {
        const auto association_start = std::chrono::steady_clock::now();
        match = bestVisibleTrackletMatch();
        if (match.valid && match.score >= min_association_score_) {
          updatePendingSelection(match.tracklet.tracklet_id);
          selected_score = match.score;
          updateSelectedReference(match.tracklet);
        } else {
          pending_tracklet_id_ = -1;
          pending_count_ = 0;
        }
        timing.association_ms = elapsedMs(association_start);
      }

      if (trigger == "cloud_event") {
        timing.fresh_cloud_processed = true;
        anchor = computeCameraGuidedAnchor(stamp, timing);
      }
      const auto kalman_start = std::chrono::steady_clock::now();
      if (trigger == "cloud_event" && anchor.valid) {
        const double variance_scale = 1.0 / std::max(0.05, anchor.quality);
        measurement_accepted = correctFilter(
          anchor.position,
          camera_anchor_xy_variance_ * variance_scale,
          z_variance_,
          max_camera_anchor_nis_xy_,
          nis_xy);
        measurement_source = "camera_guided_anchor";
        if (!measurement_accepted) {
          rejection_reason = "camera_anchor_nis_gate";
        } else {
          camera_guided_marker_offset_ = anchor.marker_center - anchor.position;
          camera_guided_marker_size_ = anchor.marker_size;
        }
      }

      if (trigger == "tracklet_event" && selected_tracklet_id_ >= 0) {
        auto selected = findTracklet(selected_tracklet_id_);
        if (selected.has_value() && selected->active) {
          measurement_accepted = correctFilter(
            fromPoint(selected->position_base),
            tracklet_xy_variance_,
            z_variance_,
            max_tracklet_nis_xy_,
            nis_xy);
          measurement_source = "selected_tracklet";
          if (!measurement_accepted) {
            rejection_reason = "selected_tracklet_nis_gate";
          }
          if (measurement_accepted) {
            updateSelectedReference(*selected);
          }
        }
      }
      timing.kalman_ms = elapsedMs(kalman_start);
      if (measurement_accepted) {
        last_lidar_seen_time_ = stamp;
        last_measurement_source_ = "camera_lidar";
        kf_confidence_ = std::max(kf_confidence_, std::max(anchor.quality, selected_score));
      }
    } else if (identity_active && kf_initialized_ && trigger == "tracklet_event") {
      const auto association_start = std::chrono::steady_clock::now();
      measurement_accepted = updateFromLidarOnly(
        stamp, measurement_source, rejection_reason, selected_score, nis_xy);
      timing.association_ms = elapsedMs(association_start);
      if (measurement_accepted) {
        last_lidar_seen_time_ = stamp;
        last_measurement_source_ = "lidar_only";
        kf_confidence_ = std::min(1.0, kf_confidence_ + 0.10);
      }
    }

    const std::string state_name = determineState(stamp, identity_active, visible);
    if (!measurement_accepted && state_name == "prediction_only") {
      measurement_source = "prediction";
    }
    publishOutputs(
      stamp, state_name, measurement_source, rejection_reason, measurement_accepted,
      selected_score, nis_xy, anchor, trigger, update_start, timing);
  }

  bool cameraTargetValid(const rclcpp::Time &stamp) const {
    if (!last_camera_target_time_.nanoseconds()) {
      return false;
    }
    if (latest_camera_target_.target_id < 0) {
      return false;
    }
    if (latest_camera_target_.lock_state !=
      track_robot_interfaces::msg::TargetState::LOCK_TARGET_LOCKED &&
      latest_camera_target_.lock_state !=
      track_robot_interfaces::msg::TargetState::LOCK_TARGET_LOST) {
      return false;
    }
    return (stamp - last_camera_target_time_).seconds() <= camera_target_timeout_sec_;
  }

  bool cameraVisible(const rclcpp::Time &stamp) const {
    return cameraTargetValid(stamp) &&
      latest_camera_target_.lock_state ==
        track_robot_interfaces::msg::TargetState::LOCK_TARGET_LOCKED &&
      latest_camera_target_.camera_visible &&
      latest_camera_target_.confidence >= camera_visible_min_confidence_;
  }

  TrackletMatch bestVisibleTrackletMatch() {
    TrackletMatch best;
    if (!have_camera_info_ || latest_tracklets_.tracklets.empty()) {
      best.reason = "missing_camera_info_or_tracklets";
      return best;
    }

    const BBox2D camera_box = cameraTargetBox();
    const Eigen::Vector2d camera_center(
      0.5 * (camera_box.x1 + camera_box.x2),
      0.5 * (camera_box.y1 + camera_box.y2));

    for (const auto &tracklet : latest_tracklets_.tracklets) {
      const auto projection = projectTracklet(tracklet);
      if (!projection.valid) {
        continue;
      }
      const Eigen::Vector2d projected_center(projection.u, projection.v);
      const double center_error = (projected_center - camera_center).norm();
      const double center_score =
        clamp01(1.0 - center_error / std::max(1.0, max_projection_center_error_px_));
      const double inside_score = inside(projection.u, projection.v, camera_box) ? 1.0 : 0.0;
      const double overlap_score = iou(camera_box, projection.box);
      const double temporal_score =
        tracklet.tracklet_id == selected_tracklet_id_ ? 1.0 : 0.0;
      const double confidence_score = clamp01(tracklet.confidence);
      const double size_score = humanSizeScore(tracklet);
      const double nis_score = kalmanNisScore(fromPoint(tracklet.position_base));

      const double score =
        0.30 * overlap_score +
        0.20 * inside_score +
        0.20 * nis_score +
        0.15 * temporal_score +
        0.10 * confidence_score +
        0.05 * size_score;

      if (score > best.score) {
        best.valid = true;
        best.tracklet = tracklet;
        best.score = score;
        best.nis_xy = last_projected_nis_xy_;
        best.reason = "best_projected_tracklet";
      }
    }
    return best;
  }

  double humanSizeScore(const track_robot_interfaces::msg::LidarTracklet &tracklet) const {
    const double width = std::max(tracklet.size.x, tracklet.size.y);
    const double depth = std::min(tracklet.size.x, tracklet.size.y);
    const double height = tracklet.size.z;
    const double width_score = clamp01(1.0 - std::abs(width - 0.55) / 1.2);
    const double depth_score = clamp01(1.0 - std::abs(depth - 0.35) / 1.0);
    const double height_score = clamp01(1.0 - std::abs(height - 1.2) / 1.8);
    return 0.4 * width_score + 0.25 * depth_score + 0.35 * height_score;
  }

  double kalmanNisScore(const Eigen::Vector3d &measurement) {
    if (!kf_initialized_) {
      last_projected_nis_xy_ = 0.0;
      return 0.7;
    }
    const Eigen::Vector2d residual = measurement.head<2>() - kf_x_.head<2>();
    Eigen::Matrix2d covariance = kf_p_.block<2, 2>(0, 0);
    covariance(0, 0) += tracklet_xy_variance_;
    covariance(1, 1) += tracklet_xy_variance_;
    const double nis = residual.transpose() * covariance.inverse() * residual;
    last_projected_nis_xy_ = nis;
    return clamp01(1.0 - nis / std::max(1e-6, max_tracklet_nis_xy_));
  }

  void updatePendingSelection(const int tracklet_id) {
    if (selected_tracklet_id_ == tracklet_id) {
      pending_tracklet_id_ = tracklet_id;
      pending_count_ = association_confirm_frames_;
      return;
    }
    if (pending_tracklet_id_ == tracklet_id) {
      ++pending_count_;
    } else {
      pending_tracklet_id_ = tracklet_id;
      pending_count_ = 1;
    }
    if (pending_count_ >= association_confirm_frames_) {
      selected_tracklet_id_ = tracklet_id;
      pending_count_ = association_confirm_frames_;
    }
  }

  std::optional<track_robot_interfaces::msg::LidarTracklet> findTracklet(const int id) const {
    for (const auto &tracklet : latest_tracklets_.tracklets) {
      if (tracklet.tracklet_id == id) {
        return tracklet;
      }
    }
    return std::nullopt;
  }

  void updateSelectedReference(const track_robot_interfaces::msg::LidarTracklet &tracklet) {
    if (tracklet.tracklet_id != selected_tracklet_id_) {
      return;
    }
    last_selected_size_ = Eigen::Vector3d(
      tracklet.size.x, tracklet.size.y, tracklet.size.z);
    last_selected_point_count_ = tracklet.point_count;
    have_selected_reference_ = true;
  }

  double measurementNisXY(const Eigen::Vector3d &measurement) const {
    if (!kf_initialized_) {
      return 0.0;
    }
    const Eigen::Vector2d residual = measurement.head<2>() - kf_x_.head<2>();
    Eigen::Matrix2d innovation = kf_p_.block<2, 2>(0, 0);
    innovation.diagonal().array() += tracklet_xy_variance_;
    return residual.transpose() * innovation.inverse() * residual;
  }

  double relinkScore(const track_robot_interfaces::msg::LidarTracklet &tracklet) const {
    if (!tracklet.active || !tracklet.confirmed) {
      return -1.0;
    }
    const Eigen::Vector3d position = fromPoint(tracklet.position_base);
    const double distance = (position.head<2>() - kf_x_.head<2>()).norm();
    const double nis = measurementNisXY(position);
    if (distance > selected_relink_absolute_gate_m_ || nis > max_tracklet_nis_xy_) {
      return -1.0;
    }
    const double nis_score = clamp01(1.0 - nis / std::max(1e-6, max_tracklet_nis_xy_));
    double size_score = 0.6;
    double point_score = 0.6;
    if (have_selected_reference_) {
      const Eigen::Vector3d size(tracklet.size.x, tracklet.size.y, tracklet.size.z);
      const double relative_size =
        (size - last_selected_size_).cwiseAbs().sum() /
        std::max(0.2, last_selected_size_.sum());
      size_score = clamp01(1.0 - relative_size / 1.5);
      const double point_ratio = std::abs(std::log(
        (static_cast<double>(tracklet.point_count) + 1.0) /
        (static_cast<double>(last_selected_point_count_) + 1.0)));
      point_score = clamp01(1.0 - point_ratio / std::log(4.0));
    }
    const Eigen::Vector2d filter_velocity = kf_x_.segment<2>(3);
    const Eigen::Vector2d tracklet_velocity(tracklet.velocity.x, tracklet.velocity.y);
    double velocity_score = 0.7;
    if (filter_velocity.norm() > 0.2 && tracklet_velocity.norm() > 0.2) {
      velocity_score = clamp01(
        0.5 + 0.5 * filter_velocity.dot(tracklet_velocity) /
        (filter_velocity.norm() * tracklet_velocity.norm()));
    }
    return
      0.55 * nis_score +
      0.20 * size_score +
      0.15 * point_score +
      0.10 * velocity_score;
  }

  bool updateFromLidarOnly(
    const rclcpp::Time &stamp,
    std::string &measurement_source,
    std::string &rejection_reason,
    double &score,
    double &nis_xy) {
    auto selected = findTracklet(selected_tracklet_id_);
    if (selected.has_value() && selected->active) {
      const bool accepted = correctFilter(
        fromPoint(selected->position_base),
        tracklet_xy_variance_,
        z_variance_,
        max_tracklet_nis_xy_,
        nis_xy);
      measurement_source = "selected_tracklet_lidar_only";
      if (accepted) {
        selected_missing_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        relink_tracklet_id_ = -1;
        relink_count_ = 0;
        score = 1.0;
        updateSelectedReference(*selected);
        return true;
      }
      rejection_reason = "selected_tracklet_nis_gate";
      return false;
    }

    if (!selected_missing_since_.nanoseconds()) {
      selected_missing_since_ = stamp;
    }
    const double missing_sec = (stamp - selected_missing_since_).seconds();
    if (missing_sec < selected_exact_id_grace_sec_) {
      rejection_reason = "selected_tracklet_grace";
      return false;
    }
    if (missing_sec > selected_relink_timeout_sec_) {
      rejection_reason = "selected_tracklet_relink_timeout";
      return false;
    }

    int best_id = -1;
    double best_score = selected_relink_min_score_;
    std::optional<track_robot_interfaces::msg::LidarTracklet> best;
    for (const auto &candidate : latest_tracklets_.tracklets) {
      if (candidate.tracklet_id == selected_tracklet_id_) {
        continue;
      }
      const double candidate_score = relinkScore(candidate);
      if (candidate_score > best_score) {
        best_id = candidate.tracklet_id;
        best_score = candidate_score;
        best = candidate;
      }
    }
    if (!best.has_value()) {
      relink_tracklet_id_ = -1;
      relink_count_ = 0;
      rejection_reason = "no_compatible_local_relink";
      return false;
    }
    if (relink_tracklet_id_ == best_id) {
      ++relink_count_;
    } else {
      relink_tracklet_id_ = best_id;
      relink_count_ = 1;
    }
    score = best_score;
    measurement_source = "pending_local_relink";
    if (relink_count_ < selected_relink_confirm_frames_) {
      rejection_reason = "local_relink_pending";
      return false;
    }

    selected_tracklet_id_ = best_id;
    pending_tracklet_id_ = best_id;
    pending_count_ = association_confirm_frames_;
    selected_missing_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    relink_tracklet_id_ = -1;
    relink_count_ = 0;
    const bool accepted = correctFilter(
      fromPoint(best->position_base),
      tracklet_xy_variance_,
      z_variance_,
      max_tracklet_nis_xy_,
      nis_xy);
    measurement_source = "strict_local_relink";
    if (!accepted) {
      rejection_reason = "relinked_tracklet_nis_gate";
      return false;
    }
    updateSelectedReference(*best);
    return true;
  }

  std::string determineState(
    const rclcpp::Time &stamp,
    const bool identity_active,
    const bool camera_visible) const {
    if (!identity_active) {
      return "none";
    }
    const double since_lidar = timeSince(last_lidar_seen_time_, stamp);
    if (camera_visible) {
      if (since_lidar <= input_timeout_sec_) {
        return "camera_lidar";
      }
      return kf_initialized_ ? "prediction_only" : "camera_only";
    }
    if (last_measurement_source_ == "lidar_only" && since_lidar <= input_timeout_sec_) {
      return "lidar_only";
    }
    if (kf_initialized_ && since_lidar <= prediction_only_timeout_sec_) {
      return "prediction_only";
    }
    return "target_lost";
  }

  AnchorMeasurement computeCameraGuidedAnchor(
    const rclcpp::Time &stamp,
    ProcessingTiming &timing) {
    AnchorMeasurement result;
    if (!have_cloud_ || !have_camera_info_) {
      result.status = "missing_cloud_or_camera_info";
      publishCameraGuidedCloud({}, stamp);
      return result;
    }
    if ((stamp - last_cloud_time_).seconds() > input_timeout_sec_) {
      result.status = "stale_cloud";
      publishCameraGuidedCloud({}, stamp);
      return result;
    }

    const auto parse_start = std::chrono::steady_clock::now();
    std::vector<PointXYZI> raw_points;
    std::string parse_status;
    if (!readCloud(latest_cloud_, raw_points, parse_status) || raw_points.empty()) {
      timing.cloud_parse_ms = elapsedMs(parse_start);
      result.status = parse_status;
      publishCameraGuidedCloud({}, stamp);
      return result;
    }
    timing.cloud_parse_ms = elapsedMs(parse_start);

    if (static_cast<int>(raw_points.size()) > max_cloud_points_for_projection_) {
      const size_t stride = static_cast<size_t>(
        std::ceil(static_cast<double>(raw_points.size()) /
        std::max(1, max_cloud_points_for_projection_)));
      std::vector<PointXYZI> sampled;
      sampled.reserve(static_cast<size_t>(max_cloud_points_for_projection_));
      for (size_t i = 0; i < raw_points.size(); i += stride) {
        sampled.emplace_back(raw_points[i]);
      }
      raw_points.swap(sampled);
    }

    const auto projection_start = std::chrono::steady_clock::now();
    const std::string cloud_frame = latest_cloud_.header.frame_id.empty() ?
      lidar_frame_ : latest_cloud_.header.frame_id;
    std::vector<Eigen::Vector3d> base_points;
    std::vector<Eigen::Vector3d> camera_points;
    if (!transformPointSet(raw_points, cloud_frame, base_frame_, base_points)) {
      result.status = "missing_base_tf";
      publishCameraGuidedCloud({}, stamp);
      return result;
    }
    if (!transformPointSetToCamera(raw_points, cloud_frame, camera_points)) {
      result.status = "missing_camera_tf";
      publishCameraGuidedCloud({}, stamp);
      return result;
    }

    const BBox2D roi = targetBodyRoi(&result.roi_type);
    const auto intr = cameraIntrinsics();
    std::vector<Eigen::Vector3d> selected_base;
    std::vector<double> selected_depth;
    selected_base.reserve(base_points.size());
    selected_depth.reserve(base_points.size());

    for (size_t i = 0; i < base_points.size() && i < camera_points.size(); ++i) {
      const auto &bp = base_points[i];
      const auto &cp = camera_points[i];
      const double range = normXY(bp);
      if (range < min_range_ || range > max_range_ || bp.z() < min_z_ || bp.z() > max_z_) {
        continue;
      }
      if (cp.z() <= 0.05) {
        continue;
      }
      const double u = intr[0] * cp.x() / cp.z() + intr[2];
      const double v = intr[1] * cp.y() / cp.z() + intr[3];
      if (!inside(u, v, roi)) {
        continue;
      }
      selected_base.emplace_back(bp);
      selected_depth.emplace_back(cp.z());
    }

    if (selected_base.size() < static_cast<size_t>(camera_guided_min_points_)) {
      result.status = "too_few_roi_points";
      result.point_count = selected_base.size();
      publishCameraGuidedCloud(selected_base, stamp);
      return result;
    }

    const double low = percentile(selected_depth, camera_guided_depth_percentile_low_);
    const double high = percentile(selected_depth, camera_guided_depth_percentile_high_);
    std::vector<Eigen::Vector3d> filtered;
    std::vector<double> filtered_depth;
    for (size_t i = 0; i < selected_base.size(); ++i) {
      if (selected_depth[i] >= low && selected_depth[i] <= high) {
        filtered.emplace_back(selected_base[i]);
        filtered_depth.emplace_back(selected_depth[i]);
      }
    }
    if (filtered.size() < static_cast<size_t>(camera_guided_min_points_)) {
      filtered = selected_base;
      filtered_depth = selected_depth;
    }

    const Eigen::Vector3d anchor = medianPoint(filtered);
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    xs.reserve(filtered.size());
    ys.reserve(filtered.size());
    zs.reserve(filtered.size());
    for (const auto &point : filtered) {
      xs.emplace_back(point.x());
      ys.emplace_back(point.y());
      zs.emplace_back(point.z());
    }
    const Eigen::Vector3d robust_min(
      percentile(xs, 10.0), percentile(ys, 10.0), percentile(zs, 10.0));
    const Eigen::Vector3d robust_max(
      percentile(xs, 90.0), percentile(ys, 90.0), percentile(zs, 90.0));
    const double depth_spread =
      percentile(filtered_depth, 90.0) - percentile(filtered_depth, 10.0);
    const double prediction_distance =
      kf_initialized_ ? (anchor.head<2>() - kf_x_.head<2>()).norm() : 0.5;
    const double point_score = clamp01(
      static_cast<double>(filtered.size()) /
      std::max(1.0, 5.0 * static_cast<double>(camera_guided_min_points_)));
    const double spread_score = clamp01(
      1.0 - depth_spread / std::max(1e-6, camera_guided_max_depth_spread_));
    const double prediction_score = clamp01(
      1.0 - prediction_distance / std::max(1e-6, camera_guided_prediction_gate_m_));
    const double camera_score = clamp01(latest_camera_target_.confidence);
    result.quality =
      0.35 * point_score +
      0.25 * spread_score +
      0.25 * prediction_score +
      0.15 * camera_score;
    result.position = anchor;
    result.marker_center = 0.5 * (robust_min + robust_max);
    result.marker_size = Eigen::Vector3d(
      std::max(0.20, robust_max.x() - robust_min.x()),
      std::max(0.20, robust_max.y() - robust_min.y()),
      std::max(0.35, robust_max.z() - robust_min.z()));
    result.valid = result.quality >= 0.25;
    result.point_count = filtered.size();
    result.depth_spread = depth_spread;
    result.prediction_distance = prediction_distance;
    result.status = result.valid ? "ok" : "low_quality";
    publishCameraGuidedCloud(filtered, stamp);
    timing.projection_ms = elapsedMs(projection_start);
    return result;
  }

  Eigen::Vector3d medianPoint(const std::vector<Eigen::Vector3d> &points) const {
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    xs.reserve(points.size());
    ys.reserve(points.size());
    zs.reserve(points.size());
    for (const auto &point : points) {
      xs.emplace_back(point.x());
      ys.emplace_back(point.y());
      zs.emplace_back(point.z());
    }
    return Eigen::Vector3d(
      percentile(xs, 50.0), percentile(ys, 50.0), percentile(zs, 50.0));
  }

  BBox2D targetBodyRoi(std::string *roi_type) const {
    const BBox2D bbox = cameraTargetBox();
    const auto detection = matchedDetection();
    if (detection.has_value()) {
      const auto &keypoints = detection->keypoints;
      const auto valid = [&](const size_t index) {
        const size_t offset = index * 3;
        return keypoints.size() > offset + 2 && keypoints[offset + 2] >= min_keypoint_confidence_;
      };
      if (valid(5) && valid(6) && valid(11) && valid(12)) {
        if (roi_type) {
          *roi_type = "skeleton_torso";
        }
        std::vector<double> xs{
          keypoints[15], keypoints[18], keypoints[33], keypoints[36]};
        std::vector<double> ys{
          keypoints[16], keypoints[19], keypoints[34], keypoints[37]};
        return expandRoi(xs, ys, bbox, 0.30, 0.20);
      }
      if (valid(5) && valid(6)) {
        if (roi_type) {
          *roi_type = "skeleton_upper_body";
        }
        const double y_mid = 0.5 * (keypoints[16] + keypoints[19]);
        std::vector<double> xs{keypoints[15], keypoints[18]};
        std::vector<double> ys{y_mid, bbox.y1 + 0.62 * (bbox.y2 - bbox.y1)};
        return expandRoi(xs, ys, bbox, 0.45, 0.15);
      }
    }
    if (roi_type) {
      *roi_type = "central_bbox";
    }
    const double width = std::max(1.0, bbox.x2 - bbox.x1);
    const double height = std::max(1.0, bbox.y2 - bbox.y1);
    const double cx = 0.5 * (bbox.x1 + bbox.x2);
    const double half_width = 0.5 * camera_guided_roi_center_width_fraction_ * width;
    return BBox2D{
      cx - half_width,
      bbox.y1 + camera_guided_roi_y_min_fraction_ * height,
      cx + half_width,
      bbox.y1 + camera_guided_roi_y_max_fraction_ * height};
  }

  BBox2D expandRoi(
    const std::vector<double> &xs,
    const std::vector<double> &ys,
    const BBox2D &bbox,
    const double x_margin_fraction,
    const double y_margin_fraction) const {
    const auto minmax_x = std::minmax_element(xs.begin(), xs.end());
    const auto minmax_y = std::minmax_element(ys.begin(), ys.end());
    const double width = std::max(1.0, bbox.x2 - bbox.x1);
    const double height = std::max(1.0, bbox.y2 - bbox.y1);
    return BBox2D{
      std::max(bbox.x1, *minmax_x.first - x_margin_fraction * width),
      std::max(bbox.y1, *minmax_y.first - y_margin_fraction * height),
      std::min(bbox.x2, *minmax_x.second + x_margin_fraction * width),
      std::min(bbox.y2, *minmax_y.second + y_margin_fraction * height)};
  }

  std::optional<track_robot_interfaces::msg::HumanDetection2D> matchedDetection() const {
    const BBox2D target = cameraTargetBox();
    std::optional<track_robot_interfaces::msg::HumanDetection2D> best;
    double best_score = 0.0;
    const double tx = 0.5 * (target.x1 + target.x2);
    const double ty = 0.5 * (target.y1 + target.y2);
    const double diag = std::max(
      1.0, std::hypot(target.x2 - target.x1, target.y2 - target.y1));
    for (const auto &detection : latest_detections_.detections) {
      BBox2D box{
        detection.bbox[0], detection.bbox[1], detection.bbox[2], detection.bbox[3]};
      const double dx = 0.5 * (box.x1 + box.x2) - tx;
      const double dy = 0.5 * (box.y1 + box.y2) - ty;
      const double center_score = clamp01(1.0 - std::hypot(dx, dy) / diag);
      const double overlap = iou(target, box);
      const double score = 0.75 * overlap + 0.25 * center_score;
      if (score > best_score) {
        best = detection;
        best_score = score;
      }
    }
    return best_score > 0.05 ? best : std::nullopt;
  }

  bool correctFilter(
    const Eigen::Vector3d &measurement,
    const double xy_variance,
    const double z_variance,
    const double max_nis_xy,
    double &nis_xy) {
    if (!kf_initialized_) {
      kf_x_.setZero();
      kf_x_.head<3>() = measurement;
      kf_p_.setZero();
      kf_p_.diagonal() <<
        initial_position_variance_, initial_position_variance_, initial_position_variance_,
        initial_velocity_variance_, initial_velocity_variance_, initial_velocity_variance_;
      kf_initialized_ = true;
      last_filter_time_ = now();
      nis_xy = 0.0;
      return true;
    }

    Eigen::Matrix<double, 3, 6> h;
    h.setZero();
    h(0, 0) = 1.0;
    h(1, 1) = 1.0;
    h(2, 2) = 1.0;
    Eigen::Matrix3d r = Eigen::Matrix3d::Zero();
    r(0, 0) = std::max(1e-6, xy_variance);
    r(1, 1) = std::max(1e-6, xy_variance);
    r(2, 2) = std::max(1e-6, z_variance);
    const Eigen::Vector3d residual = measurement - h * kf_x_;
    Eigen::Matrix2d s_xy = kf_p_.block<2, 2>(0, 0);
    s_xy(0, 0) += r(0, 0);
    s_xy(1, 1) += r(1, 1);
    nis_xy = residual.head<2>().transpose() * s_xy.inverse() * residual.head<2>();
    if (nis_xy > max_nis_xy) {
      return false;
    }
    const Eigen::Matrix3d s = h * kf_p_ * h.transpose() + r;
    const Eigen::Matrix<double, 6, 3> k = kf_p_ * h.transpose() * s.inverse();
    kf_x_ = kf_x_ + k * residual;
    const Eigen::Matrix<double, 6, 6> identity =
      Eigen::Matrix<double, 6, 6>::Identity();
    const auto kh = k * h;
    kf_p_ = (identity - kh) * kf_p_ * (identity - kh).transpose() + k * r * k.transpose();
    kf_confidence_ = std::min(1.0, kf_confidence_ + 0.15);
    return true;
  }

  void predictFilter(const rclcpp::Time &stamp) {
    if (!kf_initialized_) {
      last_filter_time_ = stamp;
      return;
    }
    double dt = last_filter_time_.nanoseconds() ?
      (stamp - last_filter_time_).seconds() : 0.0;
    dt = std::max(0.0, std::min(0.5, dt));
    if (dt <= 1e-6) {
      return;
    }
    Eigen::Matrix<double, 6, 6> f = Eigen::Matrix<double, 6, 6>::Identity();
    f(0, 3) = dt;
    f(1, 4) = dt;
    f(2, 5) = dt;
    Eigen::Matrix<double, 6, 6> q = Eigen::Matrix<double, 6, 6>::Zero();
    const double accel_var = process_noise_accel_std_ * process_noise_accel_std_;
    for (int i = 0; i < 3; ++i) {
      q(i, i) = 0.25 * dt * dt * dt * dt * accel_var;
      q(i, i + 3) = 0.5 * dt * dt * dt * accel_var;
      q(i + 3, i) = 0.5 * dt * dt * dt * accel_var;
      q(i + 3, i + 3) = dt * dt * accel_var;
    }
    kf_x_ = f * kf_x_;
    kf_p_ = f * kf_p_ * f.transpose() + q;
    const double since_lidar = timeSince(last_lidar_seen_time_, stamp);
    if (since_lidar > prediction_only_timeout_sec_) {
      kf_x_.tail<3>().setZero();
    } else if (since_lidar > prediction_velocity_damping_delay_sec_) {
      const double damping = std::exp(-prediction_velocity_damping_rate_ * dt);
      kf_x_.tail<3>() *= damping;
    }
    const double speed_xy = kf_x_.segment<2>(3).norm();
    if (speed_xy > max_target_speed_mps_ && speed_xy > 1e-6) {
      kf_x_.segment<2>(3) *= max_target_speed_mps_ / speed_xy;
    }
    last_filter_time_ = stamp;
    kf_confidence_ = std::max(0.0, kf_confidence_ - 0.02 * dt);
  }

  Projection projectTracklet(const track_robot_interfaces::msg::LidarTracklet &tracklet) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(9);
    points.emplace_back(fromPoint(tracklet.position_base));
    const Eigen::Vector3d mn = fromPoint(tracklet.minimum);
    const Eigen::Vector3d mx = fromPoint(tracklet.maximum);
    for (int ix = 0; ix < 2; ++ix) {
      for (int iy = 0; iy < 2; ++iy) {
        for (int iz = 0; iz < 2; ++iz) {
          points.emplace_back(
            ix ? mx.x() : mn.x(),
            iy ? mx.y() : mn.y(),
            iz ? mx.z() : mn.z());
        }
      }
    }

    std::vector<Eigen::Vector3d> camera_points;
    if (!transformVectorsToCamera(points, base_frame_, camera_points)) {
      return Projection{};
    }
    return projectCameraPoints(camera_points);
  }

  Projection projectCameraPoints(const std::vector<Eigen::Vector3d> &camera_points) const {
    Projection projection;
    const auto intr = cameraIntrinsics();
    bool any_valid = false;
    double min_u = std::numeric_limits<double>::max();
    double min_v = std::numeric_limits<double>::max();
    double max_u = std::numeric_limits<double>::lowest();
    double max_v = std::numeric_limits<double>::lowest();
    for (size_t i = 0; i < camera_points.size(); ++i) {
      const auto &point = camera_points[i];
      if (point.z() <= 0.05) {
        continue;
      }
      const double u = intr[0] * point.x() / point.z() + intr[2];
      const double v = intr[1] * point.y() / point.z() + intr[3];
      if (i == 0) {
        projection.u = u;
        projection.v = v;
        projection.valid = true;
      }
      min_u = std::min(min_u, u);
      min_v = std::min(min_v, v);
      max_u = std::max(max_u, u);
      max_v = std::max(max_v, v);
      any_valid = true;
    }
    if (!projection.valid || !any_valid) {
      return Projection{};
    }
    projection.box = BBox2D{min_u, min_v, max_u, max_v};
    return projection;
  }

  std::array<double, 4> cameraIntrinsics() const {
    if (latest_camera_info_.p[0] != 0.0) {
      return {
        latest_camera_info_.p[0],
        latest_camera_info_.p[5],
        latest_camera_info_.p[2],
        latest_camera_info_.p[6]};
    }
    return {
      latest_camera_info_.k[0],
      latest_camera_info_.k[4],
      latest_camera_info_.k[2],
      latest_camera_info_.k[5]};
  }

  BBox2D cameraTargetBox() const {
    return BBox2D{
      latest_camera_target_.bbox[0],
      latest_camera_target_.bbox[1],
      latest_camera_target_.bbox[2],
      latest_camera_target_.bbox[3]};
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
      } else if (field.name == "intensity" &&
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

  Eigen::Matrix3d quaternionToRotation(
    const double x, const double y, const double z, const double w) const {
    Eigen::Quaterniond q(w, x, y, z);
    q.normalize();
    return q.toRotationMatrix();
  }

  Eigen::Matrix3d transformRotation(const geometry_msgs::msg::TransformStamped &transform) const {
    const auto &q = transform.transform.rotation;
    return quaternionToRotation(q.x, q.y, q.z, q.w);
  }

  Eigen::Vector3d transformTranslation(
    const geometry_msgs::msg::TransformStamped &transform) const {
    const auto &t = transform.transform.translation;
    return Eigen::Vector3d(t.x, t.y, t.z);
  }

  bool lookupTransform(
    const std::string &target_frame,
    const std::string &source_frame,
    geometry_msgs::msg::TransformStamped &transform) {
    if (target_frame == source_frame) {
      transform.header.frame_id = target_frame;
      transform.child_frame_id = source_frame;
      transform.transform.rotation.w = 1.0;
      return true;
    }
    try {
      transform = tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero, 20ms);
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  bool transformPointSet(
    const std::vector<PointXYZI> &input,
    const std::string &source_frame,
    const std::string &target_frame,
    std::vector<Eigen::Vector3d> &output) {
    geometry_msgs::msg::TransformStamped transform;
    if (!lookupTransform(target_frame, source_frame, transform)) {
      return false;
    }
    const auto rotation = transformRotation(transform);
    const auto translation = transformTranslation(transform);
    output.reserve(input.size());
    for (const auto &point : input) {
      output.emplace_back(rotation * Eigen::Vector3d(point.x, point.y, point.z) + translation);
    }
    return true;
  }

  bool transformVectorsToCamera(
    const std::vector<Eigen::Vector3d> &input,
    const std::string &source_frame,
    std::vector<Eigen::Vector3d> &output) {
    geometry_msgs::msg::TransformStamped transform;
    if (lookupTransform(camera_frame_, source_frame, transform)) {
      const auto rotation = transformRotation(transform);
      const auto translation = transformTranslation(transform);
      output.reserve(input.size());
      for (const auto &point : input) {
        output.emplace_back(rotation * point + translation);
      }
      return true;
    }
    if (source_frame == base_frame_) {
      geometry_msgs::msg::TransformStamped base_to_lidar;
      if (!lookupTransform(lidar_frame_, base_frame_, base_to_lidar)) {
        return false;
      }
      const auto rotation = transformRotation(base_to_lidar);
      const auto translation = transformTranslation(base_to_lidar);
      output.reserve(input.size());
      for (const auto &point : input) {
        const auto lidar_point = rotation * point + translation;
        output.emplace_back(direct_optical_rotation_ * lidar_point + direct_optical_translation_);
      }
      return true;
    }
    return false;
  }

  bool transformPointSetToCamera(
    const std::vector<PointXYZI> &input,
    const std::string &source_frame,
    std::vector<Eigen::Vector3d> &output) {
    geometry_msgs::msg::TransformStamped transform;
    if (lookupTransform(camera_frame_, source_frame, transform)) {
      const auto rotation = transformRotation(transform);
      const auto translation = transformTranslation(transform);
      output.reserve(input.size());
      for (const auto &point : input) {
        output.emplace_back(rotation * Eigen::Vector3d(point.x, point.y, point.z) + translation);
      }
      return true;
    }
    if (source_frame == lidar_frame_ &&
      direct_optical_child_frame_ == lidar_frame_ &&
      direct_optical_parent_frame_ == camera_frame_) {
      output.reserve(input.size());
      for (const auto &point : input) {
        output.emplace_back(
          direct_optical_rotation_ * Eigen::Vector3d(point.x, point.y, point.z) +
          direct_optical_translation_);
      }
      return true;
    }
    return false;
  }

  void publishOutputs(
    const rclcpp::Time &stamp,
    const std::string &state_name,
    const std::string &measurement_source,
    const std::string &rejection_reason,
    const bool measurement_accepted,
    const double association_score,
    const double nis_xy,
    const AnchorMeasurement &anchor,
    const std::string &trigger,
    const std::chrono::steady_clock::time_point &update_start,
    ProcessingTiming timing) {
    const auto publish_start = std::chrono::steady_clock::now();
    auto state = makeTargetState(stamp, state_name);
    fused_pub_->publish(state);
    compat_pub_->publish(state);
    publishSelectedTracklet(stamp, association_score);
    selected_target_marker_pub_->publish(makeSelectedTargetMarkers(state, state_name));
    selected_tracklet_marker_pub_->publish(makeSelectedTrackletMarkers(stamp));
    prediction_gate_marker_pub_->publish(makePredictionGateMarkers(stamp));
    fused_marker_pub_->publish(makeFusedMarkers(state, state_name));
    timing.publish_ms = elapsedMs(publish_start);

    const auto update_end = std::chrono::steady_clock::now();
    const double processing_ms =
      std::chrono::duration<double, std::milli>(update_end - update_start).count();
    if ((stamp - last_debug_time_).seconds() >= 1.0 / std::max(0.1, debug_rate_)) {
      last_debug_time_ = stamp;
      publishDebug(
        state_name, measurement_source, rejection_reason, measurement_accepted,
        association_score, nis_xy, anchor, trigger, processing_ms, timing);
    }
  }

  track_robot_interfaces::msg::TargetState makeTargetState(
    const rclcpp::Time &stamp,
    const std::string &state_name) const {
    track_robot_interfaces::msg::TargetState state;
    state.header.stamp = stamp;
    state.header.frame_id = base_frame_;
    state.target_id = latest_camera_target_.target_id;
    state.lock_state = latest_camera_target_.lock_state;
    state.bbox = latest_camera_target_.bbox;
    state.camera_visible = cameraVisible(stamp);
    state.confidence = latest_camera_target_.confidence;
    if (state_name == "camera_lidar") {
      state.track_state = track_robot_interfaces::msg::TargetState::TRACK_CAMERA_LIDAR_TRACKED;
      state.source_state = track_robot_interfaces::msg::TargetState::SOURCE_CAMERA_LIDAR;
      state.lidar_visible = true;
      state.confidence = std::max(state.confidence, static_cast<float>(kf_confidence_));
    } else if (state_name == "prediction_only") {
      state.track_state = track_robot_interfaces::msg::TargetState::TRACK_PREDICTION_ONLY;
      state.source_state = track_robot_interfaces::msg::TargetState::SOURCE_PREDICTION_ONLY;
      state.lidar_visible = false;
      state.confidence = static_cast<float>(kf_confidence_);
    } else if (state_name == "lidar_only") {
      state.track_state = track_robot_interfaces::msg::TargetState::TRACK_LIDAR_ONLY_TRACKING;
      state.source_state = track_robot_interfaces::msg::TargetState::SOURCE_LIDAR_ONLY;
      state.lidar_visible = true;
      state.confidence = static_cast<float>(kf_confidence_);
    } else if (state_name == "camera_only") {
      state.track_state = track_robot_interfaces::msg::TargetState::TRACK_CAMERA_LOCKED;
      state.source_state = track_robot_interfaces::msg::TargetState::SOURCE_CAMERA_ONLY;
      state.lidar_visible = false;
    } else if (state_name == "target_lost") {
      state.track_state = track_robot_interfaces::msg::TargetState::TRACK_TARGET_LOST;
      state.source_state = track_robot_interfaces::msg::TargetState::SOURCE_NONE;
      state.lidar_visible = false;
      state.confidence = 0.0F;
    } else {
      state.track_state = track_robot_interfaces::msg::TargetState::TRACK_NO_TARGET;
      state.source_state = track_robot_interfaces::msg::TargetState::SOURCE_NONE;
      state.lidar_visible = false;
    }
    if (kf_initialized_) {
      const Eigen::Vector3d position = kf_x_.head<3>();
      const Eigen::Vector3d velocity = kf_x_.tail<3>();
      state.position_base = toPoint(position);
      state.velocity = toVector(velocity);
      state.distance = static_cast<float>(normXY(position));
      state.bearing = static_cast<float>(std::atan2(position.y(), position.x()));
    }
    state.position_map_valid = false;
    state.time_since_camera_seen = static_cast<float>(timeSince(last_camera_seen_time_, stamp));
    state.time_since_lidar_seen = static_cast<float>(timeSince(last_lidar_seen_time_, stamp));
    return state;
  }

  void publishSelectedTracklet(const rclcpp::Time &stamp, const double score) {
    track_robot_interfaces::msg::SelectedLidarTracklet msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = base_frame_;
    msg.camera_target_id = latest_camera_target_.target_id;
    msg.selected_tracklet_id = selected_tracklet_id_;
    msg.selected = selected_tracklet_id_ >= 0;
    msg.confirmed = selected_tracklet_id_ >= 0;
    msg.association_score = static_cast<float>(score);
    msg.time_since_selected_seen = static_cast<float>(timeSince(last_lidar_seen_time_, stamp));
    auto selected = findTracklet(selected_tracklet_id_);
    if (selected.has_value()) {
      msg.tracklet = *selected;
    }
    selected_pub_->publish(msg);
  }

  visualization_msgs::msg::MarkerArray makeSelectedTargetMarkers(
    const track_robot_interfaces::msg::TargetState &state,
    const std::string &state_name) const {
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.emplace_back(clearMarker(state.header));
    if (!kf_initialized_ || state.track_state == track_robot_interfaces::msg::TargetState::TRACK_NO_TARGET) {
      return markers;
    }
    visualization_msgs::msg::Marker box;
    box.header = state.header;
    box.ns = "selected_target_state";
    box.id = 1;
    box.type = visualization_msgs::msg::Marker::CUBE;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.pose.position = toPoint(fromPoint(state.position_base) + camera_guided_marker_offset_);
    box.pose.orientation.w = 1.0;
    box.scale.x = camera_guided_marker_size_.x();
    box.scale.y = camera_guided_marker_size_.y();
    box.scale.z = camera_guided_marker_size_.z();
    box.color = colorForState(state_name, 0.32F);
    box.lifetime = rclcpp::Duration::from_seconds(0.25);
    markers.markers.emplace_back(box);

    visualization_msgs::msg::Marker text;
    text.header = state.header;
    text.ns = "selected_target_label";
    text.id = 2;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position = state.position_base;
    text.pose.position.z += 1.45;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.20;
    text.color.r = 1.0F;
    text.color.g = 1.0F;
    text.color.b = 1.0F;
    text.color.a = 1.0F;
    text.text = "target id=" + std::to_string(state.target_id) +
      " src=" + state_name +
      " lidar=" + std::to_string(selected_tracklet_id_);
    text.lifetime = rclcpp::Duration::from_seconds(0.25);
    markers.markers.emplace_back(text);
    return markers;
  }

  visualization_msgs::msg::MarkerArray makeSelectedTrackletMarkers(
    const rclcpp::Time &stamp) const {
    visualization_msgs::msg::MarkerArray markers;
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = base_frame_;
    markers.markers.emplace_back(clearMarker(header));
    auto selected = findTracklet(selected_tracklet_id_);
    if (!selected.has_value()) {
      return markers;
    }
    visualization_msgs::msg::Marker box;
    box.header = header;
    box.ns = "selected_lidar_tracklet";
    box.id = 1;
    box.type = visualization_msgs::msg::Marker::CUBE;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.pose.position.x = 0.5 * (selected->minimum.x + selected->maximum.x);
    box.pose.position.y = 0.5 * (selected->minimum.y + selected->maximum.y);
    box.pose.position.z = 0.5 * (selected->minimum.z + selected->maximum.z);
    box.pose.orientation.w = 1.0;
    box.scale = selected->size;
    box.color.r = 1.0F;
    box.color.g = 0.15F;
    box.color.b = 0.05F;
    box.color.a = 0.38F;
    box.lifetime = rclcpp::Duration::from_seconds(0.25);
    markers.markers.emplace_back(box);
    return markers;
  }

  visualization_msgs::msg::MarkerArray makePredictionGateMarkers(const rclcpp::Time &stamp) const {
    visualization_msgs::msg::MarkerArray markers;
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = base_frame_;
    markers.markers.emplace_back(clearMarker(header));
    if (!kf_initialized_) {
      return markers;
    }
    visualization_msgs::msg::Marker gate;
    gate.header = header;
    gate.ns = "target_prediction_gate";
    gate.id = 1;
    gate.type = visualization_msgs::msg::Marker::CYLINDER;
    gate.action = visualization_msgs::msg::Marker::ADD;
    gate.pose.position = toPoint(kf_x_.head<3>());
    gate.pose.position.z = 0.05;
    gate.pose.orientation.w = 1.0;
    const double radius = std::min(
      max_prediction_gate_radius_m_,
      std::max(
        prediction_gate_radius_m_,
        2.0 * std::sqrt(std::max(kf_p_(0, 0), kf_p_(1, 1)))));
    gate.scale.x = 2.0 * radius;
    gate.scale.y = 2.0 * radius;
    gate.scale.z = 0.05;
    gate.color.r = 1.0F;
    gate.color.g = 0.8F;
    gate.color.b = 0.05F;
    gate.color.a = 0.20F;
    gate.lifetime = rclcpp::Duration::from_seconds(0.25);
    markers.markers.emplace_back(gate);
    return markers;
  }

  visualization_msgs::msg::MarkerArray makeFusedMarkers(
    const track_robot_interfaces::msg::TargetState &state,
    const std::string &state_name) const {
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.emplace_back(clearMarker(state.header));
    if (!kf_initialized_) {
      return markers;
    }
    visualization_msgs::msg::Marker sphere;
    sphere.header = state.header;
    sphere.ns = "fused_target";
    sphere.id = 1;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position = state.position_base;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = 0.35;
    sphere.scale.y = 0.35;
    sphere.scale.z = 0.35;
    sphere.color = colorForState(state_name, 1.0F);
    sphere.lifetime = rclcpp::Duration::from_seconds(0.25);
    markers.markers.emplace_back(sphere);
    return markers;
  }

  visualization_msgs::msg::Marker clearMarker(const std_msgs::msg::Header &header) const {
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    return clear;
  }

  std_msgs::msg::ColorRGBA colorForState(
    const std::string &state_name,
    const float alpha) const {
    std_msgs::msg::ColorRGBA color;
    color.a = alpha;
    if (state_name == "camera_lidar") {
      color.r = 0.1F;
      color.g = 1.0F;
      color.b = 0.25F;
    } else if (state_name == "lidar_only") {
      color.r = 0.05F;
      color.g = 0.9F;
      color.b = 1.0F;
    } else if (state_name == "prediction_only") {
      color.r = 1.0F;
      color.g = 0.8F;
      color.b = 0.05F;
    } else if (state_name == "target_lost") {
      color.r = 1.0F;
      color.g = 0.1F;
      color.b = 0.1F;
    } else {
      color.r = 0.7F;
      color.g = 0.7F;
      color.b = 0.7F;
    }
    return color;
  }

  void publishCameraGuidedCloud(
    const std::vector<Eigen::Vector3d> &points,
    const rclcpp::Time &stamp) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = base_frame_;
    msg.height = 1;
    const size_t stride = points.size() > static_cast<size_t>(debug_cloud_max_points_) ?
      static_cast<size_t>(std::ceil(
        static_cast<double>(points.size()) / std::max(1, debug_cloud_max_points_))) : 1;
    msg.width = static_cast<uint32_t>((points.size() + stride - 1) / stride);
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.is_bigendian = false;
    msg.point_step = 12;
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = false;
    msg.data.resize(static_cast<size_t>(msg.row_step));
    size_t out = 0;
    for (size_t i = 0; i < points.size(); i += stride) {
      const float x = static_cast<float>(points[i].x());
      const float y = static_cast<float>(points[i].y());
      const float z = static_cast<float>(points[i].z());
      const size_t offset = out * msg.point_step;
      std::memcpy(msg.data.data() + offset, &x, sizeof(float));
      std::memcpy(msg.data.data() + offset + 4, &y, sizeof(float));
      std::memcpy(msg.data.data() + offset + 8, &z, sizeof(float));
      ++out;
    }
    camera_guided_points_pub_->publish(msg);
  }

  void publishDebug(
    const std::string &state_name,
    const std::string &measurement_source,
    const std::string &rejection_reason,
    const bool measurement_accepted,
    const double association_score,
    const double nis_xy,
    const AnchorMeasurement &anchor,
    const std::string &trigger,
    const double processing_ms,
    const ProcessingTiming &timing) {
    std::ostringstream ss;
    ss << "{"
       << "\"state\":\"" << state_name << "\","
       << "\"trigger\":\"" << trigger << "\","
       << "\"camera_target_id\":" << latest_camera_target_.target_id << ","
       << "\"selected_lidar_tracklet_id\":" << selected_tracklet_id_ << ","
       << "\"pending_lidar_tracklet_id\":" << pending_tracklet_id_ << ","
       << "\"pending_count\":" << pending_count_ << ","
       << "\"relink_lidar_tracklet_id\":" << relink_tracklet_id_ << ","
       << "\"relink_count\":" << relink_count_ << ","
       << "\"camera_visible\":" << (latest_camera_target_.camera_visible ? "true" : "false") << ","
       << "\"camera_confidence\":" << shortFloat(latest_camera_target_.confidence) << ","
       << "\"tracklet_count\":" << latest_tracklets_.tracklets.size() << ","
       << "\"candidate_count\":" << latest_candidates_.clusters.size() << ","
       << "\"camera_association_score\":" << shortFloat(association_score) << ","
       << "\"camera_guided_anchor_quality\":" << shortFloat(anchor.quality) << ","
       << "\"camera_guided_status\":\"" << anchor.status << "\","
       << "\"camera_guided_roi\":\"" << anchor.roi_type << "\","
       << "\"camera_guided_points\":" << anchor.point_count << ","
       << "\"measurement_source\":\"" << measurement_source << "\","
       << "\"kalman_nis_xy\":" << shortFloat(nis_xy) << ","
       << "\"kalman_initialized\":" << (kf_initialized_ ? "true" : "false") << ","
       << "\"measurement_accepted\":" << (measurement_accepted ? "true" : "false") << ","
       << "\"rejection_reason\":\"" << rejection_reason << "\","
       << "\"fresh_cloud_processed\":" <<
          (timing.fresh_cloud_processed ? "true" : "false") << ","
       << "\"cloud_parse_ms\":" << shortFloat(timing.cloud_parse_ms) << ","
       << "\"projection_ms\":" << shortFloat(timing.projection_ms) << ","
       << "\"association_ms\":" << shortFloat(timing.association_ms) << ","
       << "\"kalman_ms\":" << shortFloat(timing.kalman_ms) << ","
       << "\"publish_ms\":" << shortFloat(timing.publish_ms) << ","
       << "\"total_measurement_ms\":" << shortFloat(processing_ms) << ","
       << "\"processing_ms\":" << shortFloat(processing_ms)
       << "}";
    debug_pub_->publish(std_msgs::msg::String().set__data(ss.str()));
  }

  double timeSince(const rclcpp::Time &then, const rclcpp::Time &stamp) const {
    if (!then.nanoseconds()) {
      return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, (stamp - then).seconds());
  }

  double elapsedMs(const std::chrono::steady_clock::time_point &start) const {
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  }

  rclcpp::Time now() {
    return get_clock()->now();
  }

  void clearTarget() {
    selected_tracklet_id_ = -1;
    pending_tracklet_id_ = -1;
    pending_count_ = 0;
    relink_tracklet_id_ = -1;
    relink_count_ = 0;
    selected_missing_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    have_selected_reference_ = false;
    last_selected_point_count_ = 0;
    last_measurement_source_ = "none";
    camera_guided_marker_offset_ = Eigen::Vector3d(0.0, 0.0, 0.55);
    camera_guided_marker_size_ = Eigen::Vector3d(0.55, 0.55, 1.35);
    kf_initialized_ = false;
    kf_x_.setZero();
    kf_p_.setIdentity();
    kf_confidence_ = 0.0;
    last_camera_seen_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_lidar_seen_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  std::string camera_target_topic_;
  std::string detections_topic_;
  std::string lidar_tracklets_topic_;
  std::string lidar_topic_;
  std::string candidate_clusters_topic_;
  std::string camera_info_topic_;
  std::string selected_tracklet_topic_;
  std::string fused_target_topic_;
  std::string compat_target_topic_;
  std::string selected_tracklet_marker_topic_;
  std::string selected_target_marker_topic_;
  std::string prediction_gate_marker_topic_;
  std::string fused_marker_topic_;
  std::string camera_guided_points_topic_;
  std::string debug_topic_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string camera_frame_;
  std::string map_frame_;
  std::string direct_optical_parent_frame_;
  std::string direct_optical_child_frame_;
  Eigen::Vector3d direct_optical_translation_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d direct_optical_rotation_{Eigen::Matrix3d::Identity()};
  double direct_optical_qx_{0.5};
  double direct_optical_qy_{-0.5};
  double direct_optical_qz_{0.5};
  double direct_optical_qw_{0.5};

  double camera_visible_min_confidence_{0.35};
  double min_association_score_{0.55};
  int association_confirm_frames_{2};
  double max_projection_center_error_px_{220.0};
  double camera_target_timeout_sec_{1.0};
  double input_timeout_sec_{1.0};
  double publish_rate_{10.0};
  double debug_rate_{2.0};
  double min_range_{0.5};
  double max_range_{10.0};
  double min_z_{-0.25};
  double max_z_{2.2};
  int max_cloud_points_for_projection_{15000};
  int debug_cloud_max_points_{3000};
  int camera_guided_min_points_{3};
  double camera_guided_roi_center_width_fraction_{0.50};
  double camera_guided_roi_y_min_fraction_{0.20};
  double camera_guided_roi_y_max_fraction_{0.70};
  double camera_guided_depth_percentile_low_{20.0};
  double camera_guided_depth_percentile_high_{55.0};
  double min_keypoint_confidence_{0.35};
  double camera_guided_max_depth_spread_{1.25};
  double camera_guided_prediction_gate_m_{1.5};
  double process_noise_accel_std_{1.2};
  double initial_position_variance_{0.35};
  double initial_velocity_variance_{1.0};
  double camera_anchor_xy_variance_{0.08};
  double tracklet_xy_variance_{0.18};
  double z_variance_{0.35};
  double max_camera_anchor_nis_xy_{25.0};
  double max_tracklet_nis_xy_{9.21};
  double prediction_gate_radius_m_{1.2};
  double max_prediction_gate_radius_m_{2.0};
  double selected_exact_id_grace_sec_{0.5};
  int selected_relink_confirm_frames_{3};
  double selected_relink_timeout_sec_{1.5};
  double selected_relink_absolute_gate_m_{1.2};
  double selected_relink_min_score_{0.55};
  double prediction_velocity_damping_delay_sec_{0.4};
  double prediction_velocity_damping_rate_{0.8};
  double prediction_only_timeout_sec_{2.0};
  double max_target_speed_mps_{2.2};
  double camera_projection_max_rate_hz_{10.0};

  track_robot_interfaces::msg::TargetState latest_camera_target_;
  track_robot_interfaces::msg::HumanDetection2DArray latest_detections_;
  track_robot_interfaces::msg::LidarTrackletArray latest_tracklets_;
  track_robot_interfaces::msg::LidarClusterArray latest_candidates_;
  sensor_msgs::msg::CameraInfo latest_camera_info_;
  sensor_msgs::msg::PointCloud2 latest_cloud_;
  bool have_camera_info_{false};
  bool have_cloud_{false};
  rclcpp::Time last_camera_target_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_detections_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tracklets_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_candidates_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cloud_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_camera_seen_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_lidar_seen_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_filter_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_debug_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_camera_projection_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time selected_missing_since_{0, 0, RCL_ROS_TIME};

  int selected_tracklet_id_{-1};
  int pending_tracklet_id_{-1};
  int pending_count_{0};
  int relink_tracklet_id_{-1};
  int relink_count_{0};
  std::string last_measurement_source_{"none"};
  Eigen::Vector3d last_selected_size_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d camera_guided_marker_offset_{Eigen::Vector3d(0.0, 0.0, 0.55)};
  Eigen::Vector3d camera_guided_marker_size_{Eigen::Vector3d(0.55, 0.55, 1.35)};
  uint32_t last_selected_point_count_{0};
  bool have_selected_reference_{false};
  bool kf_initialized_{false};
  double kf_confidence_{0.0};
  double last_projected_nis_xy_{0.0};
  Eigen::Matrix<double, 6, 1> kf_x_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 6> kf_p_{Eigen::Matrix<double, 6, 6>::Identity()};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<track_robot_interfaces::msg::TargetState>::SharedPtr camera_target_sub_;
  rclcpp::Subscription<track_robot_interfaces::msg::HumanDetection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<track_robot_interfaces::msg::LidarTrackletArray>::SharedPtr tracklets_sub_;
  rclcpp::Subscription<track_robot_interfaces::msg::LidarClusterArray>::SharedPtr candidates_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<track_robot_interfaces::msg::SelectedLidarTracklet>::SharedPtr selected_pub_;
  rclcpp::Publisher<track_robot_interfaces::msg::TargetState>::SharedPtr fused_pub_;
  rclcpp::Publisher<track_robot_interfaces::msg::TargetState>::SharedPtr compat_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr selected_tracklet_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr selected_target_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr prediction_gate_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fused_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr camera_guided_points_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SelectedHumanTargetTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
