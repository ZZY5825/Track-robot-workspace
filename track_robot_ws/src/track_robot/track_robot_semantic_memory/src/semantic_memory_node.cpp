#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nlohmann/json.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "track_robot_interfaces/msg/association_debug.hpp"
#include "track_robot_interfaces/msg/semantic_lidar_tracklet_array.hpp"
#include "track_robot_interfaces/msg/semantic_localization_state.hpp"
#include "track_robot_interfaces/msg/semantic_memory_event.hpp"
#include "track_robot_interfaces/msg/semantic_object_array.hpp"
#include "track_robot_interfaces/msg/semantic_observation_array.hpp"
#include "track_robot_interfaces/msg/semantic_task.hpp"
#include "track_robot_interfaces/srv/get_semantic_object.hpp"
#include "track_robot_interfaces/srv/mark_semantic_object_inspected.hpp"
#include "track_robot_interfaces/srv/query_semantic_objects.hpp"
#include "track_robot_interfaces/srv/reset_semantic_memory.hpp"
#include "track_robot_semantic_memory/camera_lidar_projector.hpp"
#include "track_robot_semantic_memory/association_calibration.hpp"
#include "track_robot_semantic_memory/cross_modal_associator.hpp"
#include "track_robot_semantic_memory/memory_core.hpp"
#include "track_robot_semantic_memory/ros_conversions.hpp"
#include "track_robot_semantic_memory/reidentification_calibration.hpp"
#include "track_robot_semantic_memory/runtime_association.hpp"
#include "track_robot_semantic_memory/runtime_task_services.hpp"
#include "track_robot_semantic_memory/source_time_buffer.hpp"

namespace track_robot_semantic_memory
{
namespace
{

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

std::int64_t seconds_to_nanoseconds(double seconds, const char * name)
{
  if (!std::isfinite(seconds) || seconds < 0.0 ||
    seconds > static_cast<double>(std::numeric_limits<std::int64_t>::max()) * 1e-9)
  {
    throw std::invalid_argument(std::string(name) + " must be finite and non-negative");
  }
  return static_cast<std::int64_t>(std::llround(seconds * kNanosecondsPerSecond));
}

std::string bounded_reason(std::string reason)
{
  if (reason.size() > 256U) {
    reason.resize(256U);
  }
  return reason;
}

RigidTransform projector_transform(
  const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & q = transform.transform.rotation;
  const double norm = std::sqrt(
    q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm < 1e-12 ||
    !std::isfinite(transform.transform.translation.x) ||
    !std::isfinite(transform.transform.translation.y) ||
    !std::isfinite(transform.transform.translation.z))
  {
    throw std::invalid_argument("camera-LiDAR transform is invalid");
  }
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  return RigidTransform{
    {
      1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
      2.0 * (x * z + y * w),
      2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
      2.0 * (y * z - x * w),
      2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
      1.0 - 2.0 * (x * x + y * y)},
    {
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z}};
}

CameraModel camera_model_from_info(const sensor_msgs::msg::CameraInfo & info)
{
  const double fx = info.p[0] > 0.0 ? info.p[0] : info.k[0];
  const double fy = info.p[5] > 0.0 ? info.p[5] : info.k[4];
  const double cx = info.p[0] > 0.0 ? info.p[2] : info.k[2];
  const double cy = info.p[5] > 0.0 ? info.p[6] : info.k[5];
  CameraModel model{info.width, info.height, fx, fy, cx, cy, 0.05};
  const LidarBox3d validation_box{
    1, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}};
  const TransformEvidence identity{true, 1, RigidTransform{}};
  (void)CameraLidarProjector().project(model, validation_box, identity);
  return model;
}

std::string domain_identity(const MemoryDomainKey & domain)
{
  return std::to_string(static_cast<unsigned int>(domain.mode())) + ":" +
         std::to_string(domain.localization_epoch_id()) + ":" +
         domain.canonical_frame_id();
}

double bounded_unit(double value) noexcept
{
  return std::clamp(value, 0.0, 1.0);
}

}  // namespace

class SemanticMemoryNode final : public rclcpp::Node
{
public:
  SemanticMemoryNode()
  : Node("semantic_memory"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    enabled_ = declare_parameter<bool>("enabled", true);
    publish_active_ = declare_parameter<bool>("publish_active_objects", true);
    publish_events_ = declare_parameter<bool>("publish_events", true);
    publish_association_debug_ = declare_parameter<bool>(
      "publish_association_debug", true);
    association_shadow_mode_ = declare_parameter<bool>(
      "association_shadow_mode", true);
    camera_attachment_enabled_ = declare_parameter<bool>(
      "camera_attachment_enabled", false);
    appearance_memory_enabled_ = declare_parameter<bool>(
      "appearance_memory_enabled", true);
    reidentification_shadow_mode_ = declare_parameter<bool>(
      "reidentification_shadow_mode", true);
    reidentification_mutation_enabled_ = declare_parameter<bool>(
      "reidentification_mutation_enabled", false);
    const auto reidentification_calibration_status =
      declare_parameter<std::string>(
      "reidentification_calibration_status", "uncalibrated");
    const auto reidentification_calibration_report =
      declare_parameter<std::string>("reidentification_calibration_report", "");
    if (reidentification_mutation_enabled_ &&
      (reidentification_shadow_mode_ ||
      reidentification_calibration_status != "calibrated"))
    {
      throw std::invalid_argument(
              "reidentification mutation requires calibrated non-shadow configuration");
    }
    const auto calibration_status = declare_parameter<std::string>(
      "association_calibration_status", "not_calibrated");
    const auto calibration_report = declare_parameter<std::string>(
      "association_calibration_report", "");
    if (camera_attachment_enabled_ &&
      (association_shadow_mode_ || calibration_status != "calibrated"))
    {
      throw std::invalid_argument(
              "camera attachment requires calibrated non-shadow configuration");
    }
    const auto diagnostics_topic = declare_parameter<std::string>(
      "diagnostics_topic", "/semantic_memory/diagnostics");
    const auto active_topic = declare_parameter<std::string>(
      "active_objects_topic", "/semantic_memory/active_objects");
    const auto events_topic = declare_parameter<std::string>(
      "events_topic", "/semantic_memory/events");
    observations_topic_ = declare_parameter<std::string>(
      "observations_topic", "/semantic_memory/observations");
    association_debug_topic_ = declare_parameter<std::string>(
      "association_debug_topic", "/semantic_memory/association_debug");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/zed/zed_node/left/camera_info");
    tasks_topic_ = declare_parameter<std::string>(
      "tasks_topic", "/semantic_memory/tasks");
    const auto best_candidate_topic = declare_parameter<std::string>(
      "best_candidate_topic", "/semantic_memory/best_candidate");
    publish_best_candidate_ = declare_parameter<bool>(
      "publish_best_candidate", true);
    camera_calibration_id_ = declare_parameter<std::string>(
      "camera_calibration_id", "zed_left_rectified_v1");
    if (camera_calibration_id_.empty() || camera_calibration_id_.size() > 128U) {
      throw std::invalid_argument(
              "camera_calibration_id must contain 1 to 128 characters");
    }
    observation_queue_depth_ = bounded_depth(
      declare_parameter<std::int64_t>("observation_queue_depth", 1),
      "observation_queue_depth");
    task_queue_depth_ = bounded_depth(
      declare_parameter<std::int64_t>("task_queue_depth", 1),
      "task_queue_depth");
    task_relevance_config_.appearance_weight = declare_parameter<double>(
      "task_appearance_weight", 0.75);
    task_relevance_config_.semantic_weight = declare_parameter<double>(
      "task_semantic_weight", 0.25);
    task_relevance_config_.validate();
    best_candidate_config_.threshold_calibrated = declare_parameter<bool>(
      "best_candidate_threshold_calibrated", false);
    best_candidate_config_.minimum_relevance = declare_parameter<double>(
      "best_candidate_minimum_relevance", 1.0);
    if (!std::isfinite(best_candidate_config_.minimum_relevance) ||
      best_candidate_config_.minimum_relevance < 0.0 ||
      best_candidate_config_.minimum_relevance > 1.0)
    {
      throw std::invalid_argument(
              "best_candidate_minimum_relevance must be within [0, 1]");
    }
    max_association_debug_pairs_ = bounded_depth(
      declare_parameter<std::int64_t>(
        "max_association_debug_pairs_per_batch", 1024),
      "max_association_debug_pairs_per_batch", 1024U);

    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    if (publish_active_) {
      active_objects_publisher_ =
        create_publisher<track_robot_interfaces::msg::SemanticObjectArray>(
        active_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    }
    if (publish_events_) {
      events_publisher_ =
        create_publisher<track_robot_interfaces::msg::SemanticMemoryEvent>(
        events_topic, rclcpp::QoS(rclcpp::KeepLast(64)).reliable());
    }
    if (publish_association_debug_) {
      association_debug_publisher_ =
        create_publisher<track_robot_interfaces::msg::AssociationDebug>(
        association_debug_topic_, rclcpp::QoS(rclcpp::KeepLast(64)).reliable());
    }
    if (publish_best_candidate_) {
      best_candidate_publisher_ =
        create_publisher<track_robot_interfaces::msg::SemanticObjectArray>(
        best_candidate_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    }

    const auto association_config = read_association_config();
    association_max_source_time_delta_ns_ = seconds_to_nanoseconds(
      association_config.max_source_time_delta_s,
      "association_max_source_time_delta_sec");
    associator_ = std::make_unique<CrossModalAssociator>(association_config);
    const auto runtime_association_config = read_runtime_association_config();
    if (camera_attachment_enabled_) {
      validate_calibration_report(
        calibration_report, association_config, runtime_association_config,
        camera_calibration_id_);
    }
    runtime_association_ = std::make_unique<RuntimeAssociationCoordinator>(
      runtime_association_config);

    reidentification_config_ = read_reidentification_config();
    if (reidentification_mutation_enabled_) {
      validate_reidentification_report(
        reidentification_calibration_report, reidentification_config_);
    }
    reidentification_ =
      std::make_unique<RuntimeReidentificationCoordinator>(
      reidentification_config_);

    core_config_ = read_core_config();
    task_relevance_config_.normalization_tolerance =
      core_config_.appearance_normalization_tolerance;
    task_relevance_config_.maximum_semantic_evidence = 16U;
    task_relevance_config_.validate();
    const auto association_lidar_buffer_depth = bounded_depth(
      declare_parameter<std::int64_t>("association_lidar_buffer_depth", 128),
      "association_lidar_buffer_depth", 512U);
    const auto association_lidar_buffer_max_age_ns = seconds_to_nanoseconds(
      declare_parameter<double>("association_lidar_buffer_max_age_sec", 3.0),
      "association_lidar_buffer_max_age_sec");
    if (association_lidar_buffer_max_age_ns == 0) {
      throw std::invalid_argument(
              "association_lidar_buffer_max_age_sec must be positive");
    }
    association_lidar_batches_ = std::make_unique<SourceTimeBuffer<
        track_robot_interfaces::msg::SemanticLidarTrackletArray>>(
      SourceTimeBufferLimits{
        association_lidar_buffer_depth,
        association_lidar_buffer_max_age_ns,
        core_config_.rollback_tolerance_ns});
    const auto epoch_override = declare_parameter<std::int64_t>(
      "initial_memory_epoch_id", 0);
    if (epoch_override < 0) {
      throw std::invalid_argument("initial_memory_epoch_id must be non-negative");
    }
    initial_memory_epoch_override_ = static_cast<std::uint64_t>(epoch_override);
    const auto timeout_seconds = declare_parameter<double>("tf_lookup_timeout_sec", 0.03);
    if (!std::isfinite(timeout_seconds) || timeout_seconds <= 0.0 || timeout_seconds > 0.1) {
      throw std::invalid_argument("tf_lookup_timeout_sec must be in (0, 0.1]");
    }
    tf_lookup_timeout_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timeout_seconds));
    localization_state_timeout_ns_ = seconds_to_nanoseconds(
      declare_parameter<double>("localization_state_timeout_sec", 0.5),
      "localization_state_timeout_sec");
    if (localization_state_timeout_ns_ == 0) {
      throw std::invalid_argument("localization_state_timeout_sec must be positive");
    }

    if (enabled_) {
      create_input_subscriptions();
    }
    create_task_services();
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SemanticMemoryNode::publish_diagnostics, this));
  }

private:
  struct PendingRejection
  {
    std::int64_t tracklet_id{-1};
    std::string reason;
  };

  struct ScoredAssociationPair
  {
    const track_robot_interfaces::msg::LidarTracklet * tracklet{nullptr};
    PairAssociationScore score;
  };

  struct ScoredVisualObservation
  {
    const track_robot_interfaces::msg::SemanticObservation * observation{nullptr};
    const track_robot_interfaces::msg::SemanticLidarTrackletArray * lidar_batch{nullptr};
    std::int64_t visual_stamp_ns{0};
    std::vector<ScoredAssociationPair> pairs;
  };

  static std::size_t bounded_depth(
    std::int64_t depth, const char * name, std::size_t maximum = 64U)
  {
    if (depth < 1 || static_cast<std::uint64_t>(depth) > maximum) {
      throw std::invalid_argument(
              std::string(name) + " is outside its configured bound");
    }
    return static_cast<std::size_t>(depth);
  }

  AssociationConfig read_association_config()
  {
    AssociationConfig config;
    config.max_source_time_delta_s = declare_parameter<double>(
      "association_max_source_time_delta_sec", 0.10);
    config.max_evidence_age_s = declare_parameter<double>(
      "association_max_evidence_age_sec", 0.50);
    config.max_position_nis = declare_parameter<double>(
      "association_max_position_nis", 9.21);
    config.minimum_size_ratio = declare_parameter<double>(
      "association_minimum_size_ratio", 0.25);
    config.maximum_size_ratio = declare_parameter<double>(
      "association_maximum_size_ratio", 4.0);
    config.max_relative_speed_mps = declare_parameter<double>(
      "association_max_relative_speed_mps", 3.0);
    config.position_distance_max_m = declare_parameter<double>(
      "association_position_distance_max_m", 3.0);
    config.center_distance_max_px = declare_parameter<double>(
      "association_center_distance_max_px", 200.0);
    config.descriptor_normalization_tolerance = declare_parameter<double>(
      "association_descriptor_normalization_tolerance", 0.0001);
    config.require_position_nis = false;
    config.require_size_ratio = false;
    config.require_motion_gate = false;
    config.require_descriptors = false;
    config.weights.position_consistency = declare_parameter<double>(
      "association_weight_position_consistency", 1.0);
    config.weights.projected_centroid = declare_parameter<double>(
      "association_weight_projected_centroid", 1.0);
    config.weights.inside_fraction = declare_parameter<double>(
      "association_weight_inside_fraction", 1.0);
    config.weights.projected_iou = declare_parameter<double>(
      "association_weight_projected_iou", 1.0);
    config.weights.visual_cosine = declare_parameter<double>(
      "association_weight_visual_cosine", 1.0);
    config.weights.extent_consistency = declare_parameter<double>(
      "association_weight_extent_consistency", 1.0);
    config.weights.point_count_consistency = declare_parameter<double>(
      "association_weight_point_count_consistency", 1.0);
    config.weights.motion_continuity = declare_parameter<double>(
      "association_weight_motion_continuity", 1.0);
    config.weights.previous_association = declare_parameter<double>(
      "association_weight_previous_association", 1.0);
    config.weights.detector_confidence = declare_parameter<double>(
      "association_weight_detector_confidence", 1.0);
    config.weights.geometry_confidence = declare_parameter<double>(
      "association_weight_geometry_confidence", 1.0);
    config.weights.sensor_confidence = declare_parameter<double>(
      "association_weight_sensor_confidence", 1.0);
    return config;
  }

  RuntimeAssociationConfig read_runtime_association_config()
  {
    RuntimeAssociationConfig config;
    config.match_threshold = declare_parameter<double>(
      "association_match_threshold", 0.6303095604656801);
    config.confirmation.ambiguity_margin = declare_parameter<double>(
      "association_ambiguity_margin", 0.058443876599150624);
    config.confirmation.previous_association_hysteresis = declare_parameter<double>(
      "association_previous_hysteresis", 0.02);
    const auto confirmation_frames = declare_parameter<std::int64_t>(
      "association_confirmation_frames", 3);
    const auto detach_after_misses = declare_parameter<std::int64_t>(
      "association_detach_after_misses", 2);
    const auto cooldown_frames = declare_parameter<std::int64_t>(
      "association_cooldown_frames", 2);
    if (confirmation_frames < 1 || confirmation_frames > 16 ||
      detach_after_misses < 1 || detach_after_misses > 16 ||
      cooldown_frames < 1 || cooldown_frames > 64)
    {
      throw std::invalid_argument("runtime association confirmation bounds are invalid");
    }
    config.confirmation.confirmation_frames =
      static_cast<std::uint32_t>(confirmation_frames);
    config.confirmation.detach_after_misses =
      static_cast<std::uint32_t>(detach_after_misses);
    config.confirmation.cooldown_frames =
      static_cast<std::uint64_t>(cooldown_frames);
    config.maximum_pairs = max_association_debug_pairs_;
    config.validate();
    return config;
  }

  ReidentificationConfig read_reidentification_config()
  {
    ReidentificationConfig config;
    config.maximum_age_ns = seconds_to_nanoseconds(
      declare_parameter<double>("reidentification_maximum_age_sec", 5.0),
      "reidentification_maximum_age_sec");
    config.maximum_spatial_distance_m = declare_parameter<double>(
      "reidentification_maximum_spatial_distance_m", 3.0);
    config.minimum_appearance_similarity = declare_parameter<double>(
      "reidentification_minimum_appearance_similarity", 0.75);
    config.minimum_combined_score = declare_parameter<double>(
      "reidentification_minimum_combined_score", 0.70);
    config.ambiguity_margin = declare_parameter<double>(
      "reidentification_ambiguity_margin", 0.05);
    const auto confirmation_frames = declare_parameter<std::int64_t>(
      "reidentification_confirmation_frames", 3);
    if (confirmation_frames < 1 || confirmation_frames > 16) {
      throw std::invalid_argument(
              "reidentification confirmation frame count is invalid");
    }
    config.confirmation_frames = static_cast<std::uint32_t>(confirmation_frames);
    config.validate();
    return config;
  }

  static void validate_reidentification_report(
    const std::string & path,
    const ReidentificationConfig & config)
  {
    if (path.empty()) {
      throw std::invalid_argument(
              "reidentification mutation requires a calibration report");
    }
    const std::string resolved_path = path.front() == '/' ? path :
      ament_index_cpp::get_package_share_directory(
      "track_robot_semantic_memory") + "/" + path;
    std::ifstream input(resolved_path);
    if (!input) {
      throw std::invalid_argument(
              "reidentification_calibration_report cannot be opened");
    }
    nlohmann::json report;
    input >> report;
    validate_reidentification_calibration_report(
      report, {"stage2e_reidentification_v1", config});
  }

  static void validate_calibration_report(
    const std::string & path,
    const AssociationConfig & association_config,
    const RuntimeAssociationConfig & runtime_config,
    const std::string & camera_calibration_id)
  {
    if (path.empty()) {
      throw std::invalid_argument(
              "camera attachment requires association_calibration_report");
    }
    const std::string resolved_path = !path.empty() && path.front() == '/' ? path :
      ament_index_cpp::get_package_share_directory(
      "track_robot_semantic_memory") + "/" + path;
    std::ifstream input(resolved_path);
    if (!input) {
      throw std::invalid_argument(
              "association_calibration_report cannot be opened");
    }
    nlohmann::json report;
    input >> report;
    validate_association_calibration_report(report, {
        "stage2d_association_v1", camera_calibration_id,
        association_config, runtime_config});
  }

  MemoryCoreConfig read_core_config()
  {
    MemoryCoreConfig config;
    const auto max_objects = declare_parameter<std::int64_t>("max_objects", 256);
    const auto max_history = declare_parameter<std::int64_t>("max_history", 16);
    if (max_objects < 1 || max_objects > 256 || max_history < 0 || max_history > 16) {
      throw std::invalid_argument("memory object/history bounds violate Stage 2 contracts");
    }
    config.max_objects = static_cast<std::size_t>(max_objects);
    config.max_history = static_cast<std::size_t>(max_history);
    const auto maximum_prototypes = declare_parameter<std::int64_t>(
      "appearance_maximum_prototypes", 4);
    if (maximum_prototypes < 1 || maximum_prototypes > 4) {
      throw std::invalid_argument("appearance prototype bound is invalid");
    }
    config.max_feature_prototypes =
      static_cast<std::size_t>(maximum_prototypes);
    config.appearance_minimum_quality = declare_parameter<double>(
      "appearance_minimum_quality", 0.5);
    config.appearance_new_prototype_similarity_threshold =
      declare_parameter<double>(
      "appearance_new_prototype_similarity_threshold", 0.8);
    config.appearance_normalization_tolerance = declare_parameter<double>(
      "appearance_descriptor_normalization_tolerance", 0.0001);
    config.rollback_tolerance_ns = seconds_to_nanoseconds(
      declare_parameter<double>("rollback_tolerance_sec", 0.001),
      "rollback_tolerance_sec");
    config.static_lifecycle = read_lifecycle("static", {3U, 500000000, 2000000000, 10000000000});
    config.dynamic_lifecycle = read_lifecycle("dynamic", {3U, 300000000, 1000000000, 5000000000});
    config.static_process_noise = declare_parameter<double>("static_process_noise", 0.01);
    config.dynamic_process_noise = declare_parameter<double>("dynamic_process_noise", 0.10);
    config.static_max_speed_mps = declare_parameter<double>("static_max_speed_mps", 0.15);
    config.dynamic_min_speed_mps = declare_parameter<double>("dynamic_min_speed_mps", 0.35);
    config.validate();
    return config;
  }

  LifecyclePolicyConfig read_lifecycle(
    const std::string & prefix, LifecyclePolicyConfig defaults)
  {
    LifecyclePolicyConfig config;
    const auto hits = declare_parameter<std::int64_t>(
      prefix + "_confirmation_hits", defaults.confirmation_hits);
    if (hits < 1 || hits > std::numeric_limits<std::uint32_t>::max()) {
      throw std::invalid_argument(prefix + "_confirmation_hits is invalid");
    }
    config.confirmation_hits = static_cast<std::uint32_t>(hits);
    config.stale_after_ns = seconds_to_nanoseconds(
      declare_parameter<double>(
        prefix + "_stale_after_sec", defaults.stale_after_ns * 1e-9),
      (prefix + "_stale_after_sec").c_str());
    config.lost_after_ns = seconds_to_nanoseconds(
      declare_parameter<double>(
        prefix + "_lost_after_sec", defaults.lost_after_ns * 1e-9),
      (prefix + "_lost_after_sec").c_str());
    config.archive_after_ns = seconds_to_nanoseconds(
      declare_parameter<double>(
        prefix + "_archive_after_sec", defaults.archive_after_ns * 1e-9),
      (prefix + "_archive_after_sec").c_str());
    config.validate();
    return config;
  }

  static AppearanceDescriptor task_descriptor_from_message(
    const track_robot_interfaces::msg::VisualDescriptor & descriptor)
  {
    AppearanceDescriptor output;
    output.encoder_id = descriptor.encoder_id;
    output.checkpoint_id = descriptor.checkpoint_id;
    output.version = descriptor.version;
    output.dimension = descriptor.dimension;
    output.l2_normalized = descriptor.l2_normalized;
    output.values.assign(descriptor.values.begin(), descriptor.values.end());
    return output;
  }

  static std::string service_reason_string(ServiceReason reason)
  {
    switch (reason) {
      case ServiceReason::kOk: return "ok";
      case ServiceReason::kInvalidRequest: return "invalid_request";
      case ServiceReason::kStaleEpoch: return "stale_epoch";
      case ServiceReason::kNotFound: return "not_found";
      case ServiceReason::kEpochMismatch: return "epoch_mismatch";
      case ServiceReason::kThresholdNotCalibrated:
        return "threshold_not_calibrated";
      case ServiceReason::kNoEligibleCandidate:
        return "no_eligible_candidate";
      case ServiceReason::kBelowThreshold: return "below_threshold";
    }
    return "invalid_request";
  }

  std::map<GlobalObjectKey, std::vector<AppearancePrototype>>
  appearance_snapshot(
    const MemoryUpdateResult & snapshot,
    const MemoryCore & core) const
  {
    std::map<GlobalObjectKey, std::vector<AppearancePrototype>> output;
    for (const auto & object : snapshot.objects) {
      const auto * prototypes = core.appearance_prototypes(object.key);
      if (prototypes != nullptr && !prototypes->empty()) {
        output.emplace(object.key, *prototypes);
      }
    }
    return output;
  }

  RuntimeTaskServiceCoordinator synchronized_task_services(
    const MemoryUpdateResult & snapshot,
    const MemoryCore & core) const
  {
    auto next = task_services_ ?
      *task_services_ : RuntimeTaskServiceCoordinator(
      task_relevance_config_, best_candidate_config_, snapshot.memory_epoch_id);
    if (next.current_epoch() != snapshot.memory_epoch_id) {
      next.reset_to_epoch(snapshot.memory_epoch_id, "memory core epoch changed");
    }
    next.synchronize(snapshot, appearance_snapshot(snapshot, core));
    return next;
  }

  bool runtime_view_available() const noexcept
  {
    return current_domain_.has_value() && runtime_view_domain_.has_value() &&
           *current_domain_ == *runtime_view_domain_ && task_services_;
  }

  bool accept_task_message(
    const track_robot_interfaces::msg::SemanticTask & message,
    RuntimeTaskServiceCoordinator & services) const
  {
    return services.accept_task(
      {{message.query_id, message.query_version},
        task_descriptor_from_message(message.task_descriptor)},
      message.query_text, message.producer_epoch_id,
      rclcpp::Time(message.header.stamp).nanoseconds());
  }

  void publish_empty_runtime_snapshots(
    const std_msgs::msg::Header & source_header,
    const std::string & reason)
  {
    if (!current_domain_ || current_memory_epoch_id_ == 0U) {
      return;
    }
    auto header = source_header;
    header.frame_id = current_domain_->canonical_frame_id();
    if (publish_active_ && active_objects_publisher_) {
      track_robot_interfaces::msg::SemanticObjectArray output;
      output.header = header;
      output.memory_epoch_id = current_memory_epoch_id_;
      output.snapshot_sequence = ++snapshot_sequence_;
      active_objects_publisher_->publish(std::move(output));
    }
    if (publish_best_candidate_ && best_candidate_publisher_) {
      track_robot_interfaces::msg::SemanticObjectArray output;
      output.header = header;
      output.memory_epoch_id = current_memory_epoch_id_;
      output.snapshot_sequence = ++best_candidate_sequence_;
      best_candidate_publisher_->publish(std::move(output));
    }
    latest_active_object_count_ = 0U;
    last_best_candidate_reason_ = reason;
  }

  void publish_runtime_snapshots(const std_msgs::msg::Header & source_header)
  {
    if (!runtime_view_available()) {
      return;
    }
    const auto epoch = task_services_->current_epoch();
    auto header = source_header;
    header.frame_id = current_domain_->canonical_frame_id();
    const auto active = task_services_->active_objects();
    latest_active_object_count_ = active.size();
    if (publish_active_ && active_objects_publisher_) {
      track_robot_interfaces::msg::SemanticObjectArray output;
      output.header = header;
      output.memory_epoch_id = epoch;
      output.snapshot_sequence = ++snapshot_sequence_;
      for (const auto & view : active) {
        output.objects.push_back(
          semantic_object_from_runtime_view(view, *current_domain_));
      }
      active_objects_publisher_->publish(std::move(output));
    }
    if (publish_best_candidate_ && best_candidate_publisher_) {
      track_robot_interfaces::msg::SemanticObjectArray output;
      output.header = header;
      output.memory_epoch_id = epoch;
      output.snapshot_sequence = ++best_candidate_sequence_;
      const auto candidate = task_services_->best_candidate();
      if (candidate.object.has_value()) {
        output.objects.push_back(semantic_object_from_runtime_view(
            *candidate.object, *current_domain_));
      }
      best_candidate_publisher_->publish(std::move(output));
      last_best_candidate_reason_ = service_reason_string(candidate.reason);
    }
  }

  void create_task_services()
  {
    using GetObject = track_robot_interfaces::srv::GetSemanticObject;
    using QueryObjects = track_robot_interfaces::srv::QuerySemanticObjects;
    using MarkInspected = track_robot_interfaces::srv::MarkSemanticObjectInspected;
    using ResetMemory = track_robot_interfaces::srv::ResetSemanticMemory;
    get_object_service_ = create_service<GetObject>(
      "/semantic_memory/get_object",
      std::bind(
        &SemanticMemoryNode::on_get_object, this,
        std::placeholders::_1, std::placeholders::_2));
    query_objects_service_ = create_service<QueryObjects>(
      "/semantic_memory/query_objects",
      std::bind(
        &SemanticMemoryNode::on_query_objects, this,
        std::placeholders::_1, std::placeholders::_2));
    mark_inspected_service_ = create_service<MarkInspected>(
      "/semantic_memory/mark_inspected",
      std::bind(
        &SemanticMemoryNode::on_mark_inspected, this,
        std::placeholders::_1, std::placeholders::_2));
    reset_memory_service_ = create_service<ResetMemory>(
      "/semantic_memory/reset",
      std::bind(
        &SemanticMemoryNode::on_reset_memory, this,
        std::placeholders::_1, std::placeholders::_2));
  }

  void on_task(track_robot_interfaces::msg::SemanticTask::ConstSharedPtr message)
  {
    ++task_message_count_;
    if (!runtime_view_available()) {
      const auto source_stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
      if (source_stamp_ns < 0 ||
        (pending_task_producer_epoch_id_ == message->producer_epoch_id &&
        pending_task_source_stamp_ns_.has_value() &&
        source_stamp_ns < *pending_task_source_stamp_ns_))
      {
        pending_task_.reset();
        pending_task_clear_requested_ = true;
        pending_task_producer_epoch_id_ = message->producer_epoch_id;
        pending_task_source_stamp_ns_ = source_stamp_ns;
        ++rejected_task_count_;
        last_task_reason_ = "pending_task_source_time_rollback";
        return;
      }
      pending_task_ = *message;
      pending_task_clear_requested_ = false;
      pending_task_producer_epoch_id_ = message->producer_epoch_id;
      pending_task_source_stamp_ns_ = source_stamp_ns;
      last_task_reason_ = "pending_memory_snapshot";
      return;
    }
    try {
      auto next = *task_services_;
      const auto accepted = accept_task_message(*message, next);
      *task_services_ = std::move(next);
      if (!accepted) {
        ++rejected_task_count_;
        last_task_reason_ = "invalid_or_rolled_back_task";
      } else {
        last_task_reason_ = "accepted";
      }
      publish_runtime_snapshots(message->header);
    } catch (const std::exception & error) {
      ++rejected_task_count_;
      last_task_reason_ = bounded_reason(error.what());
    }
  }

  void on_get_object(
    const std::shared_ptr<track_robot_interfaces::srv::GetSemanticObject::Request> request,
    std::shared_ptr<track_robot_interfaces::srv::GetSemanticObject::Response> response)
  {
    ++service_call_count_;
    if (!runtime_view_available()) {
      response->reason = "not_found";
      return;
    }
    const auto result = task_services_->get(
      {request->memory_epoch_id, request->global_object_id});
    response->reason = service_reason_string(result.reason);
    response->found = result.object.has_value();
    if (result.object.has_value()) {
      response->object = semantic_object_from_runtime_view(
        *result.object, *current_domain_);
    }
  }

  void on_query_objects(
    const std::shared_ptr<track_robot_interfaces::srv::QuerySemanticObjects::Request> request,
    std::shared_ptr<track_robot_interfaces::srv::QuerySemanticObjects::Response> response)
  {
    ++service_call_count_;
    if (!runtime_view_available()) {
      response->reason = "not_found";
      return;
    }
    const QueryMemoryRequest query{
      request->page_size, request->page_token,
      request->include_stale, request->include_lost,
      request->include_archived, request->include_inspected};
    RuntimeObjectQueryResult result;
    if (request->query_mode == request->QUERY_ACTIVE_TASK) {
      result = task_services_->query_active(
        {request->query_id, request->query_version}, query);
    } else if (request->query_mode == request->QUERY_DESCRIPTOR) {
      result = task_services_->query_descriptor(
        {{request->query_id, request->query_version},
          task_descriptor_from_message(request->task_descriptor)},
        request->query_text, query);
    } else {
      response->reason = "invalid_request";
      return;
    }
    response->accepted = result.accepted;
    response->reason = service_reason_string(result.reason);
    response->next_page_token = result.next_page_token;
    response->has_more = result.has_more;
    for (const auto & view : result.objects) {
      response->objects.push_back(
        semantic_object_from_runtime_view(view, *current_domain_));
    }
  }

  void on_mark_inspected(
    const std::shared_ptr<
      track_robot_interfaces::srv::MarkSemanticObjectInspected::Request> request,
    std::shared_ptr<
      track_robot_interfaces::srv::MarkSemanticObjectInspected::Response> response)
  {
    ++service_call_count_;
    if (!runtime_view_available()) {
      response->reason = "not_found";
      return;
    }
    const GlobalObjectKey key{
      request->memory_epoch_id, request->global_object_id};
    const auto before = task_services_->get(key);
    auto next = *task_services_;
    const auto result = next.mark_inspected(
      key, static_cast<InspectionState>(request->inspection_state));
    response->updated = result.updated;
    response->reason = service_reason_string(result.reason);
    if (!result.updated) {
      return;
    }
    *task_services_ = std::move(next);
    const auto current = task_services_->get(key);
    if (current.object.has_value()) {
      response->object = semantic_object_from_runtime_view(
        *current.object, *current_domain_);
    }
    const bool changed = before.object.has_value() &&
      current.object.has_value() &&
      before.object->inspection != current.object->inspection;
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = current_domain_->canonical_frame_id();
    if (changed && publish_events_ && events_publisher_) {
      events_publisher_->publish(semantic_event_from_memory(
          {MemoryEventType::kInspectionChanged, key}, header,
          ++event_sequence_, task_services_->current_epoch()));
    }
    publish_runtime_snapshots(header);
  }

  void on_reset_memory(
    const std::shared_ptr<track_robot_interfaces::srv::ResetSemanticMemory::Request> request,
    std::shared_ptr<track_robot_interfaces::srv::ResetSemanticMemory::Response> response)
  {
    ++service_call_count_;
    if (!runtime_view_available() || !memory_core_) {
      response->result_reason = "not_found";
      return;
    }
    if (request->require_epoch_match &&
      request->expected_memory_epoch_id != current_memory_epoch_id_)
    {
      response->new_memory_epoch_id = current_memory_epoch_id_;
      response->result_reason = "epoch_mismatch";
      return;
    }
    try {
      auto next_core = *memory_core_;
      auto next_services = *task_services_;
      auto result = next_core.reset(*current_domain_);
      next_services.reset_to_epoch(result.memory_epoch_id, request->reason);
      *memory_core_ = std::move(next_core);
      *task_services_ = std::move(next_services);
      current_memory_epoch_id_ = result.memory_epoch_id;
      latest_active_object_count_ = 0U;
      reset_runtime_association("memory_service_reset");
      if (association_lidar_batches_) {
        association_lidar_batches_->clear();
      }
      latest_lidar_batch_.reset();
      response->reset = true;
      response->new_memory_epoch_id = result.memory_epoch_id;
      response->result_reason = "ok";
      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = current_domain_->canonical_frame_id();
      if (publish_events_ && events_publisher_) {
        for (const auto & event : result.events) {
          events_publisher_->publish(semantic_event_from_memory(
              event, header, ++event_sequence_, result.memory_epoch_id));
        }
      }
      publish_runtime_snapshots(header);
    } catch (const std::exception & error) {
      response->result_reason = bounded_reason(error.what());
    }
  }

  void create_input_subscriptions()
  {
    const auto localization_topic = declare_parameter<std::string>(
      "localization_topic", "/semantic_memory/localization_state");
    const auto lidar_topic = declare_parameter<std::string>(
      "lidar_tracklets_topic", "/semantic_memory/lidar_tracklets");
    const auto localization_depth = bounded_depth(
      declare_parameter<std::int64_t>("localization_queue_depth", 1),
      "localization_queue_depth");
    const auto lidar_depth = bounded_depth(
      declare_parameter<std::int64_t>("lidar_queue_depth", 1),
      "lidar_queue_depth");

    localization_subscription_ = create_subscription<
      track_robot_interfaces::msg::SemanticLocalizationState>(
      localization_topic,
      rclcpp::QoS(rclcpp::KeepLast(localization_depth)).reliable(),
      std::bind(&SemanticMemoryNode::on_localization, this, std::placeholders::_1));
    lidar_subscription_ = create_subscription<
      track_robot_interfaces::msg::SemanticLidarTrackletArray>(
      lidar_topic,
      rclcpp::QoS(rclcpp::KeepLast(lidar_depth)).best_effort(),
      std::bind(&SemanticMemoryNode::on_lidar, this, std::placeholders::_1));
    observation_subscription_ = create_subscription<
      track_robot_interfaces::msg::SemanticObservationArray>(
      observations_topic_,
      rclcpp::QoS(rclcpp::KeepLast(observation_queue_depth_)).reliable(),
      std::bind(
        &SemanticMemoryNode::on_observations, this, std::placeholders::_1));
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&SemanticMemoryNode::on_camera_info, this, std::placeholders::_1));
    task_subscription_ = create_subscription<track_robot_interfaces::msg::SemanticTask>(
      tasks_topic_,
      rclcpp::QoS(rclcpp::KeepLast(task_queue_depth_)).reliable(),
      std::bind(&SemanticMemoryNode::on_task, this, std::placeholders::_1));
  }

  void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr message)
  {
    ++camera_info_message_count_;
    try {
      if (message->header.frame_id.empty()) {
        throw std::invalid_argument("camera info frame is empty");
      }
      camera_model_ = camera_model_from_info(*message);
      camera_frame_id_ = message->header.frame_id;
      last_camera_info_reason_ = "valid";
    } catch (const std::exception & error) {
      camera_model_.reset();
      camera_frame_id_.clear();
      ++rejected_camera_info_count_;
      last_camera_info_reason_ = bounded_reason(error.what());
    }
  }

  void reset_runtime_association(const std::string & reason)
  {
    if (runtime_association_) {
      runtime_association_->reset();
    }
    if (reidentification_) {
      reidentification_->reset();
    }
    reidentification_source_frame_index_ = 0U;
    last_visual_producer_epoch_id_ = 0U;
    last_visual_source_stamp_ns_.reset();
    ++runtime_association_reset_count_;
    last_association_reason_ = reason;
  }

  void on_localization(
    track_robot_interfaces::msg::SemanticLocalizationState::ConstSharedPtr message)
  {
    ++localization_message_count_;
    try {
      const auto next_domain = domain_from_localization_state(*message);
      const bool domain_changed =
        current_domain_.has_value() && *current_domain_ != next_domain;
      if (domain_changed) {
        if (association_lidar_batches_) {
          association_lidar_batches_->clear();
        }
        runtime_view_domain_.reset();
        reset_runtime_association("localization_domain_changed");
      }
      current_domain_ = next_domain;
      const auto stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
      if (stamp_ns < 0) {
        throw std::invalid_argument("localization state stamp must be non-negative");
      }
      current_localization_stamp_ns_ = stamp_ns;
      last_localization_reason_ = message->reason;
      if (domain_changed) {
        publish_empty_runtime_snapshots(
          message->header, "localization_domain_changed");
      }
    } catch (const std::exception & error) {
      if (current_domain_.has_value() && runtime_view_domain_.has_value()) {
        publish_empty_runtime_snapshots(
          message->header, "invalid_localization_state");
      }
      current_domain_.reset();
      runtime_view_domain_.reset();
      current_localization_stamp_ns_.reset();
      ++rejected_localization_count_;
      last_localization_reason_ = bounded_reason(error.what());
    }
  }

  geometry_msgs::msg::TransformStamped transform_at_measurement(
    const std::string & target_frame,
    const std::string & source_frame,
    const builtin_interfaces::msg::Time & last_measurement_stamp)
  {
    if (target_frame == source_frame) {
      geometry_msgs::msg::TransformStamped identity;
      identity.header.frame_id = target_frame;
      identity.header.stamp = last_measurement_stamp;
      identity.child_frame_id = source_frame;
      identity.transform.rotation.w = 1.0;
      return identity;
    }
    return tf_buffer_.lookupTransform(
      target_frame, source_frame, rclcpp::Time(last_measurement_stamp),
      tf_lookup_timeout_);
  }

  void on_lidar(
    track_robot_interfaces::msg::SemanticLidarTrackletArray::ConstSharedPtr message)
  {
    ++lidar_message_count_;
    producer_dropped_tracklet_count_ += message->dropped_tracklet_count;
    if (!current_domain_) {
      ++rejected_lidar_batch_count_;
      last_lidar_reason_ = "waiting_for_valid_localization_domain";
      return;
    }
    if (message->source_epoch_id == 0U || message->header.frame_id.empty()) {
      ++rejected_lidar_batch_count_;
      last_lidar_reason_ = "invalid_lidar_batch_identity_or_frame";
      return;
    }
    const std::int64_t batch_stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (batch_stamp_ns < 0) {
      ++rejected_lidar_batch_count_;
      last_lidar_reason_ = "negative_lidar_batch_stamp";
      return;
    }
    if (!current_localization_stamp_ns_ || !source_times_within_tolerance(
        *current_localization_stamp_ns_, batch_stamp_ns,
        localization_state_timeout_ns_))
    {
      ++rejected_lidar_batch_count_;
      last_lidar_reason_ = "localization_state_not_fresh_for_lidar_batch";
      return;
    }
    if (last_lidar_source_epoch_id_ != 0U &&
      last_lidar_source_epoch_id_ != message->source_epoch_id)
    {
      association_lidar_batches_->clear();
      reset_runtime_association("lidar_source_epoch_changed");
    }
    last_lidar_source_epoch_id_ = message->source_epoch_id;
    latest_lidar_batch_ = *message;
    if (association_lidar_batches_) {
      association_lidar_batches_->push(
        SourceTimeKey{
          batch_stamp_ns,
          message->source_epoch_id,
          0},
        *message);
    }
    if (!memory_core_) {
      const auto epoch = initial_memory_epoch_override_ != 0U ?
        initial_memory_epoch_override_ :
        derive_memory_epoch_seed(*current_domain_, message->source_epoch_id);
      memory_core_ = std::make_unique<MemoryCore>(core_config_, epoch);
      current_memory_epoch_id_ = epoch;
    }

    std::vector<const track_robot_interfaces::msg::LidarTracklet *> active_tracklets;
    active_tracklets.reserve(message->tracklets.size());
    for (const auto & tracklet : message->tracklets) {
      if (tracklet.active) {
        active_tracklets.push_back(&tracklet);
      }
    }
    std::sort(active_tracklets.begin(), active_tracklets.end(),
      [](const auto * left, const auto * right) {
        if (left->tracklet_id != right->tracklet_id) {
          return left->tracklet_id < right->tracklet_id;
        }
        return rclcpp::Time(left->last_measurement_stamp).nanoseconds() <
               rclcpp::Time(right->last_measurement_stamp).nanoseconds();
      });

    std::map<std::int32_t, std::size_t> id_counts;
    for (const auto * tracklet : active_tracklets) {
      ++id_counts[tracklet->tracklet_id];
    }
    std::map<std::int64_t, geometry_msgs::msg::TransformStamped> transforms;
    std::vector<LidarObservation> observations;
    std::vector<PendingRejection> pending_rejections;
    observations.reserve(active_tracklets.size());
    for (const auto * tracklet : active_tracklets) {
      if (id_counts[tracklet->tracklet_id] > 1U) {
        pending_rejections.push_back(
          {tracklet->tracklet_id, "duplicate_lidar_source_key"});
        continue;
      }
      try {
        const auto measurement_ns = rclcpp::Time(
          tracklet->last_measurement_stamp).nanoseconds();
        if (measurement_ns < 0 || measurement_ns > batch_stamp_ns) {
          throw std::invalid_argument("tracklet stamp is outside its batch");
        }
        auto found = transforms.find(measurement_ns);
        if (found == transforms.end()) {
          found = transforms.emplace(
            measurement_ns,
            transform_at_measurement(
              current_domain_->canonical_frame_id(), message->header.frame_id,
              tracklet->last_measurement_stamp)).first;
        }
        observations.push_back(lidar_observation_from_tracklet(
            *tracklet, message->source_epoch_id, found->second));
      } catch (const std::exception & error) {
        pending_rejections.push_back(
          {tracklet->tracklet_id,
            bounded_reason(std::string("tracklet_rejected:") + error.what())});
      }
    }

    try {
      const auto previous_memory_epoch = current_memory_epoch_id_;
      auto next_memory_core = *memory_core_;
      auto result = next_memory_core.update(
        *current_domain_, batch_stamp_ns, std::move(observations));
      auto next_task_services = synchronized_task_services(
        result, next_memory_core);
      const bool had_pending_task = pending_task_.has_value();
      const bool had_pending_clear = pending_task_clear_requested_;
      bool pending_task_accepted = true;
      if (had_pending_clear) {
        next_task_services.clear_task();
      } else if (had_pending_task) {
        pending_task_accepted = accept_task_message(
          *pending_task_, next_task_services);
      }
      *memory_core_ = std::move(next_memory_core);
      if (!task_services_) {
        task_services_ = std::make_unique<RuntimeTaskServiceCoordinator>(
          std::move(next_task_services));
      } else {
        *task_services_ = std::move(next_task_services);
      }
      runtime_view_domain_ = *current_domain_;
      if (had_pending_task || had_pending_clear) {
        pending_task_.reset();
        pending_task_clear_requested_ = false;
        pending_task_producer_epoch_id_ = 0U;
        pending_task_source_stamp_ns_.reset();
        if (had_pending_clear) {
          last_task_reason_ = "rolled_back_pending_task_cleared";
        } else if (pending_task_accepted) {
          last_task_reason_ = "accepted_after_memory_snapshot";
        } else {
          ++rejected_task_count_;
          last_task_reason_ = "invalid_or_rolled_back_task";
        }
      }
      current_memory_epoch_id_ = result.memory_epoch_id;
      if (previous_memory_epoch != 0U &&
        previous_memory_epoch != current_memory_epoch_id_)
      {
        reset_runtime_association("memory_epoch_changed");
      }
      rejected_observation_count_ += result.rejected_observations;
      rejected_observation_count_ += pending_rejections.size();
      std_msgs::msg::Header output_header = message->header;
      output_header.frame_id = current_domain_->canonical_frame_id();
      publish_runtime_snapshots(output_header);
      if (publish_events_) {
        for (const auto & event : result.events) {
          events_publisher_->publish(semantic_event_from_memory(
              event, output_header, ++event_sequence_, result.memory_epoch_id));
        }
        for (const auto & rejection : pending_rejections) {
          publish_rejection_event(
            output_header, message->source_epoch_id, rejection);
        }
      }
      last_lidar_reason_ = "updated";
    } catch (const std::exception & error) {
      ++rejected_lidar_batch_count_;
      last_lidar_reason_ = bounded_reason(std::string("memory_update_rejected:") + error.what());
    }
  }

  std::optional<VisualAssociationKey> stable_visual_key(
    const track_robot_interfaces::msg::SemanticObservation & observation) const
  {
    if (observation.camera_track_id_valid && observation.camera_track_id >= 0) {
      return VisualAssociationKey{
        VisualAssociationKind::kCameraTrack,
        observation.producer_epoch_id,
        static_cast<std::uint64_t>(observation.camera_track_id)};
    }
    if (observation.upstream_proposal_id_valid &&
      observation.upstream_producer_epoch_id != 0U)
    {
      return VisualAssociationKey{
        VisualAssociationKind::kUpstreamProposal,
        observation.upstream_producer_epoch_id,
        observation.upstream_proposal_id};
    }
    return std::nullopt;
  }

  static VisualMemorySupplement make_visual_supplement(
    const track_robot_interfaces::msg::SemanticObservation & observation,
    const VisualAssociationKey & visual_key,
    const LidarAssociationKey & lidar_key,
    double association_confidence,
    bool appearance_memory_enabled)
  {
    return visual_supplement_from_semantic_observation(
      observation, visual_key, lidar_key, association_confidence,
      appearance_memory_enabled);
  }

  std::vector<ScoredAssociationPair> score_visual_pairs(
    const track_robot_interfaces::msg::SemanticObservation & observation,
    const track_robot_interfaces::msg::SemanticLidarTrackletArray & lidar_batch,
    const std::vector<const track_robot_interfaces::msg::LidarTracklet *> & tracklets,
    const std::int64_t evaluation_stamp_ns)
  {
    const auto visual_stamp_ns = rclcpp::Time(observation.camera_stamp).nanoseconds();
    std::vector<ScoredAssociationPair> pairs;
    pairs.reserve(tracklets.size());
    for (const auto * tracklet : tracklets) {
      PairEvidence evidence;
      evidence.visual_stamp_ns = visual_stamp_ns;
      evidence.lidar_stamp_ns = rclcpp::Time(
        tracklet->last_measurement_stamp).nanoseconds();
      evidence.evaluation_stamp_ns = std::max({
          evaluation_stamp_ns, evidence.visual_stamp_ns, evidence.lidar_stamp_ns});
      evidence.visual_domain = domain_identity(*current_domain_);
      evidence.lidar_domain = evidence.visual_domain;
      evidence.calibration_available = camera_model_.has_value() &&
        !camera_frame_id_.empty() && observation.header.frame_id == camera_frame_id_ &&
        observation.calibration_id == camera_calibration_id_ &&
        observation.image_width == camera_model_->width &&
        observation.image_height == camera_model_->height;

      if (evidence.calibration_available && evidence.lidar_stamp_ns >= 0) {
        try {
          const auto transform = transform_at_measurement(
            camera_frame_id_, lidar_batch.header.frame_id,
            tracklet->last_measurement_stamp);
          evidence.transform_available = true;
          const TransformEvidence transform_evidence{
            true, evidence.lidar_stamp_ns, projector_transform(transform)};
          const Point3d half_extent{
            0.5 * std::max(0.0, tracklet->size.x),
            0.5 * std::max(0.0, tracklet->size.y),
            0.5 * std::max(0.0, tracklet->size.z)};
          const LidarBox3d lidar_box{
            evidence.lidar_stamp_ns,
            {tracklet->position.x, tracklet->position.y, tracklet->position.z},
            {
              tracklet->position.x - half_extent.x,
              tracklet->position.y - half_extent.y,
              tracklet->position.z - half_extent.z},
            {
              tracklet->position.x + half_extent.x,
              tracklet->position.y + half_extent.y,
              tracklet->position.z + half_extent.z}};
          const auto projection = projector_.project(
            *camera_model_, lidar_box, transform_evidence);
          evidence.field_of_view_compatible =
            projection.status == ProjectionStatus::kVisible;
          if (evidence.field_of_view_compatible) {
            const Box2d roi{
              static_cast<double>(observation.roi.x_offset),
              static_cast<double>(observation.roi.y_offset),
              static_cast<double>(observation.roi.x_offset + observation.roi.width),
              static_cast<double>(observation.roi.y_offset + observation.roi.height)};
            evidence.projected_centroid_distance_px =
              center_distance_pixels(projection.projected_box, roi);
            evidence.projected_inside_fraction =
              inside_fraction(projection.projected_box, roi);
            evidence.projected_iou = intersection_over_union(
              projection.projected_box, roi);
            const double projected_area =
              (projection.projected_box.right - projection.projected_box.left) *
              (projection.projected_box.bottom - projection.projected_box.top);
            const double roi_area = static_cast<double>(observation.roi.width) *
              static_cast<double>(observation.roi.height);
            if (projected_area > 0.0 && roi_area > 0.0) {
              const double ratio = roi_area / projected_area;
              evidence.size_ratio = ratio;
              evidence.extent_consistency = std::min(ratio, 1.0 / ratio);
            }
          }
        } catch (const std::exception &) {
          evidence.transform_available = false;
          evidence.field_of_view_compatible = false;
        }
      }
      evidence.previous_association =
        observation.lidar_tracklet_id_valid &&
        observation.lidar_source_epoch_id == lidar_batch.source_epoch_id &&
        observation.lidar_tracklet_id == tracklet->tracklet_id ? 1.0 : 0.0;
      evidence.detector_confidence = bounded_unit(observation.detector_confidence);
      evidence.geometry_confidence = bounded_unit(observation.geometry_confidence);
      evidence.sensor_confidence = bounded_unit(std::min(
          static_cast<double>(tracklet->confidence),
          static_cast<double>(tracklet->observation_quality)));
      pairs.push_back({tracklet, associator_->score(evidence)});
    }
    return pairs;
  }

  void on_observations(
    track_robot_interfaces::msg::SemanticObservationArray::ConstSharedPtr message)
  {
    ++observation_message_count_;
    const auto evaluation_stamp_ns = latest_lidar_batch_.has_value() ?
      rclcpp::Time(latest_lidar_batch_->header.stamp).nanoseconds() :
      now().nanoseconds();
    if (!association_shadow_mode_ && !camera_attachment_enabled_) {
      last_association_reason_ = "association_outputs_disabled";
      return;
    }
    if (association_shadow_mode_ && camera_attachment_enabled_) {
      ++rejected_observation_batch_count_;
      last_association_reason_ = "invalid_shadow_attachment_configuration";
      return;
    }
    if (!current_domain_ || !memory_core_ || !association_lidar_batches_ ||
      association_lidar_batches_->entries().empty())
    {
      ++rejected_observation_batch_count_;
      last_association_reason_ = "waiting_for_domain_and_lidar_evidence";
      return;
    }
    if (last_visual_producer_epoch_id_ != 0U &&
      last_visual_producer_epoch_id_ != message->producer_epoch_id)
    {
      reset_runtime_association("visual_producer_epoch_changed");
    }
    last_visual_producer_epoch_id_ = message->producer_epoch_id;
    if (message->producer_epoch_id == 0U) {
      ++rejected_observation_batch_count_;
      last_association_reason_ = "invalid_observation_producer_epoch";
      return;
    }

    std::vector<const track_robot_interfaces::msg::SemanticObservation *> observations;
    observations.reserve(message->observations.size());
    for (const auto & observation : message->observations) {
      if (observation.producer_epoch_id != message->producer_epoch_id ||
        observation.visual_candidate_id == 0U ||
        !observation.camera_stamp_valid || observation.image_width == 0U ||
        observation.image_height == 0U || observation.roi.width == 0U ||
        observation.roi.height == 0U ||
        observation.roi.x_offset > observation.image_width ||
        observation.roi.y_offset > observation.image_height ||
        observation.roi.width >
        observation.image_width - observation.roi.x_offset ||
        observation.roi.height >
        observation.image_height - observation.roi.y_offset)
      {
        ++rejected_visual_observation_count_;
        continue;
      }
      const auto camera_stamp_ns = rclcpp::Time(observation.camera_stamp).nanoseconds();
      if (camera_stamp_ns < 0) {
        ++rejected_visual_observation_count_;
        continue;
      }
      observations.push_back(&observation);
    }
    std::sort(observations.begin(), observations.end(),
      [](const auto * left, const auto * right) {
        return left->visual_candidate_id < right->visual_candidate_id;
      });

    std::vector<ScoredVisualObservation> scored_visuals;
    scored_visuals.reserve(observations.size());
    std::size_t possible = 0U;
    bool oversized = false;
    for (const auto * observation : observations) {
      const auto visual_stamp_ns = rclcpp::Time(
        observation->camera_stamp).nanoseconds();
      const auto * buffered = association_lidar_batches_->nearest(
        visual_stamp_ns, association_max_source_time_delta_ns_,
        last_lidar_source_epoch_id_);
      if (buffered == nullptr) {
        ++rejected_visual_observation_count_;
        continue;
      }
      const auto & lidar_batch = buffered->value;
      std::vector<const track_robot_interfaces::msg::LidarTracklet *> tracklets;
      for (const auto & tracklet : lidar_batch.tracklets) {
        if (tracklet.active && tracklet.tracklet_id >= 0) {
          tracklets.push_back(&tracklet);
        }
      }
      std::sort(tracklets.begin(), tracklets.end(),
        [](const auto * left, const auto * right) {
          return left->tracklet_id < right->tracklet_id;
        });
      possible += tracklets.size();
      if (camera_attachment_enabled_ && possible > max_association_debug_pairs_) {
        oversized = true;
        break;
      }
      auto pairs = score_visual_pairs(
        *observation, lidar_batch, tracklets, evaluation_stamp_ns);
      if (association_shadow_mode_ &&
        possible > max_association_debug_pairs_)
      {
        const auto keep = max_association_debug_pairs_ -
          (possible - tracklets.size());
        pairs.resize(std::min(keep, pairs.size()));
      }
      scored_visuals.push_back(
        {observation, &lidar_batch, visual_stamp_ns, std::move(pairs)});
      if (association_shadow_mode_ && possible >= max_association_debug_pairs_) {
        break;
      }
    }

    if (oversized) {
      ++rejected_observation_batch_count_;
      dropped_association_debug_pair_count_ += possible;
      last_association_reason_ = "complete association matrix exceeds configured bound";
      return;
    }

    std::size_t emitted = 0U;
    if (association_shadow_mode_) {
      for (const auto & scored : scored_visuals) {
        std::size_t best_index = scored.pairs.size();
        double best_score = -std::numeric_limits<double>::infinity();
        double second_score = -std::numeric_limits<double>::infinity();
        for (std::size_t index = 0U; index < scored.pairs.size(); ++index) {
          if (!scored.pairs[index].score.accepted_by_gates) {
            continue;
          }
          const double value = scored.pairs[index].score.total_score;
          if (value > best_score) {
            second_score = best_score;
            best_score = value;
            best_index = index;
          } else if (value > second_score) {
            second_score = value;
          }
        }
        const double top_two_margin = best_index < scored.pairs.size() &&
          std::isfinite(second_score) ? best_score - second_score : 0.0;
        if (publish_association_debug_ && association_debug_publisher_) {
          for (std::size_t index = 0U; index < scored.pairs.size(); ++index) {
            const auto & pair = scored.pairs[index];
            const auto decision = pair.score.accepted_by_gates ?
              track_robot_interfaces::msg::AssociationDebug::DECISION_UNMATCHED :
              track_robot_interfaces::msg::AssociationDebug::DECISION_REJECTED_GATE;
            const std::string reason = pair.score.accepted_by_gates ?
              "shadow_mode_attachment_disabled" :
              bounded_reason(pair.score.rejection_reason);
            association_debug_publisher_->publish(association_debug_from_score(
                pair.score, scored.observation->header, current_memory_epoch_id_,
                message->producer_epoch_id,
                scored.observation->visual_candidate_id,
                scored.lidar_batch->source_epoch_id,
                pair.tracklet->tracklet_id, decision,
                index == best_index ? top_two_margin : 0.0,
                reason.empty() ? "shadow_gate_rejected" : reason));
            ++emitted;
          }
        }
      }
      association_debug_pair_count_ += emitted;
      dropped_association_debug_pair_count_ += possible > emitted ?
        possible - emitted : 0U;
      last_association_reason_ = emitted > 0U ?
        "shadow_debug_published" : "no_valid_candidate_pairs";
      return;
    }

    if (scored_visuals.empty()) {
      ++rejected_observation_batch_count_;
      last_association_reason_ = "no_visuals_have_source_time_lidar_evidence";
      return;
    }
    const auto common_stamp = scored_visuals.front().visual_stamp_ns;
    const auto common_lidar_epoch = scored_visuals.front().lidar_batch->source_epoch_id;
    const auto common_lidar_stamp = rclcpp::Time(
      scored_visuals.front().lidar_batch->header.stamp).nanoseconds();
    for (const auto & scored : scored_visuals) {
      if (scored.visual_stamp_ns != common_stamp ||
        scored.lidar_batch->source_epoch_id != common_lidar_epoch ||
        rclcpp::Time(scored.lidar_batch->header.stamp).nanoseconds() !=
        common_lidar_stamp)
      {
        ++rejected_observation_batch_count_;
        last_association_reason_ = "attachment requires one complete source-time frame";
        return;
      }
    }
    if (last_visual_source_stamp_ns_.has_value()) {
      if (common_stamp < *last_visual_source_stamp_ns_) {
        reset_runtime_association("visual_source_time_rollback");
      } else if (common_stamp == *last_visual_source_stamp_ns_) {
        ++rejected_observation_batch_count_;
        last_association_reason_ = "duplicate_visual_source_frame";
        return;
      }
    }
    RuntimeAssociationFrame runtime_frame;
    runtime_frame.frame_index = runtime_association_frame_index_ + 1U;
    std::map<std::int64_t, LidarAssociationKey> lidar_keys;
    for (const auto & scored : scored_visuals) {
      runtime_frame.visuals.push_back({
          scored.observation->visual_candidate_id,
          stable_visual_key(*scored.observation)});
      for (const auto & pair : scored.pairs) {
        const LidarAssociationKey lidar{
          scored.lidar_batch->source_epoch_id, pair.tracklet->tracklet_id};
        lidar_keys.emplace(pair.tracklet->tracklet_id, lidar);
        runtime_frame.pairs.push_back({
            scored.observation->visual_candidate_id, lidar,
            pair.score.total_score, pair.score.accepted_by_gates});
      }
    }
    for (const auto & item : lidar_keys) {
      runtime_frame.lidars.push_back(item.second);
    }

    RuntimeAssociationResult runtime_result;
    auto next_runtime_association = *runtime_association_;
    auto next_reidentification = *reidentification_;
    auto next_memory_core = *memory_core_;
    try {
      for (const auto & scored : scored_visuals) {
        const auto visual_key = stable_visual_key(*scored.observation);
        if (visual_key.has_value()) {
          (void)make_visual_supplement(
            *scored.observation, *visual_key,
            {common_lidar_epoch, 0}, 0.0, appearance_memory_enabled_);
        }
      }
      runtime_result = next_runtime_association.process(runtime_frame);
    } catch (const std::exception & error) {
      ++rejected_observation_batch_count_;
      last_association_reason_ = bounded_reason(
        std::string("runtime_association_rejected:") + error.what());
      return;
    }
    std::map<std::uint64_t, RuntimeAssociationDecision> decisions;
    for (const auto & decision : runtime_result.decisions) {
      decisions.emplace(decision.visual_candidate_id, decision);
    }
    std::map<std::uint64_t, std::uint64_t> attached_global_ids;
    std::map<GlobalObjectKey, CurrentAppearanceEvidence>
      current_reidentification_evidence;
    std::optional<MemoryUpdateResult> latest_snapshot;
    std::vector<std::pair<std_msgs::msg::Header, MemoryEvent>> pending_events;
    std::uint64_t accepted_in_batch = 0U;
    bool supplement_failed = false;
    for (const auto & scored : scored_visuals) {
      const auto found = decisions.find(scored.observation->visual_candidate_id);
      if (found == decisions.end()) {
        continue;
      }
      const auto & decision = found->second;
      if (decision.decision == ConfirmationDecision::kMatched &&
        decision.assigned_lidar.has_value() &&
        decision.attached_lidar == decision.assigned_lidar)
      {
        const auto visual_key = stable_visual_key(*scored.observation);
        if (visual_key.has_value()) {
          try {
            const auto supplement = make_visual_supplement(
              *scored.observation, *visual_key,
              *decision.attached_lidar, decision.best_score,
              appearance_memory_enabled_);
            auto applied = next_memory_core.supplement_visual(
              *current_domain_, supplement);
            if (!applied.accepted) {
              throw std::invalid_argument(applied.reason);
            }
            for (const auto & object : applied.snapshot.objects) {
              if (object.lidar_key == supplement.lidar_key) {
                attached_global_ids[scored.observation->visual_candidate_id] =
                  object.key.global_object_id;
                break;
              }
            }
            if (applied.current_appearance_evidence.has_value() &&
              !current_reidentification_evidence.emplace(
                applied.current_appearance_evidence->object_key,
                *applied.current_appearance_evidence).second)
            {
              throw std::invalid_argument(
                      "duplicate current appearance evidence for one object");
            }
            auto event_header = scored.observation->header;
            event_header.frame_id = current_domain_->canonical_frame_id();
            for (const auto & event : applied.snapshot.events) {
              pending_events.emplace_back(event_header, event);
            }
            latest_snapshot = std::move(applied.snapshot);
            ++accepted_in_batch;
          } catch (const std::exception & error) {
            ++rejected_visual_observation_count_;
            last_association_reason_ = bounded_reason(
              std::string("visual_supplement_rejected:") + error.what());
            supplement_failed = true;
            break;
          }
        }
      }
    }
    if (supplement_failed) {
      ++rejected_observation_batch_count_;
      return;
    }

    try {
      std::vector<CurrentAppearanceEvidence> current_evidence;
      current_evidence.reserve(current_reidentification_evidence.size());
      for (const auto & item : current_reidentification_evidence) {
        current_evidence.push_back(item.second);
      }
      const auto reidentification_frame =
        next_memory_core.make_reidentification_frame(
        *current_domain_, ++reidentification_source_frame_index_,
        reidentification_config_, current_evidence);
      const auto reidentification_result =
        next_reidentification.process(reidentification_frame);
      std::vector<ReidentificationStateUpdate> state_updates;
      for (const auto & decision : reidentification_result.decisions) {
        const bool pending =
          decision.decision == ReidentificationDecision::kTentative ||
          (decision.decision == ReidentificationDecision::kConfirmed &&
          !reidentification_mutation_enabled_);
        state_updates.push_back({
            decision.lost_key,
            pending ? ReidentificationState::kPending :
            ReidentificationState::kRejected});
      }
      latest_snapshot = next_memory_core.apply_reidentification_states(
        *current_domain_, state_updates);
      if (reidentification_mutation_enabled_) {
        for (const auto & decision : reidentification_result.decisions) {
          if (decision.decision != ReidentificationDecision::kConfirmed) {
            continue;
          }
          auto transferred = next_memory_core.reidentify(
            *current_domain_, decision.lost_key, decision.candidate_key,
            decision.expected_candidate_lidar_key,
            decision.expected_candidate_visual_key);
          if (!transferred.accepted) {
            throw std::invalid_argument(transferred.reason);
          }
          pending_events.erase(std::remove_if(
              pending_events.begin(), pending_events.end(),
              [&decision](const auto & item) {
                return item.second.object_key == decision.candidate_key &&
                       (item.second.type == MemoryEventType::kAssociationAttached ||
                       item.second.type == MemoryEventType::kAssociationDetached);
              }), pending_events.end());
          auto event_header = message->header;
          event_header.frame_id = current_domain_->canonical_frame_id();
          for (const auto & event : transferred.snapshot.events) {
            pending_events.emplace_back(event_header, event);
          }
          latest_snapshot = std::move(transferred.snapshot);
        }
      }
    } catch (const std::exception & error) {
      ++rejected_observation_batch_count_;
      last_association_reason_ = bounded_reason(
        std::string("runtime_reidentification_rejected:") + error.what());
      return;
    }

    std::optional<RuntimeTaskServiceCoordinator> next_task_services;
    if (latest_snapshot.has_value()) {
      try {
        next_task_services.emplace(synchronized_task_services(
            *latest_snapshot, next_memory_core));
      } catch (const std::exception & error) {
        ++rejected_observation_batch_count_;
        last_association_reason_ = bounded_reason(
          std::string("runtime_task_sync_rejected:") + error.what());
        return;
      }
    }
    *runtime_association_ = std::move(next_runtime_association);
    *reidentification_ = std::move(next_reidentification);
    *memory_core_ = std::move(next_memory_core);
    if (next_task_services.has_value()) {
      *task_services_ = std::move(*next_task_services);
    }
    runtime_association_frame_index_ = runtime_frame.frame_index;
    last_visual_source_stamp_ns_ = common_stamp;
    accepted_camera_attachment_count_ += accepted_in_batch;
    if (publish_events_) {
      for (const auto & item : pending_events) {
        events_publisher_->publish(semantic_event_from_memory(
            item.second, item.first, ++event_sequence_,
            current_memory_epoch_id_));
      }
    }
    if (latest_snapshot.has_value()) {
      auto output_header = message->header;
      output_header.frame_id = current_domain_->canonical_frame_id();
      publish_runtime_snapshots(output_header);
    }

    if (publish_association_debug_ && association_debug_publisher_) {
      for (const auto & scored : scored_visuals) {
        const auto & decision = decisions.at(scored.observation->visual_candidate_id);
        for (const auto & pair : scored.pairs) {
          std::uint8_t debug_decision = pair.score.accepted_by_gates ?
            track_robot_interfaces::msg::AssociationDebug::DECISION_UNMATCHED :
            track_robot_interfaces::msg::AssociationDebug::DECISION_REJECTED_GATE;
          if (decision.assigned_lidar.has_value() &&
            decision.assigned_lidar->tracklet_id == pair.tracklet->tracklet_id)
          {
            switch (decision.decision) {
              case ConfirmationDecision::kTentative:
              case ConfirmationDecision::kCooldown:
                debug_decision = track_robot_interfaces::msg::AssociationDebug::DECISION_TENTATIVE;
                break;
              case ConfirmationDecision::kMatched:
                debug_decision = track_robot_interfaces::msg::AssociationDebug::DECISION_MATCHED;
                break;
              case ConfirmationDecision::kAmbiguous:
                debug_decision = track_robot_interfaces::msg::AssociationDebug::DECISION_AMBIGUOUS;
                break;
              case ConfirmationDecision::kUnmatched:
                debug_decision = track_robot_interfaces::msg::AssociationDebug::DECISION_UNMATCHED;
                break;
            }
          }
          auto debug = association_debug_from_score(
            pair.score, scored.observation->header, current_memory_epoch_id_,
            message->producer_epoch_id, scored.observation->visual_candidate_id,
            scored.lidar_batch->source_epoch_id, pair.tracklet->tracklet_id,
            debug_decision,
            decision.assigned_lidar.has_value() &&
            decision.assigned_lidar->tracklet_id == pair.tracklet->tracklet_id ?
            decision.margin : 0.0,
            pair.score.accepted_by_gates ? decision.reason :
            bounded_reason(pair.score.rejection_reason));
          const auto global = attached_global_ids.find(
            scored.observation->visual_candidate_id);
          if (global != attached_global_ids.end()) {
            debug.global_object_id_valid = true;
            debug.global_object_id = global->second;
          }
          association_debug_publisher_->publish(std::move(debug));
          ++emitted;
        }
      }
    }
    association_debug_pair_count_ += emitted;
    last_association_reason_ = accepted_in_batch > 0U ?
      "runtime_attachment_evaluated" : "runtime_association_no_attachment";
  }

  void publish_rejection_event(
    const std_msgs::msg::Header & header,
    std::uint64_t lidar_source_epoch_id,
    const PendingRejection & rejection)
  {
    track_robot_interfaces::msg::SemanticMemoryEvent event;
    event.header = header;
    event.sequence = ++event_sequence_;
    event.memory_epoch_id = current_memory_epoch_id_;
    event.event_type = event.EVENT_OBSERVATION_REJECTED;
    event.lidar_source_epoch_id = lidar_source_epoch_id;
    event.lidar_tracklet_id = rejection.tracklet_id;
    event.reason = rejection.reason;
    events_publisher_->publish(event);
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray output;
    output.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "semantic_memory/core";
    status.hardware_id = "track_robot_semantic_memory";
    if (!enabled_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      status.message = "disabled";
    } else if (!current_domain_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting_for_valid_localization_domain";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = memory_core_ ? "running" : "waiting_for_lidar";
    }
    status.values = {
      make_value("localization_messages", localization_message_count_),
      make_value("rejected_localization", rejected_localization_count_),
      make_value("lidar_messages", lidar_message_count_),
      make_value("observation_messages", observation_message_count_),
      make_value("rejected_observation_batches", rejected_observation_batch_count_),
      make_value("rejected_visual_observations", rejected_visual_observation_count_),
      make_value("camera_info_messages", camera_info_message_count_),
      make_value("rejected_camera_info", rejected_camera_info_count_),
      make_value("association_debug_pairs", association_debug_pair_count_),
      make_value("accepted_camera_attachments", accepted_camera_attachment_count_),
      make_value("runtime_association_resets", runtime_association_reset_count_),
      make_value(
        "dropped_association_debug_pairs", dropped_association_debug_pair_count_),
      make_value("rejected_lidar_batches", rejected_lidar_batch_count_),
      make_value("rejected_observations", rejected_observation_count_),
      make_value("producer_dropped_tracklets", producer_dropped_tracklet_count_),
      make_value("task_messages", task_message_count_),
      make_value("rejected_tasks", rejected_task_count_),
      make_value("service_calls", service_call_count_),
      make_value("active_objects", latest_active_object_count_),
      make_value("memory_epoch_id", current_memory_epoch_id_),
      make_value("snapshot_sequence", snapshot_sequence_),
      make_value("event_sequence", event_sequence_),
      make_value("localization_reason", last_localization_reason_),
      make_value("lidar_reason", last_lidar_reason_),
      make_value("camera_info_reason", last_camera_info_reason_),
      make_value("association_reason", last_association_reason_)};
    status.values.push_back(make_value("task_reason", last_task_reason_));
    status.values.push_back(make_value(
        "best_candidate_reason", last_best_candidate_reason_));
    output.status.push_back(status);
    diagnostics_publisher_->publish(output);
  }

  static diagnostic_msgs::msg::KeyValue make_value(
    const std::string & key, std::uint64_t value)
  {
    return make_value(key, std::to_string(value));
  }

  static diagnostic_msgs::msg::KeyValue make_value(
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue output;
    output.key = key;
    output.value = value;
    return output;
  }

  bool enabled_{true};
  bool publish_active_{true};
  bool publish_events_{true};
  bool publish_association_debug_{true};
  bool association_shadow_mode_{true};
  bool camera_attachment_enabled_{false};
  bool appearance_memory_enabled_{true};
  bool reidentification_shadow_mode_{true};
  bool reidentification_mutation_enabled_{false};
  bool publish_best_candidate_{true};
  std::size_t observation_queue_depth_{1U};
  std::size_t task_queue_depth_{1U};
  std::size_t max_association_debug_pairs_{1024U};
  std::string observations_topic_;
  std::string association_debug_topic_;
  std::string camera_info_topic_;
  std::string tasks_topic_;
  std::string camera_calibration_id_;
  std::uint64_t initial_memory_epoch_override_{0U};
  std::uint64_t current_memory_epoch_id_{0U};
  std::uint64_t localization_message_count_{0U};
  std::uint64_t rejected_localization_count_{0U};
  std::uint64_t lidar_message_count_{0U};
  std::uint64_t observation_message_count_{0U};
  std::uint64_t rejected_observation_batch_count_{0U};
  std::uint64_t rejected_visual_observation_count_{0U};
  std::uint64_t camera_info_message_count_{0U};
  std::uint64_t rejected_camera_info_count_{0U};
  std::uint64_t association_debug_pair_count_{0U};
  std::uint64_t accepted_camera_attachment_count_{0U};
  std::uint64_t runtime_association_reset_count_{0U};
  std::uint64_t runtime_association_frame_index_{0U};
  std::uint64_t reidentification_source_frame_index_{0U};
  std::uint64_t last_visual_producer_epoch_id_{0U};
  std::uint64_t last_lidar_source_epoch_id_{0U};
  std::uint64_t dropped_association_debug_pair_count_{0U};
  std::uint64_t rejected_lidar_batch_count_{0U};
  std::uint64_t rejected_observation_count_{0U};
  std::uint64_t producer_dropped_tracklet_count_{0U};
  std::uint64_t latest_active_object_count_{0U};
  std::uint64_t snapshot_sequence_{0U};
  std::uint64_t event_sequence_{0U};
  std::uint64_t best_candidate_sequence_{0U};
  std::uint64_t task_message_count_{0U};
  std::uint64_t rejected_task_count_{0U};
  std::uint64_t service_call_count_{0U};
  std::string last_localization_reason_{"not_received"};
  std::string last_lidar_reason_{"not_received"};
  std::string last_camera_info_reason_{"not_received"};
  std::string last_association_reason_{"not_received"};
  std::string last_task_reason_{"not_received"};
  std::string last_best_candidate_reason_{"not_published"};
  std::chrono::nanoseconds tf_lookup_timeout_{30000000};
  std::int64_t localization_state_timeout_ns_{500000000};
  std::int64_t association_max_source_time_delta_ns_{100000000};
  std::optional<std::int64_t> last_visual_source_stamp_ns_;
  MemoryCoreConfig core_config_;
  ReidentificationConfig reidentification_config_;
  TaskRelevanceConfig task_relevance_config_;
  BestCandidateConfig best_candidate_config_;
  std::optional<MemoryDomainKey> current_domain_;
  std::optional<MemoryDomainKey> runtime_view_domain_;
  std::optional<std::int64_t> current_localization_stamp_ns_;
  std::optional<CameraModel> camera_model_;
  std::string camera_frame_id_;
  std::optional<track_robot_interfaces::msg::SemanticLidarTrackletArray>
    latest_lidar_batch_;
  std::optional<track_robot_interfaces::msg::SemanticTask> pending_task_;
  bool pending_task_clear_requested_{false};
  std::uint64_t pending_task_producer_epoch_id_{0U};
  std::optional<std::int64_t> pending_task_source_stamp_ns_;
  std::unique_ptr<SourceTimeBuffer<
      track_robot_interfaces::msg::SemanticLidarTrackletArray>>
    association_lidar_batches_;
  std::unique_ptr<MemoryCore> memory_core_;
  std::unique_ptr<CrossModalAssociator> associator_;
  std::unique_ptr<RuntimeAssociationCoordinator> runtime_association_;
  std::unique_ptr<RuntimeReidentificationCoordinator> reidentification_;
  std::unique_ptr<RuntimeTaskServiceCoordinator> task_services_;
  CameraLidarProjector projector_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::Publisher<track_robot_interfaces::msg::SemanticObjectArray>::SharedPtr
    active_objects_publisher_;
  rclcpp::Publisher<track_robot_interfaces::msg::SemanticMemoryEvent>::SharedPtr
    events_publisher_;
  rclcpp::Publisher<track_robot_interfaces::msg::AssociationDebug>::SharedPtr
    association_debug_publisher_;
  rclcpp::Publisher<track_robot_interfaces::msg::SemanticObjectArray>::SharedPtr
    best_candidate_publisher_;
  rclcpp::Subscription<
    track_robot_interfaces::msg::SemanticLocalizationState>::SharedPtr
    localization_subscription_;
  rclcpp::Subscription<
    track_robot_interfaces::msg::SemanticLidarTrackletArray>::SharedPtr
    lidar_subscription_;
  rclcpp::Subscription<
    track_robot_interfaces::msg::SemanticObservationArray>::SharedPtr
    observation_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    camera_info_subscription_;
  rclcpp::Subscription<track_robot_interfaces::msg::SemanticTask>::SharedPtr
    task_subscription_;
  rclcpp::Service<track_robot_interfaces::srv::GetSemanticObject>::SharedPtr
    get_object_service_;
  rclcpp::Service<track_robot_interfaces::srv::QuerySemanticObjects>::SharedPtr
    query_objects_service_;
  rclcpp::Service<
    track_robot_interfaces::srv::MarkSemanticObjectInspected>::SharedPtr
    mark_inspected_service_;
  rclcpp::Service<track_robot_interfaces::srv::ResetSemanticMemory>::SharedPtr
    reset_memory_service_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace track_robot_semantic_memory

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<track_robot_semantic_memory::SemanticMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
