#include "track_robot_semantic_memory/visualization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>
#include <set>
#include <stdexcept>
#include <vector>

#include "track_robot_interfaces/msg/semantic_object.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace track_robot_semantic_memory
{
namespace
{

constexpr const char * kObjectNamespace = "semantic_memory_objects";
constexpr const char * kLabelNamespace = "semantic_memory_labels";
constexpr const char * kBestNamespace = "semantic_memory_best_candidate";
constexpr const char * kBestLabelNamespace =
  "semantic_memory_best_candidate_label";
constexpr double kMinimumMarkerExtent = 0.05;

bool finite(double value)
{
  return std::isfinite(value);
}

const char * lifecycle_name(std::uint8_t state)
{
  using Object = track_robot_interfaces::msg::SemanticObject;
  switch (state) {
    case Object::LIFECYCLE_TENTATIVE:
      return "tentative";
    case Object::LIFECYCLE_CONFIRMED:
      return "confirmed";
    case Object::LIFECYCLE_STALE:
      return "stale";
    case Object::LIFECYCLE_LOST:
      return "lost";
    case Object::LIFECYCLE_ARCHIVED:
      return "archived";
    default:
      return "invalid-lifecycle";
  }
}

const char * support_name(std::uint8_t state)
{
  using Object = track_robot_interfaces::msg::SemanticObject;
  switch (state) {
    case Object::SUPPORT_NONE:
      return "no-support";
    case Object::SUPPORT_CAMERA_LIDAR:
      return "camera+lidar";
    case Object::SUPPORT_CAMERA_ONLY:
      return "camera-only";
    case Object::SUPPORT_LIDAR_ONLY:
      return "lidar-only";
    case Object::SUPPORT_PREDICTION_ONLY:
      return "prediction-only";
    default:
      return "invalid-support";
  }
}

const char * motion_name(std::uint8_t state)
{
  using Object = track_robot_interfaces::msg::SemanticObject;
  switch (state) {
    case Object::MOTION_STATIC:
      return "static";
    case Object::MOTION_DYNAMIC:
      return "dynamic";
    case Object::MOTION_UNCERTAIN:
      return "motion-uncertain";
    case Object::MOTION_TEMPORARILY_MOVING:
      return "temporarily-moving";
    default:
      return "invalid-motion";
  }
}

visualization_msgs::msg::Marker make_object_marker(
  const track_robot_interfaces::msg::SemanticObjectArray & snapshot,
  const track_robot_interfaces::msg::SemanticObject & object,
  std::int32_t marker_id)
{
  using Object = track_robot_interfaces::msg::SemanticObject;
  visualization_msgs::msg::Marker marker;
  marker.header = snapshot.header;
  marker.ns = kObjectNamespace;
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = object.position;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = std::max(kMinimumMarkerExtent, object.extent.x);
  marker.scale.y = std::max(kMinimumMarkerExtent, object.extent.y);
  marker.scale.z = std::max(kMinimumMarkerExtent, object.extent.z);

  if (object.lifecycle_state == Object::LIFECYCLE_TENTATIVE) {
    marker.color.r = 1.0F;
    marker.color.g = 0.85F;
    marker.color.b = 0.1F;
  } else if (object.lifecycle_state == Object::LIFECYCLE_STALE) {
    marker.color.r = 1.0F;
    marker.color.g = 0.45F;
    marker.color.b = 0.05F;
  } else if (object.motion_class == Object::MOTION_DYNAMIC) {
    marker.color.r = 0.1F;
    marker.color.g = 0.3F;
    marker.color.b = 0.95F;
  } else {
    marker.color.r = 0.1F;
    marker.color.g = 0.85F;
    marker.color.b = 0.25F;
  }
  marker.color.a = object.support_state == Object::SUPPORT_PREDICTION_ONLY ?
    0.35F : 0.65F;
  return marker;
}

visualization_msgs::msg::Marker make_label_marker(
  const track_robot_interfaces::msg::SemanticObjectArray & snapshot,
  const track_robot_interfaces::msg::SemanticObject & object,
  std::int32_t marker_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = snapshot.header;
  marker.ns = kLabelNamespace;
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = object.position;
  marker.pose.position.z +=
    std::max(kMinimumMarkerExtent, object.extent.z * 0.5) + 0.15;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.12;
  marker.color.r = 1.0F;
  marker.color.g = 1.0F;
  marker.color.b = 1.0F;
  marker.color.a = 0.95F;

  std::ostringstream text;
  text << "object " << object.global_object_id << '\n'
       << lifecycle_name(object.lifecycle_state) << " | "
       << support_name(object.support_state) << " | "
       << motion_name(object.motion_class);
  if (object.active_query_id != 0U && object.active_query_version != 0U &&
    finite(object.task_relevance))
  {
    text.setf(std::ios::fixed);
    text.precision(3);
    text << "\nrelevance=" << object.task_relevance;
  }
  marker.text = text.str();
  return marker;
}

visualization_msgs::msg::Marker make_best_marker(
  const track_robot_interfaces::msg::SemanticObjectArray & snapshot,
  const track_robot_interfaces::msg::SemanticObject & object,
  std::int32_t marker_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = snapshot.header;
  marker.ns = kBestNamespace;
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = object.position;
  marker.pose.orientation.w = 1.0;
  marker.scale.x =
    std::max(kMinimumMarkerExtent, object.extent.x * 1.15 + 0.03);
  marker.scale.y =
    std::max(kMinimumMarkerExtent, object.extent.y * 1.15 + 0.03);
  marker.scale.z =
    std::max(kMinimumMarkerExtent, object.extent.z * 1.15 + 0.03);
  marker.color.r = 1.0F;
  marker.color.g = 0.05F;
  marker.color.b = 0.85F;
  marker.color.a = 0.22F;
  return marker;
}

visualization_msgs::msg::Marker make_best_label_marker(
  const track_robot_interfaces::msg::SemanticObjectArray & snapshot,
  const track_robot_interfaces::msg::SemanticObject & object,
  std::int32_t marker_id)
{
  auto marker = make_label_marker(snapshot, object, marker_id);
  marker.ns = kBestLabelNamespace;
  marker.pose.position.z += 0.17;
  marker.scale.z = 0.14;
  marker.color.r = 1.0F;
  marker.color.g = 0.2F;
  marker.color.b = 0.9F;
  marker.text = "BEST CANDIDATE";
  return marker;
}

visualization_msgs::msg::Marker make_delete_marker(
  const track_robot_interfaces::msg::SemanticObjectArray & snapshot,
  const char * marker_namespace,
  std::int32_t marker_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = snapshot.header;
  marker.ns = marker_namespace;
  marker.id = marker_id;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  return marker;
}

}  // namespace

MarkerRegistry::MarkerRegistry(std::size_t max_objects)
: max_objects_(max_objects)
{
  if (max_objects_ == 0U || max_objects_ > 256U) {
    throw std::invalid_argument("visualizer max_objects must be in [1,256]");
  }
}

void MarkerRegistry::set_best_candidate(
  std::optional<GlobalObjectKey> candidate)
{
  if (candidate.has_value() && !candidate->valid()) {
    throw std::invalid_argument("best candidate key must be valid");
  }
  best_candidate_ = candidate;
}

visualization_msgs::msg::MarkerArray MarkerRegistry::update(
  const track_robot_interfaces::msg::SemanticObjectArray & snapshot)
{
  if (snapshot.memory_epoch_id == 0U || snapshot.snapshot_sequence == 0U ||
    (snapshot.memory_epoch_id == last_memory_epoch_id_ &&
    snapshot.snapshot_sequence <= last_snapshot_sequence_) ||
    snapshot.header.frame_id.empty() || snapshot.objects.size() > max_objects_)
  {
    throw std::invalid_argument("semantic snapshot is invalid, oversized, or out of order");
  }

  std::set<GlobalObjectKey> seen_keys;
  std::map<GlobalObjectKey, const track_robot_interfaces::msg::SemanticObject *> objects;
  for (const auto & object : snapshot.objects) {
    const GlobalObjectKey key{object.memory_epoch_id, object.global_object_id};
    if (!key.valid() || key.memory_epoch_id != snapshot.memory_epoch_id) {
      throw std::invalid_argument("semantic object cannot be visualized safely");
    }
    if (!seen_keys.emplace(key).second) {
      throw std::invalid_argument("semantic snapshot contains a duplicate object key");
    }
    if (!object.position_valid || !object.extent_valid) {
      continue;
    }
    if (
      !finite(object.position.x) || !finite(object.position.y) ||
      !finite(object.position.z) || !finite(object.extent.x) ||
      !finite(object.extent.y) || !finite(object.extent.z) ||
      object.extent.x < 0.0 || object.extent.y < 0.0 || object.extent.z < 0.0)
    {
      throw std::invalid_argument("semantic object cannot be visualized safely");
    }
    objects.emplace(key, &object);
  }

  std::vector<GlobalObjectKey> removed_keys;
  for (const auto & assignment : marker_by_key_) {
    if (objects.count(assignment.first) == 0U) {
      removed_keys.push_back(assignment.first);
    }
  }

  auto next_marker_by_key = marker_by_key_;
  auto next_key_by_marker = key_by_marker_;
  visualization_msgs::msg::MarkerArray output;
  output.markers.reserve(
    objects.size() * 2U + removed_keys.size() * 2U + 4U);
  std::optional<GlobalObjectKey> current_best;
  if (best_candidate_.has_value() &&
    best_candidate_->memory_epoch_id == snapshot.memory_epoch_id &&
    objects.count(*best_candidate_) != 0U)
  {
    current_best = best_candidate_;
  }
  for (const auto & entry : objects) {
    auto assignment = next_marker_by_key.find(entry.first);
    if (assignment == next_marker_by_key.end()) {
      const auto id = allocate_marker_id(entry.first, next_key_by_marker);
      assignment = next_marker_by_key.emplace(entry.first, id).first;
      next_key_by_marker.emplace(id, entry.first);
    }
    output.markers.push_back(
      make_object_marker(snapshot, *entry.second, assignment->second));
    output.markers.push_back(
      make_label_marker(snapshot, *entry.second, assignment->second));
    if (current_best.has_value() && entry.first == *current_best) {
      output.markers.push_back(
        make_best_marker(snapshot, *entry.second, assignment->second));
      output.markers.push_back(
        make_best_label_marker(snapshot, *entry.second, assignment->second));
    }
  }
  for (const auto & key : removed_keys) {
    const auto assignment = next_marker_by_key.find(key);
    output.markers.push_back(
      make_delete_marker(snapshot, kObjectNamespace, assignment->second));
    output.markers.push_back(
      make_delete_marker(snapshot, kLabelNamespace, assignment->second));
    next_key_by_marker.erase(assignment->second);
    next_marker_by_key.erase(assignment);
  }
  if (published_best_candidate_.has_value() &&
    (!current_best.has_value() ||
    *published_best_candidate_ != *current_best))
  {
    const auto old_assignment =
      marker_by_key_.find(*published_best_candidate_);
    if (old_assignment != marker_by_key_.end()) {
      output.markers.push_back(
        make_delete_marker(
          snapshot, kBestNamespace, old_assignment->second));
      output.markers.push_back(
        make_delete_marker(
          snapshot, kBestLabelNamespace, old_assignment->second));
    }
  }

  marker_by_key_ = std::move(next_marker_by_key);
  key_by_marker_ = std::move(next_key_by_marker);
  last_memory_epoch_id_ = snapshot.memory_epoch_id;
  last_snapshot_sequence_ = snapshot.snapshot_sequence;
  published_best_candidate_ = current_best;
  return output;
}

std::int32_t MarkerRegistry::base_marker_id(const GlobalObjectKey & key) noexcept
{
  const std::uint64_t folded = key.memory_epoch_id ^
    (key.memory_epoch_id >> 32U) ^ key.global_object_id ^
    (key.global_object_id >> 32U);
  return static_cast<std::int32_t>(folded & 0x7fffffffU);
}

std::int32_t MarkerRegistry::allocate_marker_id(
  const GlobalObjectKey & key,
  const std::map<std::int32_t, GlobalObjectKey> & occupied) const
{
  std::uint32_t candidate = static_cast<std::uint32_t>(base_marker_id(key));
  for (std::size_t attempt = 0U; attempt <= occupied.size(); ++attempt) {
    const auto id = static_cast<std::int32_t>(candidate & 0x7fffffffU);
    if (occupied.count(id) == 0U) {
      return id;
    }
    candidate = (candidate + 1U) & 0x7fffffffU;
  }
  throw std::overflow_error("unable to allocate a bounded RViz marker ID");
}

}  // namespace track_robot_semantic_memory
