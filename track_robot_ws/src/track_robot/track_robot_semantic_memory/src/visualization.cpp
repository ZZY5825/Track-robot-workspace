#include "track_robot_semantic_memory/visualization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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
constexpr double kMinimumMarkerExtent = 0.05;

bool finite(double value)
{
  return std::isfinite(value);
}

visualization_msgs::msg::Marker make_add_marker(
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

visualization_msgs::msg::Marker make_delete_marker(
  const track_robot_interfaces::msg::SemanticObjectArray & snapshot,
  std::int32_t marker_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = snapshot.header;
  marker.ns = kObjectNamespace;
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

  std::map<GlobalObjectKey, const track_robot_interfaces::msg::SemanticObject *> objects;
  for (const auto & object : snapshot.objects) {
    const GlobalObjectKey key{object.memory_epoch_id, object.global_object_id};
    if (!key.valid() || key.memory_epoch_id != snapshot.memory_epoch_id ||
      !object.position_valid || !object.extent_valid ||
      !finite(object.position.x) || !finite(object.position.y) ||
      !finite(object.position.z) || !finite(object.extent.x) ||
      !finite(object.extent.y) || !finite(object.extent.z) ||
      object.extent.x < 0.0 || object.extent.y < 0.0 || object.extent.z < 0.0)
    {
      throw std::invalid_argument("semantic object cannot be visualized safely");
    }
    if (!objects.emplace(key, &object).second) {
      throw std::invalid_argument("semantic snapshot contains a duplicate object key");
    }
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
  output.markers.reserve(objects.size() + removed_keys.size());
  for (const auto & entry : objects) {
    auto assignment = next_marker_by_key.find(entry.first);
    if (assignment == next_marker_by_key.end()) {
      const auto id = allocate_marker_id(entry.first, next_key_by_marker);
      assignment = next_marker_by_key.emplace(entry.first, id).first;
      next_key_by_marker.emplace(id, entry.first);
    }
    output.markers.push_back(
      make_add_marker(snapshot, *entry.second, assignment->second));
  }
  for (const auto & key : removed_keys) {
    const auto assignment = next_marker_by_key.find(key);
    output.markers.push_back(make_delete_marker(snapshot, assignment->second));
    next_key_by_marker.erase(assignment->second);
    next_marker_by_key.erase(assignment);
  }

  marker_by_key_ = std::move(next_marker_by_key);
  key_by_marker_ = std::move(next_key_by_marker);
  last_memory_epoch_id_ = snapshot.memory_epoch_id;
  last_snapshot_sequence_ = snapshot.snapshot_sequence;
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
