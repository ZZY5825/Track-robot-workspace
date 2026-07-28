#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>

#include "track_robot_interfaces/msg/semantic_object_array.hpp"
#include "track_robot_semantic_memory/id_types.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace track_robot_semantic_memory
{

class MarkerRegistry
{
public:
  explicit MarkerRegistry(std::size_t max_objects);

  void set_best_candidate(std::optional<GlobalObjectKey> candidate);

  visualization_msgs::msg::MarkerArray update(
    const track_robot_interfaces::msg::SemanticObjectArray & snapshot);

private:
  static std::int32_t base_marker_id(const GlobalObjectKey & key) noexcept;
  std::int32_t allocate_marker_id(
    const GlobalObjectKey & key,
    const std::map<std::int32_t, GlobalObjectKey> & occupied) const;

  std::size_t max_objects_;
  std::uint64_t last_memory_epoch_id_{0U};
  std::uint64_t last_snapshot_sequence_{0U};
  std::map<GlobalObjectKey, std::int32_t> marker_by_key_;
  std::map<std::int32_t, GlobalObjectKey> key_by_marker_;
  std::optional<GlobalObjectKey> best_candidate_;
  std::optional<GlobalObjectKey> published_best_candidate_;
};

}  // namespace track_robot_semantic_memory
