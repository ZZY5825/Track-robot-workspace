#pragma once

#include <cstdint>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "track_robot_interfaces/msg/lidar_tracklet.hpp"
#include "track_robot_interfaces/msg/association_debug.hpp"
#include "track_robot_interfaces/msg/semantic_localization_state.hpp"
#include "track_robot_interfaces/msg/semantic_memory_event.hpp"
#include "track_robot_interfaces/msg/semantic_observation.hpp"
#include "track_robot_interfaces/msg/semantic_object.hpp"
#include "track_robot_interfaces/msg/semantic_object_array.hpp"
#include "track_robot_semantic_memory/memory_core.hpp"
#include "track_robot_semantic_memory/memory_domain.hpp"
#include "track_robot_semantic_memory/cross_modal_associator.hpp"
#include "track_robot_semantic_memory/runtime_task_services.hpp"

namespace track_robot_semantic_memory
{

bool source_times_within_tolerance(
  std::int64_t first_ns,
  std::int64_t second_ns,
  std::int64_t tolerance_ns);

std::uint64_t derive_memory_epoch_seed(
  const MemoryDomainKey & domain,
  std::uint64_t lidar_source_epoch_id);

MemoryDomainKey domain_from_localization_state(
  const track_robot_interfaces::msg::SemanticLocalizationState & state);

LidarObservation lidar_observation_from_tracklet(
  const track_robot_interfaces::msg::LidarTracklet & tracklet,
  std::uint64_t source_epoch_id,
  const geometry_msgs::msg::TransformStamped & transform);

VisualMemorySupplement visual_supplement_from_semantic_observation(
  const track_robot_interfaces::msg::SemanticObservation & observation,
  const VisualAssociationKey & visual_key,
  const LidarAssociationKey & lidar_key,
  double association_confidence,
  bool appearance_memory_enabled);

track_robot_interfaces::msg::SemanticObject semantic_object_from_memory(
  const MemoryObject & object,
  const MemoryDomainKey & domain);

track_robot_interfaces::msg::SemanticObject semantic_object_from_runtime_view(
  const RuntimeObjectView & view,
  const MemoryDomainKey & domain);

track_robot_interfaces::msg::SemanticObjectArray semantic_object_array_from_result(
  const MemoryUpdateResult & result,
  const MemoryDomainKey & domain,
  const std_msgs::msg::Header & header,
  std::uint64_t snapshot_sequence);

track_robot_interfaces::msg::SemanticMemoryEvent semantic_event_from_memory(
  const MemoryEvent & event,
  const std_msgs::msg::Header & header,
  std::uint64_t sequence,
  std::uint64_t memory_epoch_id);

track_robot_interfaces::msg::AssociationDebug association_debug_from_score(
  const PairAssociationScore & score,
  const std_msgs::msg::Header & header,
  std::uint64_t memory_epoch_id,
  std::uint64_t observation_producer_epoch_id,
  std::uint64_t visual_candidate_id,
  std::uint64_t lidar_source_epoch_id,
  std::int64_t lidar_tracklet_id,
  std::uint8_t decision,
  double top_two_margin,
  const std::string & reason);

}  // namespace track_robot_semantic_memory
