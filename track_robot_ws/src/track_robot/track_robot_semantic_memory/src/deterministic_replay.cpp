#include "track_robot_semantic_memory/deterministic_replay.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "track_robot_semantic_memory/hungarian_assignment.hpp"
#include "track_robot_semantic_memory/memory_core.hpp"
#include "track_robot_semantic_memory/task_relevance_scorer.hpp"

namespace track_robot_semantic_memory
{
namespace
{

using Json = nlohmann::json;

template<std::size_t Size>
std::array<double, Size> fixed_array(const Json & value, const char * name)
{
  if (!value.is_array() || value.size() != Size) {
    throw std::invalid_argument(std::string(name) + " has an invalid size");
  }
  std::array<double, Size> output{};
  for (std::size_t index = 0U; index < Size; ++index) {
    if (!value[index].is_number()) {
      throw std::invalid_argument(std::string(name) + " must contain numbers");
    }
    output[index] = value[index].get<double>();
  }
  return output;
}

MemoryMode memory_mode(const std::string & value)
{
  if (value == "OBSERVATION_ONLY") {
    return MemoryMode::kObservationOnly;
  }
  if (value == "LOCAL_SESSION") {
    return MemoryMode::kLocalSession;
  }
  if (value == "WORLD") {
    return MemoryMode::kWorld;
  }
  throw std::invalid_argument("normalized replay memory mode is unsupported");
}

LidarObservation lidar_observation(const Json & value)
{
  if (!value.is_object()) {
    throw std::invalid_argument("normalized LiDAR observation must be an object");
  }
  LidarObservation output;
  output.source_key = {
    value.at("source_epoch_id").get<std::uint64_t>(),
    value.at("tracklet_id").get<std::int64_t>()};
  output.source_stamp_ns = value.at("source_stamp_ns").get<std::int64_t>();
  output.position = fixed_array<3U>(value.at("position"), "position");
  output.velocity = fixed_array<3U>(value.at("velocity"), "velocity");
  output.extent = fixed_array<3U>(value.at("extent"), "extent");
  output.position_covariance = fixed_array<9U>(
    value.at("position_covariance"), "position_covariance");
  output.confidence = value.at("confidence").get<double>();
  output.validate();
  return output;
}

AppearanceDescriptor appearance_descriptor(const Json & value)
{
  if (!value.is_object() || !value.at("values").is_array() ||
    value.at("values").size() > 1024U)
  {
    throw std::invalid_argument("normalized appearance descriptor is invalid");
  }
  AppearanceDescriptor output;
  output.encoder_id = value.at("encoder_id").get<std::string>();
  output.checkpoint_id = value.at("checkpoint_id").get<std::string>();
  output.version = value.at("version").get<std::uint32_t>();
  output.dimension = value.at("dimension").get<std::uint16_t>();
  output.l2_normalized = value.at("l2_normalized").get<bool>();
  output.values = value.at("values").get<std::vector<double>>();
  return output;
}

Json association_output(const Json & value)
{
  const auto row_ids = value.at("row_ids").get<std::vector<std::uint64_t>>();
  const auto column_ids = value.at("column_ids").get<std::vector<std::uint64_t>>();
  if (row_ids.size() > 256U || column_ids.size() > 256U) {
    throw std::invalid_argument("normalized assignment exceeds its bound");
  }
  const auto & serialized_costs = value.at("costs");
  if (!serialized_costs.is_array() || serialized_costs.size() != row_ids.size()) {
    throw std::invalid_argument("normalized assignment row count is invalid");
  }
  OptionalCostMatrix costs;
  costs.reserve(row_ids.size());
  for (const auto & serialized_row : serialized_costs) {
    if (!serialized_row.is_array() || serialized_row.size() != column_ids.size()) {
      throw std::invalid_argument("normalized assignment column count is invalid");
    }
    std::vector<std::optional<double>> row;
    row.reserve(column_ids.size());
    for (const auto & cost : serialized_row) {
      row.push_back(cost.is_null() ? std::nullopt :
        std::optional<double>{cost.get<double>()});
    }
    costs.push_back(std::move(row));
  }
  const auto result = hungarian_assignment(
    row_ids, column_ids, costs, value.at("unmatched_cost").get<double>());
  Json matches = Json::array();
  for (const auto & match : result.matches) {
    matches.push_back({{"row_id", match.row_id},
      {"column_id", match.column_id}, {"cost", match.cost}});
  }
  return Json{{"matches", std::move(matches)},
    {"unmatched_rows", result.unmatched_rows},
    {"unmatched_columns", result.unmatched_columns},
    {"total_cost", result.total_cost}};
}

Json task_output(const Json & value, const MemoryUpdateResult & memory_result)
{
  const auto & serialized_objects = value.at("objects");
  if (!serialized_objects.is_array() || serialized_objects.size() > 256U) {
    throw std::invalid_argument("normalized task objects exceed their bound");
  }
  SemanticTaskEvidence task{
    {value.at("query_id").get<std::uint64_t>(),
      value.at("query_version").get<std::uint64_t>()},
    appearance_descriptor(value.at("descriptor"))};
  std::vector<ObjectTaskEvidence> objects;
  for (const auto & serialized : serialized_objects) {
    const ProducerObjectKey source{
      serialized.at("source_epoch_id").get<std::uint64_t>(),
      serialized.at("tracklet_id").get<std::int64_t>()};
    const auto found = std::find_if(
      memory_result.objects.begin(), memory_result.objects.end(),
      [&source](const MemoryObject & object) {return object.lidar_key == source;});
    if (found == memory_result.objects.end()) {
      throw std::invalid_argument("task evidence references an unknown source key");
    }
    ObjectTaskEvidence object;
    object.key = found->key;
    object.lifecycle = found->lifecycle;
    const auto & prototypes = serialized.at("prototypes");
    if (!prototypes.is_array() || prototypes.size() > 4U) {
      throw std::invalid_argument("normalized task prototypes exceed their bound");
    }
    for (const auto & prototype : prototypes) {
      object.prototypes.push_back(
        {appearance_descriptor(prototype), 1.0, 1.0, 1U});
    }
    const auto & semantics = serialized.at("permanent_semantics");
    if (!semantics.is_array() || semantics.size() > 16U) {
      throw std::invalid_argument("normalized semantics exceed their bound");
    }
    for (const auto & semantic : semantics) {
      object.permanent_semantics.push_back({
        semantic.at("label").get<std::string>(),
        semantic.at("confidence").get<double>(),
        semantic.at("task_similarity").get<double>(),
        semantic.at("permanent").get<bool>()});
    }
    objects.push_back(std::move(object));
  }
  TaskRelevanceOverlay overlay(TaskRelevanceConfig{});
  (void)overlay.recompute(task, objects);
  struct Ranked
  {
    GlobalObjectKey key;
    double relevance;
  };
  std::vector<Ranked> ranked;
  for (const auto & object : objects) {
    const auto relevance = overlay.relevance(object.key);
    if (relevance.has_value()) {
      ranked.push_back({object.key, *relevance});
    }
  }
  std::sort(ranked.begin(), ranked.end(),
    [](const Ranked & left, const Ranked & right) {
      if (left.relevance != right.relevance) {
        return left.relevance > right.relevance;
      }
      return left.key < right.key;
    });
  Json ranked_json = Json::array();
  for (const auto & item : ranked) {
    ranked_json.push_back({{"memory_epoch_id", item.key.memory_epoch_id},
      {"global_object_id", item.key.global_object_id},
      {"relevance", item.relevance}});
  }
  return Json{{"query_id", task.key.query_id},
    {"query_version", task.key.query_version},
    {"ranked_objects", std::move(ranked_json)}};
}

Json memory_objects(const MemoryUpdateResult & result)
{
  Json output = Json::array();
  for (const auto & object : result.objects) {
    output.push_back({
      {"memory_epoch_id", object.key.memory_epoch_id},
      {"global_object_id", object.key.global_object_id},
      {"lidar_source_epoch_id", object.lidar_key.producer_epoch_id},
      {"lidar_tracklet_id", object.lidar_key.local_object_id},
      {"lifecycle_state", static_cast<std::uint8_t>(object.lifecycle)},
      {"support_state", static_cast<std::uint8_t>(object.support)},
      {"position", object.position},
      {"observation_count", object.observation_count}});
  }
  return output;
}

}  // namespace

std::string normalized_event_name(MemoryEventType type)
{
  switch (type) {
    case MemoryEventType::kCreated: return "object_created";
    case MemoryEventType::kConfirmed: return "object_confirmed";
    case MemoryEventType::kLifecycleChanged: return "lifecycle_changed";
    case MemoryEventType::kLost: return "object_lost";
    case MemoryEventType::kArchived: return "object_archived";
    case MemoryEventType::kDomainChanged: return "domain_changed";
    case MemoryEventType::kMemoryReset: return "memory_reset";
    case MemoryEventType::kObservationRejected: return "observation_rejected";
    case MemoryEventType::kCapacityEvicted: return "capacity_evicted";
    case MemoryEventType::kAssociationAttached: return "association_attached";
    case MemoryEventType::kAssociationDetached: return "association_detached";
    case MemoryEventType::kReidentified: return "object_reidentified";
    case MemoryEventType::kInspectionChanged: return "inspection_state_changed";
  }
  throw std::invalid_argument("normalized replay event type is unsupported");
}

std::string run_normalized_replay(const std::string & serialized_input)
{
  Json input;
  try {
    input = Json::parse(serialized_input);
  } catch (const Json::exception & error) {
    throw std::invalid_argument(
            std::string("normalized replay JSON is invalid: ") + error.what());
  }
  try {
    if (input.at("schema_version") != "1.0.0") {
      throw std::invalid_argument("normalized replay schema version is unsupported");
    }
    const auto epoch = input.at("initial_memory_epoch_id").get<std::uint64_t>();
    const auto & frames = input.at("frames");
    if (epoch == 0U || !frames.is_array() || frames.size() > 10000U) {
      throw std::invalid_argument("normalized replay root contract is invalid");
    }
    MemoryCore memory(MemoryCoreConfig{}, epoch);
    Json output{{"schema_version", "1.0.0"}, {"frames", Json::array()}};
    std::size_t frame_index = 0U;
    for (const auto & frame : frames) {
      const auto & domain_json = frame.at("domain");
      const MemoryDomainKey domain(
        memory_mode(domain_json.at("mode").get<std::string>()),
        domain_json.at("localization_epoch_id").get<std::uint64_t>(),
        domain_json.at("frame_id").get<std::string>());
      const auto & serialized_observations = frame.at("lidar_observations");
      if (!serialized_observations.is_array() ||
        serialized_observations.size() > 256U)
      {
        throw std::invalid_argument("normalized observation batch exceeds its bound");
      }
      std::vector<LidarObservation> observations;
      observations.reserve(serialized_observations.size());
      for (const auto & serialized : serialized_observations) {
        observations.push_back(lidar_observation(serialized));
      }
      const auto result = memory.update(
        domain, frame.at("batch_stamp_ns").get<std::int64_t>(),
        std::move(observations));
      std::vector<std::string> events;
      for (const auto & event : result.events) {
        events.push_back(normalized_event_name(event.type));
      }
      if (frame.contains("expected_event_types") &&
        events != frame.at("expected_event_types").get<std::vector<std::string>>())
      {
        throw std::invalid_argument("normalized replay expected events do not match");
      }
      Json frame_output{{"frame_index", frame_index++},
        {"memory_epoch_id", result.memory_epoch_id},
        {"objects", memory_objects(result)}, {"events", events}};
      if (frame.contains("association")) {
        frame_output["association"] = association_output(frame.at("association"));
      }
      if (frame.contains("task")) {
        frame_output["task"] = task_output(frame.at("task"), result);
      }
      output["frames"].push_back(std::move(frame_output));
    }
    return output.dump();
  } catch (const Json::exception & error) {
    throw std::invalid_argument(
            std::string("normalized replay contract is invalid: ") + error.what());
  }
}

}  // namespace track_robot_semantic_memory
