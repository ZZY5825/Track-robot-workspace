#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <sstream>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "track_robot_semantic_memory/deterministic_replay.hpp"
#include "track_robot_semantic_memory/memory_core.hpp"

namespace semantic_memory = track_robot_semantic_memory;
using Json = nlohmann::json;

namespace
{

Json descriptor(double first, double second)
{
  return Json{{"encoder_id", "openclip"}, {"checkpoint_id", "checkpoint-a"},
    {"version", 1}, {"dimension", 2}, {"l2_normalized", true},
    {"values", {first, second}}};
}

Json observation(std::int64_t tracklet_id, double x)
{
  return Json{{"source_epoch_id", 3}, {"tracklet_id", tracklet_id},
    {"source_stamp_ns", 100}, {"position", {x, 0.0, 0.0}},
    {"velocity", {0.0, 0.0, 0.0}}, {"extent", {1.0, 1.0, 1.0}},
    {"position_covariance", {0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1}},
    {"confidence", 0.9}};
}

Json input(bool permuted = false)
{
  Json row_ids = permuted ? Json{10, 20} : Json{20, 10};
  Json column_ids = permuted ? Json{100, 200} : Json{200, 100};
  Json observations = permuted ?
    Json{observation(10, 1.0), observation(20, 2.0)} :
    Json{observation(20, 2.0), observation(10, 1.0)};
  return Json{{"schema_version", "1.0.0"}, {"initial_memory_epoch_id", 7},
    {"frames", {Json{
      {"domain", {{"mode", "LOCAL_SESSION"},
        {"localization_epoch_id", 11}, {"frame_id", "odom"}}},
      {"batch_stamp_ns", 100}, {"lidar_observations", observations},
      {"association", {{"row_ids", row_ids}, {"column_ids", column_ids},
        {"costs", {{0.1, 0.1}, {0.1, 0.1}}}, {"unmatched_cost", 0.5}}},
      {"expected_event_types", {
        "domain_changed", "object_created", "object_created"}}
    }}}};
}

std::string reidentification_snapshot()
{
  semantic_memory::MemoryCoreConfig config;
  config.max_objects = 8U;
  config.max_history = 4U;
  config.static_lifecycle = {2U, 10, 20, 30};
  config.dynamic_lifecycle = {2U, 5, 10, 20};
  semantic_memory::MemoryCore core(config, 100U);
  const semantic_memory::MemoryDomainKey domain{
    semantic_memory::MemoryMode::kLocalSession, 7U, "odom"};
  auto lidar = [](std::int64_t id, std::int64_t stamp, double x) {
      semantic_memory::LidarObservation value;
      value.source_key = {10U, id};
      value.source_stamp_ns = stamp;
      value.position = {x, 0.0, 0.0};
      value.extent = {1.0, 0.5, 0.5};
      value.position_covariance = {
        0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.2};
      value.confidence = 0.9;
      return value;
    };
  auto visual = [](std::int64_t id, std::int64_t stamp, std::uint64_t observation_id) {
      semantic_memory::VisualMemorySupplement value;
      value.lidar_key = {10U, id};
      value.visual_key = {
        semantic_memory::VisualAssociationKind::kCameraTrack, 20U, 5U};
      value.observation_producer_epoch_id = 20U;
      value.observation_id = observation_id;
      value.visual_candidate_id = observation_id + 100U;
      value.camera_stamp_ns = stamp;
      value.association_confidence = 0.9;
      value.association_confirmed = true;
      value.appearance_evidence_valid = true;
      value.appearance_descriptor = semantic_memory::AppearanceDescriptor{
        "openclip", "checkpoint-a", 1U, 2U, true, {1.0, 0.0}};
      value.appearance_quality = 0.9;
      return value;
    };

  core.update(domain, 1, {lidar(1, 1, 1.0)});
  const auto old = core.update(domain, 2, {lidar(1, 2, 1.0)}).objects.front().key;
  core.supplement_visual(domain, visual(1, 2, 50U));
  core.update(domain, 23, {lidar(2, 23, 1.2)});
  const auto replacement_snapshot = core.update(domain, 24, {lidar(2, 24, 1.2)});
  const auto replacement = replacement_snapshot.objects.back().key;
  const auto moved = visual(2, 24, 51U);
  core.supplement_visual(domain, moved);
  const auto result = core.reidentify(
    domain, old, replacement, moved.lidar_key, moved.visual_key);

  std::ostringstream output;
  for (const auto & object : result.snapshot.objects) {
    output << object.key.memory_epoch_id << ':' << object.key.global_object_id << ':' <<
      object.lidar_key.producer_epoch_id << ':' << object.lidar_key.local_object_id << ':' <<
      object.appearance_summary_id << ';';
  }
  for (const auto & event : result.snapshot.events) {
    output << static_cast<unsigned int>(event.type) << ';';
  }
  return output.str();
}

}  // namespace

TEST(DeterministicReplay, SameNormalizedInputIsByteEquivalent)
{
  const auto serialized = input().dump();

  const auto first = semantic_memory::run_normalized_replay(serialized);
  const auto second = semantic_memory::run_normalized_replay(serialized);

  EXPECT_EQ(first, second);
  const auto parsed = Json::parse(first);
  ASSERT_EQ(parsed["frames"].size(), 1U);
  EXPECT_EQ(parsed["frames"][0]["memory_epoch_id"], 7);
  ASSERT_EQ(parsed["frames"][0]["objects"].size(), 2U);
  EXPECT_EQ(parsed["frames"][0]["objects"][0]["global_object_id"], 1);
  EXPECT_EQ(parsed["frames"][0]["objects"][1]["global_object_id"], 2);
  ASSERT_EQ(parsed["frames"][0]["association"]["matches"].size(), 2U);
}

TEST(DeterministicReplay, EveryMemoryEventHasAStableNormalizedName)
{
  using Event = semantic_memory::MemoryEventType;
  const std::vector<std::pair<Event, std::string>> expected{
    {Event::kCreated, "object_created"},
    {Event::kConfirmed, "object_confirmed"},
    {Event::kLifecycleChanged, "lifecycle_changed"},
    {Event::kLost, "object_lost"},
    {Event::kArchived, "object_archived"},
    {Event::kDomainChanged, "domain_changed"},
    {Event::kMemoryReset, "memory_reset"},
    {Event::kObservationRejected, "observation_rejected"},
    {Event::kCapacityEvicted, "capacity_evicted"},
    {Event::kAssociationAttached, "association_attached"},
    {Event::kAssociationDetached, "association_detached"},
    {Event::kReidentified, "object_reidentified"},
    {Event::kInspectionChanged, "inspection_state_changed"},
  };

  for (const auto & item : expected) {
    EXPECT_EQ(semantic_memory::normalized_event_name(item.first), item.second);
  }
}

TEST(DeterministicReplay, CandidateAndObservationPermutationDoesNotChangeOutput)
{
  EXPECT_EQ(
    semantic_memory::run_normalized_replay(input(false).dump()),
    semantic_memory::run_normalized_replay(input(true).dump()));
}

TEST(DeterministicReplay, ExpectedEventMismatchFailsClosed)
{
  auto invalid = input();
  invalid["frames"][0]["expected_event_types"] = {"object_created"};

  EXPECT_THROW(
    static_cast<void>(semantic_memory::run_normalized_replay(invalid.dump())),
    std::invalid_argument);
}

TEST(DeterministicReplay, CompatibleTaskIsSerializedWithoutChangingObjectKey)
{
  auto with_task = input();
  with_task["frames"][0]["task"] = {
    {"query_id", 9}, {"query_version", 2},
    {"descriptor", descriptor(1.0, 0.0)},
    {"objects", {Json{
      {"source_epoch_id", 3}, {"tracklet_id", 10},
      {"prototypes", {descriptor(1.0, 0.0)}},
      {"permanent_semantics", {Json{
        {"label", "crate"}, {"confidence", 1.0},
        {"task_similarity", 1.0}, {"permanent", true}}}}
    }}}
  };

  const auto result = Json::parse(
    semantic_memory::run_normalized_replay(with_task.dump()));

  EXPECT_EQ(result["frames"][0]["task"]["query_id"], 9);
  ASSERT_EQ(result["frames"][0]["task"]["ranked_objects"].size(), 1U);
  EXPECT_EQ(
    result["frames"][0]["task"]["ranked_objects"][0]["global_object_id"], 1);
  EXPECT_DOUBLE_EQ(
    result["frames"][0]["task"]["ranked_objects"][0]["relevance"], 1.0);
  EXPECT_EQ(result["frames"][0]["objects"][0]["global_object_id"], 1);
}

TEST(DeterministicReplay, ReidentificationTransferIsByteEquivalentAcrossRuns)
{
  EXPECT_EQ(reidentification_snapshot(), reidentification_snapshot());
}
