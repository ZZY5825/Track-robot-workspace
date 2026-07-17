#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "track_robot_semantic_memory/runtime_task_services.hpp"

namespace semantic_memory = track_robot_semantic_memory;

namespace
{

semantic_memory::AppearanceDescriptor descriptor(std::vector<double> values)
{
  return semantic_memory::AppearanceDescriptor{
    "openclip", "checkpoint-a", 1U,
    static_cast<std::uint16_t>(values.size()), true, std::move(values)};
}

semantic_memory::AppearancePrototype prototype(std::vector<double> values)
{
  return {descriptor(std::move(values)), 1.0, 1.0, 1U};
}

semantic_memory::MemoryObject object(
  std::uint64_t epoch, std::uint64_t id,
  semantic_memory::LifecycleState lifecycle =
  semantic_memory::LifecycleState::kConfirmed)
{
  semantic_memory::MemoryObject value;
  value.key = {epoch, id};
  value.lidar_key = {10U, static_cast<std::int64_t>(id)};
  value.lifecycle = lifecycle;
  value.support = semantic_memory::SupportState::kLidarOnly;
  return value;
}

semantic_memory::MemoryUpdateResult snapshot(
  std::uint64_t epoch, std::vector<semantic_memory::MemoryObject> objects)
{
  semantic_memory::MemoryUpdateResult value;
  value.memory_epoch_id = epoch;
  value.objects = std::move(objects);
  for (const auto & item : value.objects) {
    if (item.lifecycle != semantic_memory::LifecycleState::kLost &&
      item.lifecycle != semantic_memory::LifecycleState::kArchived)
    {
      value.active_objects.push_back(item);
    }
  }
  return value;
}

semantic_memory::RuntimeTaskServiceCoordinator coordinator(
  std::uint64_t epoch = 7U, bool calibrated = true)
{
  return semantic_memory::RuntimeTaskServiceCoordinator(
    {0.75, 0.25, 1e-5, 16U}, {calibrated, 0.8}, epoch);
}

}  // namespace

TEST(RuntimeTaskServices, ActiveTaskRanksMaximumCompatiblePrototype)
{
  auto runtime = coordinator();
  runtime.synchronize(
    snapshot(7U, {object(7U, 2U), object(7U, 1U)}),
    {{{7U, 1U}, {prototype({1.0, 0.0}), prototype({0.8, 0.6})}},
      {{7U, 2U}, {prototype({0.0, 1.0})}}});

  ASSERT_TRUE(runtime.accept_task(
      {{41U, 3U}, descriptor({1.0, 0.0})}, "red crate", 90U, 100));
  const auto page = runtime.query_active(
    {41U, 3U}, {64U, 0U, false, false, false, true});

  ASSERT_TRUE(page.accepted);
  ASSERT_EQ(page.objects.size(), 2U);
  EXPECT_EQ(page.objects[0].object.key.global_object_id, 1U);
  EXPECT_DOUBLE_EQ(page.objects[0].task_relevance, 1.0);
  ASSERT_TRUE(page.objects[0].active_task.has_value());
  EXPECT_EQ(page.objects[0].active_task->query_id, 41U);
  const auto winner = runtime.best_candidate();
  ASSERT_TRUE(winner.object.has_value());
  EXPECT_EQ(winner.object->object.key.global_object_id, 1U);
}

TEST(RuntimeTaskServices, PermanentSemanticEvidenceRequiresExactNormalizedText)
{
  auto matching = object(7U, 1U);
  matching.semantic_labels.push_back({"red crate", 0.8, "detector", 2U, 5U});
  auto unrelated = object(7U, 2U);
  unrelated.semantic_labels.push_back({"crate", 1.0, "detector", 2U, 6U});
  auto runtime = coordinator();
  runtime.synchronize(snapshot(7U, {matching, unrelated}), {});

  ASSERT_TRUE(runtime.accept_task(
      {{4U, 1U}, descriptor({1.0, 0.0})}, "  RED   crate ", 90U, 100));
  const auto page = runtime.query_active(
    {4U, 1U}, {64U, 0U, false, false, false, true});

  ASSERT_EQ(page.objects.size(), 1U);
  EXPECT_EQ(page.objects[0].object.key.global_object_id, 1U);
  EXPECT_NEAR(page.objects[0].task_relevance, 0.8, 1e-9);
}

TEST(RuntimeTaskServices, DescriptorQueryDoesNotReplaceActiveTask)
{
  auto runtime = coordinator();
  runtime.synchronize(
    snapshot(7U, {object(7U, 1U), object(7U, 2U)}),
    {{{7U, 1U}, {prototype({1.0, 0.0})}},
      {{7U, 2U}, {prototype({0.0, 1.0})}}});
  ASSERT_TRUE(runtime.accept_task(
      {{41U, 1U}, descriptor({1.0, 0.0})}, "first", 90U, 100));

  const auto temporary = runtime.query_descriptor(
    {{99U, 2U}, descriptor({0.0, 1.0})}, "second",
    {1U, 0U, false, false, false, true});

  ASSERT_TRUE(temporary.accepted);
  ASSERT_EQ(temporary.objects.size(), 1U);
  EXPECT_EQ(temporary.objects[0].object.key.global_object_id, 2U);
  ASSERT_TRUE(runtime.active_task().has_value());
  EXPECT_EQ(runtime.active_task()->query_id, 41U);
  const auto active = runtime.query_active(
    {41U, 1U}, {1U, 0U, false, false, false, true});
  ASSERT_EQ(active.objects.size(), 1U);
  EXPECT_EQ(active.objects[0].object.key.global_object_id, 1U);
}

TEST(RuntimeTaskServices, InspectionIsIdempotentAndChangesBestCandidate)
{
  auto runtime = coordinator();
  runtime.synchronize(
    snapshot(7U, {object(7U, 1U)}),
    {{{7U, 1U}, {prototype({1.0, 0.0})}}});
  ASSERT_TRUE(runtime.accept_task(
      {{41U, 1U}, descriptor({1.0, 0.0})}, "target", 90U, 100));
  ASSERT_TRUE(runtime.best_candidate().object.has_value());

  const auto changed = runtime.mark_inspected(
    {7U, 1U}, semantic_memory::InspectionState::kComplete);
  ASSERT_TRUE(changed.updated);
  EXPECT_EQ(runtime.service_events().size(), 1U);
  EXPECT_FALSE(runtime.best_candidate().object.has_value());
  const auto unchanged = runtime.mark_inspected(
    {7U, 1U}, semantic_memory::InspectionState::kComplete);
  EXPECT_TRUE(unchanged.updated);
  EXPECT_EQ(runtime.service_events().size(), 1U);

  runtime.synchronize(
    snapshot(7U, {object(7U, 1U), object(7U, 2U)}),
    {{{7U, 1U}, {prototype({1.0, 0.0})}},
      {{7U, 2U}, {prototype({0.9, 0.435889894})}}});
  const auto preserved = runtime.get({7U, 1U});
  ASSERT_TRUE(preserved.object.has_value());
  EXPECT_EQ(
    preserved.object->inspection,
    semantic_memory::InspectionState::kComplete);
}

TEST(RuntimeTaskServices, ResetAndInvalidSynchronizationAreTransactional)
{
  auto runtime = coordinator();
  runtime.synchronize(
    snapshot(7U, {object(7U, 1U)}),
    {{{7U, 1U}, {prototype({1.0, 0.0})}}});
  ASSERT_TRUE(runtime.accept_task(
      {{41U, 1U}, descriptor({1.0, 0.0})}, "target", 90U, 100));

  auto duplicate = snapshot(7U, {object(7U, 2U), object(7U, 2U)});
  EXPECT_THROW(runtime.synchronize(duplicate, {}), std::invalid_argument);
  EXPECT_EQ(runtime.current_epoch(), 7U);
  ASSERT_TRUE(runtime.get({7U, 1U}).object.has_value());

  runtime.reset_to_epoch(8U, "operator reset");
  EXPECT_EQ(runtime.current_epoch(), 8U);
  EXPECT_EQ(
    runtime.get({7U, 1U}).reason,
    semantic_memory::ServiceReason::kStaleEpoch);
  ASSERT_TRUE(runtime.active_task().has_value());
  EXPECT_EQ(runtime.active_task()->query_id, 41U);
}

TEST(RuntimeTaskServices, ProducerChangeAndRollbackReplaceTaskFailClosed)
{
  auto runtime = coordinator();
  runtime.synchronize(
    snapshot(7U, {object(7U, 1U)}),
    {{{7U, 1U}, {prototype({1.0, 0.0})}}});
  ASSERT_TRUE(runtime.accept_task(
      {{41U, 1U}, descriptor({1.0, 0.0})}, "first", 90U, 100));

  EXPECT_FALSE(runtime.accept_task(
      {{41U, 2U}, descriptor({1.0, 0.0})}, "rollback", 90U, 99));
  EXPECT_FALSE(runtime.active_task().has_value());

  ASSERT_TRUE(runtime.accept_task(
      {{42U, 1U}, descriptor({1.0, 0.0})}, "new producer", 91U, 101));
  ASSERT_TRUE(runtime.active_task().has_value());
  EXPECT_EQ(runtime.active_task()->query_id, 42U);
}

TEST(RuntimeTaskServices, QueryTextIsRawBoundedAsciiAndFailsClosed)
{
  auto unicode_object = object(7U, 1U);
  unicode_object.semantic_labels.push_back({"\xc3\xa9", 1.0, "operator", 3U, 5U});
  auto runtime = coordinator();
  runtime.synchronize(snapshot(7U, {unicode_object}), {});

  EXPECT_FALSE(runtime.accept_task(
      {{41U, 1U}, descriptor({1.0, 0.0})}, "\xc3\xa9", 90U, 100));
  EXPECT_FALSE(runtime.active_task().has_value());
  EXPECT_FALSE(runtime.accept_task(
      {{41U, 2U}, descriptor({1.0, 0.0})},
      std::string(513U, ' ') + "x", 90U, 101));
  EXPECT_FALSE(runtime.active_task().has_value());

  const auto query = runtime.query_descriptor(
    {{42U, 1U}, descriptor({1.0, 0.0})}, "\x01target",
    {64U, 0U, false, false, false, true});
  EXPECT_FALSE(query.accepted);
  EXPECT_EQ(query.reason, semantic_memory::ServiceReason::kInvalidRequest);
}

TEST(RuntimeTaskServices, BestCandidateStaysDisabledWithoutCalibration)
{
  auto runtime = coordinator(7U, false);
  runtime.synchronize(
    snapshot(7U, {object(7U, 1U)}),
    {{{7U, 1U}, {prototype({1.0, 0.0})}}});
  ASSERT_TRUE(runtime.accept_task(
      {{41U, 1U}, descriptor({1.0, 0.0})}, "target", 90U, 100));

  const auto result = runtime.best_candidate();
  EXPECT_EQ(
    result.reason,
    semantic_memory::ServiceReason::kThresholdNotCalibrated);
  EXPECT_FALSE(result.object.has_value());
}
