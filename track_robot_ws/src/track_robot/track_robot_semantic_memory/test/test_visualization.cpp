#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "track_robot_interfaces/msg/semantic_object.hpp"
#include "track_robot_interfaces/msg/semantic_object_array.hpp"
#include "track_robot_semantic_memory/visualization.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace semantic_memory = track_robot_semantic_memory;
namespace interfaces = track_robot_interfaces::msg;

namespace
{

interfaces::SemanticObject object(std::uint64_t epoch, std::uint64_t id)
{
  interfaces::SemanticObject value;
  value.header.frame_id = "odom";
  value.memory_epoch_id = epoch;
  value.global_object_id = id;
  value.position_valid = true;
  value.position.x = static_cast<double>(id);
  value.extent_valid = true;
  value.extent.x = 1.0;
  value.extent.y = 0.5;
  value.extent.z = 0.25;
  value.lifecycle_state = interfaces::SemanticObject::LIFECYCLE_CONFIRMED;
  value.support_state = interfaces::SemanticObject::SUPPORT_LIDAR_ONLY;
  value.motion_class = interfaces::SemanticObject::MOTION_STATIC;
  return value;
}

interfaces::SemanticObjectArray snapshot(
  std::uint64_t epoch, std::uint64_t sequence)
{
  interfaces::SemanticObjectArray value;
  value.header.frame_id = "odom";
  value.memory_epoch_id = epoch;
  value.snapshot_sequence = sequence;
  return value;
}

const visualization_msgs::msg::Marker * find_marker(
  const visualization_msgs::msg::MarkerArray & output,
  const std::string & marker_namespace,
  std::int32_t action = visualization_msgs::msg::Marker::ADD)
{
  for (const auto & marker : output.markers) {
    if (marker.ns == marker_namespace && marker.action == action) {
      return &marker;
    }
  }
  return nullptr;
}

}  // namespace

TEST(Visualization, KeepsStableIdsAndExplicitlyDeletesMissingObjects)
{
  semantic_memory::MarkerRegistry registry(256U);
  auto first = snapshot(10U, 1U);
  first.objects = {object(10U, 1U), object(10U, 2U)};

  const auto first_markers = registry.update(first);
  ASSERT_EQ(first_markers.markers.size(), 4U);
  const int first_id = first_markers.markers[0].id;
  const int second_id = first_markers.markers[2].id;
  EXPECT_NE(first_id, second_id);
  EXPECT_EQ(first_markers.markers[0].action,
    visualization_msgs::msg::Marker::ADD);

  auto second = snapshot(10U, 2U);
  second.objects = {object(10U, 2U)};
  const auto second_markers = registry.update(second);

  ASSERT_EQ(second_markers.markers.size(), 4U);
  EXPECT_EQ(second_markers.markers[0].action,
    visualization_msgs::msg::Marker::ADD);
  EXPECT_EQ(second_markers.markers[0].id, second_id);
  EXPECT_EQ(second_markers.markers[2].action,
    visualization_msgs::msg::Marker::DELETE);
  EXPECT_EQ(second_markers.markers[2].id, first_id);
}

TEST(Visualization, ResolvesPublicKeyHashCollisionsDeterministically)
{
  semantic_memory::MarkerRegistry registry(256U);
  auto input = snapshot(10U, 1U);
  input.objects = {
    object(10U, 1U),
    object(10U, (std::uint64_t{1U} << 31U) + 1U)};

  const auto output = registry.update(input);

  ASSERT_EQ(output.markers.size(), 4U);
  std::set<std::int32_t> ids;
  for (const auto & marker : output.markers) {
    if (marker.ns == "semantic_memory_objects") {
      ids.insert(marker.id);
    }
  }
  EXPECT_EQ(ids.size(), 2U);
}

TEST(Visualization, EncodesGeometryAndStateWithBoundedMarkers)
{
  semantic_memory::MarkerRegistry registry(2U);
  auto input = snapshot(10U, 1U);
  auto dynamic = object(10U, 1U);
  dynamic.motion_class = interfaces::SemanticObject::MOTION_DYNAMIC;
  dynamic.support_state = interfaces::SemanticObject::SUPPORT_PREDICTION_ONLY;
  input.objects = {dynamic};

  const auto output = registry.update(input);

  ASSERT_EQ(output.markers.size(), 2U);
  const auto & marker = output.markers[0];
  EXPECT_EQ(marker.header.frame_id, "odom");
  EXPECT_EQ(marker.ns, "semantic_memory_objects");
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::CUBE);
  EXPECT_DOUBLE_EQ(marker.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(marker.scale.x, 1.0);
  EXPECT_GT(marker.color.b, marker.color.g);
  EXPECT_LT(marker.color.a, 0.65F);

  auto too_many = snapshot(10U, 2U);
  too_many.objects = {object(10U, 1U), object(10U, 2U), object(10U, 3U)};
  EXPECT_THROW((void)registry.update(too_many), std::invalid_argument);
}

TEST(Visualization, RejectsDuplicateWrongEpochAndOutOfOrderSnapshots)
{
  semantic_memory::MarkerRegistry registry(256U);
  auto first = snapshot(10U, 2U);
  first.objects = {object(10U, 1U)};
  EXPECT_NO_THROW((void)registry.update(first));
  EXPECT_THROW((void)registry.update(first), std::invalid_argument);

  auto duplicate = snapshot(10U, 3U);
  duplicate.objects = {object(10U, 1U), object(10U, 1U)};
  EXPECT_THROW((void)registry.update(duplicate), std::invalid_argument);

  auto wrong_epoch = snapshot(10U, 3U);
  wrong_epoch.objects = {object(11U, 1U)};
  EXPECT_THROW((void)registry.update(wrong_epoch), std::invalid_argument);
}

TEST(Visualization, AllowsSequenceResetOnlyWhenMemoryEpochChanges)
{
  semantic_memory::MarkerRegistry registry(256U);
  auto old_epoch = snapshot(10U, 5U);
  old_epoch.objects = {object(10U, 1U)};
  EXPECT_NO_THROW((void)registry.update(old_epoch));

  auto restarted = snapshot(11U, 1U);
  restarted.objects = {object(11U, 1U)};
  const auto output = registry.update(restarted);

  ASSERT_EQ(output.markers.size(), 4U);
  EXPECT_EQ(output.markers[0].action,
    visualization_msgs::msg::Marker::ADD);
  EXPECT_EQ(output.markers[2].action,
    visualization_msgs::msg::Marker::DELETE);
  EXPECT_THROW((void)registry.update(restarted), std::invalid_argument);
}

TEST(Visualization, AddsReadableStateLabelAndCalibratedWinnerHighlight)
{
  semantic_memory::MarkerRegistry registry(256U);
  auto input = snapshot(10U, 1U);
  auto candidate = object(10U, 1U);
  candidate.support_state = interfaces::SemanticObject::SUPPORT_CAMERA_LIDAR;
  candidate.active_query_id = 9U;
  candidate.active_query_version = 2U;
  candidate.task_relevance = 0.75F;
  input.objects = {candidate};
  registry.set_best_candidate(semantic_memory::GlobalObjectKey{10U, 1U});

  const auto output = registry.update(input);

  const auto * cube = find_marker(output, "semantic_memory_objects");
  const auto * label = find_marker(output, "semantic_memory_labels");
  const auto * halo = find_marker(
    output, "semantic_memory_best_candidate");
  const auto * winner_label = find_marker(
    output, "semantic_memory_best_candidate_label");
  ASSERT_NE(cube, nullptr);
  ASSERT_NE(label, nullptr);
  ASSERT_NE(halo, nullptr);
  ASSERT_NE(winner_label, nullptr);
  EXPECT_EQ(label->type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_NE(label->text.find("object 1"), std::string::npos);
  EXPECT_NE(label->text.find("confirmed"), std::string::npos);
  EXPECT_NE(label->text.find("camera+lidar"), std::string::npos);
  EXPECT_NE(label->text.find("relevance=0.750"), std::string::npos);
  EXPECT_EQ(winner_label->text, "BEST CANDIDATE");
  EXPECT_GT(halo->color.r, halo->color.g);
  EXPECT_GT(halo->color.b, halo->color.g);
}

TEST(Visualization, ClearingWinnerAndRemovingObjectDeletesEveryMarker)
{
  semantic_memory::MarkerRegistry registry(256U);
  auto first = snapshot(10U, 1U);
  first.objects = {object(10U, 1U)};
  registry.set_best_candidate(semantic_memory::GlobalObjectKey{10U, 1U});
  EXPECT_NO_THROW((void)registry.update(first));

  auto empty = snapshot(10U, 2U);
  registry.set_best_candidate(std::nullopt);
  const auto output = registry.update(empty);

  EXPECT_NE(find_marker(
    output, "semantic_memory_objects",
    visualization_msgs::msg::Marker::DELETE), nullptr);
  EXPECT_NE(find_marker(
    output, "semantic_memory_labels",
    visualization_msgs::msg::Marker::DELETE), nullptr);
  EXPECT_NE(find_marker(
    output, "semantic_memory_best_candidate",
    visualization_msgs::msg::Marker::DELETE), nullptr);
  EXPECT_NE(find_marker(
    output, "semantic_memory_best_candidate_label",
    visualization_msgs::msg::Marker::DELETE), nullptr);
}
