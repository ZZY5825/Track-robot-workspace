#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "track_robot_interfaces/msg/lidar_tracklet.hpp"
#include "track_robot_semantic_memory/memory_core.hpp"
#include "track_robot_semantic_memory/ros_conversions.hpp"

namespace semantic_memory = track_robot_semantic_memory;
namespace interfaces = track_robot_interfaces::msg;

namespace
{

interfaces::LidarTracklet tracklet(
  std::int32_t id, std::uint32_t stamp_ns, double x)
{
  interfaces::LidarTracklet value;
  value.tracklet_id = id;
  value.position.x = x;
  value.velocity.x = 0.0;
  value.size.x = 1.0;
  value.size.y = 0.5;
  value.size.z = 0.5;
  value.position_covariance_xy = {0.1F, 0.0F, 0.0F, 0.1F};
  value.confidence = 0.9F;
  value.observation_quality = 0.8F;
  value.last_measurement_stamp.nanosec = stamp_ns;
  value.active = true;
  return value;
}

geometry_msgs::msg::TransformStamped identity()
{
  geometry_msgs::msg::TransformStamped value;
  value.header.frame_id = "odom";
  value.child_frame_id = "lidar";
  value.transform.rotation.w = 1.0;
  return value;
}

semantic_memory::MemoryCoreConfig config()
{
  semantic_memory::MemoryCoreConfig value;
  value.max_objects = 16U;
  value.max_history = 4U;
  value.rollback_tolerance_ns = 5;
  value.static_lifecycle = {2U, 100, 200, 500};
  value.dynamic_lifecycle = {2U, 50, 100, 300};
  return value;
}

struct Replay
{
  std::vector<semantic_memory::MemoryUpdateResult> batches;
  std::string encoding;
};

std::string encode(const std::vector<semantic_memory::MemoryUpdateResult> & batches)
{
  std::ostringstream output;
  for (const auto & batch : batches) {
    output << "E" << batch.memory_epoch_id << ":";
    for (const auto & object : batch.objects) {
      if (!object.lidar_key.has_value()) {
        continue;
      }
      output << object.key.global_object_id << ","
             << object.lidar_key->producer_epoch_id << ","
             << object.lidar_key->local_object_id << ","
             << static_cast<int>(object.lifecycle) << ","
             << static_cast<int>(object.support) << ","
             << object.position[0] << ","
             << object.velocity[0] << ";";
    }
    output << "|";
  }
  return output.str();
}

Replay run_replay()
{
  const semantic_memory::MemoryDomainKey local{
    semantic_memory::MemoryMode::kLocalSession, 7U, "odom"};
  const semantic_memory::MemoryDomainKey world{
    semantic_memory::MemoryMode::kWorld, 8U, "map"};
  semantic_memory::MemoryCore core(
    config(), semantic_memory::derive_memory_epoch_seed(local, 10U));
  const auto transform = identity();
  Replay replay;
  replay.batches.push_back(core.update(local, 1, {
    semantic_memory::lidar_observation_from_tracklet(tracklet(1, 1, 1.0), 10U, transform),
    semantic_memory::lidar_observation_from_tracklet(tracklet(2, 1, 2.0), 10U, transform)}));
  replay.batches.push_back(core.update(local, 2, {
    semantic_memory::lidar_observation_from_tracklet(tracklet(1, 2, 1.1), 10U, transform),
    semantic_memory::lidar_observation_from_tracklet(tracklet(2, 2, 2.1), 10U, transform)}));
  replay.batches.push_back(core.update(local, 120, {
    semantic_memory::lidar_observation_from_tracklet(tracklet(2, 120, 2.2), 10U, transform)}));
  replay.batches.push_back(core.update(local, 121, {
    semantic_memory::lidar_observation_from_tracklet(tracklet(2, 121, 2.3), 11U, transform)}));
  replay.batches.push_back(core.update(world, 122, {
    semantic_memory::lidar_observation_from_tracklet(tracklet(2, 122, 3.0), 11U, transform)}));
  replay.encoding = encode(replay.batches);
  return replay;
}

}  // namespace

TEST(Stage2BAcceptance, LidarOnlyMemoryMeetsIdentityLifecycleAndDomainGates)
{
  const auto replay = run_replay();
  ASSERT_EQ(replay.batches.size(), 5U);

  const auto & first = replay.batches[0];
  const auto & confirmed = replay.batches[1];
  ASSERT_EQ(first.objects.size(), 2U);
  ASSERT_EQ(confirmed.objects.size(), 2U);
  EXPECT_NE(first.objects[0].key, first.objects[1].key);
  EXPECT_EQ(first.objects[0].key, confirmed.objects[0].key);
  EXPECT_EQ(first.objects[1].key, confirmed.objects[1].key);
  EXPECT_EQ(confirmed.objects[0].lifecycle,
    semantic_memory::LifecycleState::kConfirmed);
  EXPECT_DOUBLE_EQ(confirmed.objects[0].velocity[0], 0.0);

  const auto & short_loss = replay.batches[2];
  ASSERT_EQ(short_loss.objects.size(), 2U);
  EXPECT_EQ(short_loss.objects[0].lifecycle,
    semantic_memory::LifecycleState::kStale);
  EXPECT_EQ(short_loss.objects[0].support,
    semantic_memory::SupportState::kPredictionOnly);
  EXPECT_EQ(short_loss.active_objects.size(), 2U);

  const auto & source_changed = replay.batches[3];
  ASSERT_EQ(source_changed.objects.size(), 3U);
  ASSERT_TRUE(source_changed.objects.back().lidar_key.has_value());
  EXPECT_EQ(source_changed.objects.back().lidar_key->producer_epoch_id, 11U);
  EXPECT_NE(source_changed.objects.back().key.global_object_id,
    confirmed.objects[1].key.global_object_id);

  const auto & domain_changed = replay.batches[4];
  ASSERT_EQ(domain_changed.objects.size(), 1U);
  EXPECT_NE(domain_changed.memory_epoch_id, source_changed.memory_epoch_id);
  EXPECT_NE(domain_changed.objects[0].key, source_changed.objects.back().key);
}

TEST(Stage2BAcceptance, NormalizedReplayIsExactlyDeterministic)
{
  const auto first = run_replay();
  const auto second = run_replay();

  EXPECT_EQ(first.encoding, second.encoding);
  EXPECT_FALSE(first.encoding.empty());
}
