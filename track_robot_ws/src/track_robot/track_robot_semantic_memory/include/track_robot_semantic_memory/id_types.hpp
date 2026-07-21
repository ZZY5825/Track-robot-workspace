#pragma once

#include <cstdint>
#include <tuple>

namespace track_robot_semantic_memory
{

struct GlobalObjectKey
{
  std::uint64_t memory_epoch_id{0U};
  std::uint64_t global_object_id{0U};

  [[nodiscard]] constexpr bool valid() const noexcept
  {
    return memory_epoch_id != 0U && global_object_id != 0U;
  }

  friend constexpr bool operator==(
    const GlobalObjectKey & left, const GlobalObjectKey & right) noexcept
  {
    return left.memory_epoch_id == right.memory_epoch_id &&
           left.global_object_id == right.global_object_id;
  }

  friend constexpr bool operator!=(
    const GlobalObjectKey & left, const GlobalObjectKey & right) noexcept
  {
    return !(left == right);
  }

  friend constexpr bool operator<(
    const GlobalObjectKey & left, const GlobalObjectKey & right) noexcept
  {
    return std::tie(left.memory_epoch_id, left.global_object_id) <
           std::tie(right.memory_epoch_id, right.global_object_id);
  }
};

struct ProducerObjectKey
{
  std::uint64_t producer_epoch_id{0U};
  std::int64_t local_object_id{-1};

  [[nodiscard]] constexpr bool valid() const noexcept
  {
    return producer_epoch_id != 0U && local_object_id >= 0;
  }

  friend constexpr bool operator==(
    const ProducerObjectKey & left, const ProducerObjectKey & right) noexcept
  {
    return left.producer_epoch_id == right.producer_epoch_id &&
           left.local_object_id == right.local_object_id;
  }

  friend constexpr bool operator!=(
    const ProducerObjectKey & left, const ProducerObjectKey & right) noexcept
  {
    return !(left == right);
  }

  friend constexpr bool operator<(
    const ProducerObjectKey & left, const ProducerObjectKey & right) noexcept
  {
    return std::tie(left.producer_epoch_id, left.local_object_id) <
           std::tie(right.producer_epoch_id, right.local_object_id);
  }
};

}  // namespace track_robot_semantic_memory
