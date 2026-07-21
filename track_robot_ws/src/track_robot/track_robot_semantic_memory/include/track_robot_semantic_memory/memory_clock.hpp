#pragma once

#include <cstdint>
#include <optional>

namespace track_robot_semantic_memory
{

enum class ClockObservation
{
  kFirst,
  kForward,
  kSame,
  kOutOfOrder,
  kRollback,
};

class MemoryClock
{
public:
  explicit MemoryClock(std::int64_t rollback_tolerance_ns);

  ClockObservation observe(std::int64_t source_stamp_ns);
  void reset(std::int64_t source_stamp_ns);
  [[nodiscard]] std::optional<std::int64_t> latest_stamp_ns() const noexcept;
  [[nodiscard]] std::uint64_t rollback_count() const noexcept;

private:
  std::int64_t rollback_tolerance_ns_;
  std::optional<std::int64_t> latest_stamp_ns_;
  std::uint64_t rollback_count_{0U};
};

}  // namespace track_robot_semantic_memory
