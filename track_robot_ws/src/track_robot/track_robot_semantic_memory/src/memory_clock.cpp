#include "track_robot_semantic_memory/memory_clock.hpp"

#include <stdexcept>

namespace track_robot_semantic_memory
{

MemoryClock::MemoryClock(std::int64_t rollback_tolerance_ns)
: rollback_tolerance_ns_(rollback_tolerance_ns)
{
  if (rollback_tolerance_ns < 0) {
    throw std::invalid_argument("rollback tolerance must be non-negative");
  }
}

ClockObservation MemoryClock::observe(std::int64_t source_stamp_ns)
{
  if (source_stamp_ns < 0) {
    throw std::invalid_argument("source stamp must be non-negative");
  }
  if (!latest_stamp_ns_.has_value()) {
    latest_stamp_ns_ = source_stamp_ns;
    return ClockObservation::kFirst;
  }
  if (source_stamp_ns < *latest_stamp_ns_ - rollback_tolerance_ns_) {
    latest_stamp_ns_ = source_stamp_ns;
    ++rollback_count_;
    return ClockObservation::kRollback;
  }
  if (source_stamp_ns < *latest_stamp_ns_) {
    return ClockObservation::kOutOfOrder;
  }
  if (source_stamp_ns == *latest_stamp_ns_) {
    return ClockObservation::kSame;
  }
  latest_stamp_ns_ = source_stamp_ns;
  return ClockObservation::kForward;
}

void MemoryClock::reset(std::int64_t source_stamp_ns)
{
  if (source_stamp_ns < 0) {
    throw std::invalid_argument("source stamp must be non-negative");
  }
  latest_stamp_ns_ = source_stamp_ns;
}

std::optional<std::int64_t> MemoryClock::latest_stamp_ns() const noexcept
{
  return latest_stamp_ns_;
}

std::uint64_t MemoryClock::rollback_count() const noexcept
{
  return rollback_count_;
}

}  // namespace track_robot_semantic_memory
