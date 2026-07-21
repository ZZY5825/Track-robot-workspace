#include "track_robot_semantic_memory/source_time_buffer.hpp"

#include <tuple>

namespace track_robot_semantic_memory
{

bool SourceTimeKey::valid() const noexcept
{
  return source_stamp_ns >= 0 && producer_epoch_id != 0U && producer_local_id >= 0;
}

bool operator==(const SourceTimeKey & left, const SourceTimeKey & right) noexcept
{
  return left.source_stamp_ns == right.source_stamp_ns &&
         left.producer_epoch_id == right.producer_epoch_id &&
         left.producer_local_id == right.producer_local_id;
}

bool operator<(const SourceTimeKey & left, const SourceTimeKey & right) noexcept
{
  return std::tie(
    left.source_stamp_ns, left.producer_epoch_id, left.producer_local_id) <
         std::tie(
    right.source_stamp_ns, right.producer_epoch_id, right.producer_local_id);
}

void SourceTimeBufferLimits::validate() const
{
  if (max_count == 0U) {
    throw std::invalid_argument("source-time max_count must be positive");
  }
  if (max_age_ns < 0 || rollback_tolerance_ns < 0) {
    throw std::invalid_argument("source-time duration bounds must be non-negative");
  }
}

}  // namespace track_robot_semantic_memory
