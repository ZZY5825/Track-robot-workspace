#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "track_robot_semantic_memory/memory_clock.hpp"

namespace track_robot_semantic_memory
{

struct SourceTimeKey
{
  std::int64_t source_stamp_ns{0};
  std::uint64_t producer_epoch_id{0U};
  std::int64_t producer_local_id{-1};

  [[nodiscard]] bool valid() const noexcept;
  friend bool operator==(const SourceTimeKey & left, const SourceTimeKey & right) noexcept;
  friend bool operator<(const SourceTimeKey & left, const SourceTimeKey & right) noexcept;
};

struct SourceTimeBufferLimits
{
  std::size_t max_count{1U};
  std::int64_t max_age_ns{0};
  std::int64_t rollback_tolerance_ns{0};

  void validate() const;
};

struct SourceTimeBufferStats
{
  std::uint64_t count_evictions{0U};
  std::uint64_t age_evictions{0U};
  std::uint64_t rollback_count{0U};
  std::uint64_t rollback_drops{0U};
  std::uint64_t duplicate_replacements{0U};
};

enum class BufferPushResult
{
  kInserted,
  kInsertedOutOfOrder,
  kInsertedAfterRollback,
  kReplacedDuplicate,
};

template<typename ValueT>
struct SourceTimeEntry
{
  SourceTimeKey key;
  ValueT value;
};

template<typename ValueT>
class SourceTimeBuffer
{
public:
  explicit SourceTimeBuffer(SourceTimeBufferLimits limits)
  : limits_(limits), clock_(limits.rollback_tolerance_ns)
  {
    limits_.validate();
  }

  BufferPushResult push(SourceTimeKey key, ValueT value)
  {
    if (!key.valid()) {
      throw std::invalid_argument("source-time key is invalid");
    }
    const auto observation = clock_.observe(key.source_stamp_ns);
    if (observation == ClockObservation::kRollback) {
      stats_.rollback_drops += entries_.size();
      entries_.clear();
      ++stats_.rollback_count;
    }

    const auto duplicate = std::find_if(
      entries_.begin(), entries_.end(),
      [&key](const auto & entry) {return entry.key == key;});
    if (duplicate != entries_.end()) {
      duplicate->value = std::move(value);
      ++stats_.duplicate_replacements;
      return BufferPushResult::kReplacedDuplicate;
    }

    entries_.push_back({key, std::move(value)});
    std::sort(
      entries_.begin(), entries_.end(),
      [](const auto & left, const auto & right) {return left.key < right.key;});
    evict_expired();
    while (entries_.size() > limits_.max_count) {
      entries_.erase(entries_.begin());
      ++stats_.count_evictions;
    }
    if (observation == ClockObservation::kRollback) {
      return BufferPushResult::kInsertedAfterRollback;
    }
    if (observation == ClockObservation::kOutOfOrder) {
      return BufferPushResult::kInsertedOutOfOrder;
    }
    return BufferPushResult::kInserted;
  }

  [[nodiscard]] const std::vector<SourceTimeEntry<ValueT>> & entries() const noexcept
  {
    return entries_;
  }

  [[nodiscard]] const SourceTimeEntry<ValueT> * nearest(
    const std::int64_t source_stamp_ns,
    const std::int64_t max_delta_ns) const
  {
    if (source_stamp_ns < 0 || max_delta_ns < 0) {
      throw std::invalid_argument("nearest source-time query parameters must be non-negative");
    }

    const SourceTimeEntry<ValueT> * nearest_entry = nullptr;
    std::int64_t nearest_delta_ns = max_delta_ns;
    for (const auto & entry : entries_) {
      const auto delta_ns = source_stamp_ns >= entry.key.source_stamp_ns ?
        source_stamp_ns - entry.key.source_stamp_ns :
        entry.key.source_stamp_ns - source_stamp_ns;
      if (delta_ns <= max_delta_ns &&
        (nearest_entry == nullptr || delta_ns < nearest_delta_ns))
      {
        nearest_entry = &entry;
        nearest_delta_ns = delta_ns;
      }
    }
    return nearest_entry;
  }

  [[nodiscard]] std::optional<std::int64_t> nearest_delta_ns(
    const std::int64_t source_stamp_ns) const
  {
    if (source_stamp_ns < 0) {
      throw std::invalid_argument(
              "nearest source-time query parameters must be non-negative");
    }
    return nearest_delta_ns_if(
      source_stamp_ns, [](const auto &) {return true;});
  }

  [[nodiscard]] std::optional<std::int64_t> nearest_delta_ns(
    const std::int64_t source_stamp_ns,
    const std::uint64_t producer_epoch_id) const
  {
    if (source_stamp_ns < 0 || producer_epoch_id == 0U) {
      throw std::invalid_argument(
              "epoch-restricted source-time query parameters are invalid");
    }
    return nearest_delta_ns_if(
      source_stamp_ns,
      [producer_epoch_id](const auto & entry) {
        return entry.key.producer_epoch_id == producer_epoch_id;
      });
  }

  [[nodiscard]] const SourceTimeEntry<ValueT> * nearest(
    const std::int64_t source_stamp_ns,
    const std::int64_t max_delta_ns,
    const std::uint64_t producer_epoch_id) const
  {
    if (source_stamp_ns < 0 || max_delta_ns < 0 || producer_epoch_id == 0U) {
      throw std::invalid_argument(
              "epoch-restricted source-time query parameters are invalid");
    }

    const SourceTimeEntry<ValueT> * nearest_entry = nullptr;
    std::int64_t nearest_delta_ns = max_delta_ns;
    for (const auto & entry : entries_) {
      if (entry.key.producer_epoch_id != producer_epoch_id) {
        continue;
      }
      const auto delta_ns = source_stamp_ns >= entry.key.source_stamp_ns ?
        source_stamp_ns - entry.key.source_stamp_ns :
        entry.key.source_stamp_ns - source_stamp_ns;
      if (delta_ns <= max_delta_ns &&
        (nearest_entry == nullptr || delta_ns < nearest_delta_ns))
      {
        nearest_entry = &entry;
        nearest_delta_ns = delta_ns;
      }
    }
    return nearest_entry;
  }

  [[nodiscard]] const SourceTimeBufferStats & stats() const noexcept
  {
    return stats_;
  }

  void clear() noexcept {entries_.clear();}

private:
  template<typename PredicateT>
  [[nodiscard]] std::optional<std::int64_t> nearest_delta_ns_if(
    const std::int64_t source_stamp_ns,
    PredicateT predicate) const
  {
    std::optional<std::int64_t> nearest_delta;
    for (const auto & entry : entries_) {
      if (!predicate(entry)) {
        continue;
      }
      const auto delta_ns = source_stamp_ns >= entry.key.source_stamp_ns ?
        source_stamp_ns - entry.key.source_stamp_ns :
        entry.key.source_stamp_ns - source_stamp_ns;
      if (!nearest_delta.has_value() || delta_ns < *nearest_delta) {
        nearest_delta = delta_ns;
      }
    }
    return nearest_delta;
  }

  void evict_expired()
  {
    const auto latest = clock_.latest_stamp_ns();
    if (!latest.has_value()) {
      return;
    }
    while (!entries_.empty() &&
      *latest - entries_.front().key.source_stamp_ns > limits_.max_age_ns)
    {
      entries_.erase(entries_.begin());
      ++stats_.age_evictions;
    }
  }

  SourceTimeBufferLimits limits_;
  MemoryClock clock_;
  SourceTimeBufferStats stats_;
  std::vector<SourceTimeEntry<ValueT>> entries_;
};

}  // namespace track_robot_semantic_memory
