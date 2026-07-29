#include "track_robot_semantic_memory/memory_services.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace track_robot_semantic_memory
{
namespace
{

constexpr std::size_t kMaximumServiceEvents = 64U;

bool inspection_valid(InspectionState state) noexcept
{
  return state == InspectionState::kNotInspected ||
         state == InspectionState::kRequested ||
         state == InspectionState::kComplete;
}

bool lifecycle_included(
  LifecycleState state, const QueryMemoryRequest & request) noexcept
{
  switch (state) {
    case LifecycleState::kTentative:
    case LifecycleState::kConfirmed:
      return true;
    case LifecycleState::kStale:
      return request.include_stale;
    case LifecycleState::kLost:
      return request.include_lost;
    case LifecycleState::kArchived:
      return request.include_archived;
  }
  return false;
}

}  // namespace

MemoryServiceStore::MemoryServiceStore(std::uint64_t initial_epoch)
: current_epoch_(initial_epoch)
{
  if (initial_epoch == 0U) {
    throw std::invalid_argument("memory service epoch must be non-zero");
  }
}

void MemoryServiceStore::upsert(const MemoryServiceRecord & record)
{
  if (!record.key.valid() || record.key.memory_epoch_id != current_epoch_ ||
    !inspection_valid(record.inspection) || !std::isfinite(record.task_relevance) ||
    record.task_relevance < 0.0 || record.task_relevance > 1.0)
  {
    throw std::invalid_argument("memory service record violates bounded contract");
  }
  if (records_.find(record.key) == records_.end() && records_.size() >= 256U) {
    throw std::invalid_argument("memory service store reached its object bound");
  }
  records_[record.key] = record;
}

void MemoryServiceStore::synchronize(
  std::uint64_t memory_epoch_id,
  const std::vector<MemoryServiceRecord> & records)
{
  if (memory_epoch_id == 0U || records.size() > 256U) {
    throw std::invalid_argument(
            "memory service synchronization violates its bound");
  }
  std::map<GlobalObjectKey, MemoryServiceRecord> next;
  for (auto record : records) {
    if (!record.key.valid() ||
      record.key.memory_epoch_id != memory_epoch_id ||
      !inspection_valid(record.inspection) ||
      !std::isfinite(record.task_relevance) ||
      record.task_relevance < 0.0 || record.task_relevance > 1.0)
    {
      throw std::invalid_argument(
              "memory service synchronization record is invalid");
    }
    const auto previous = records_.find(record.key);
    if (memory_epoch_id == current_epoch_ && previous != records_.end()) {
      record.inspection = previous->second.inspection;
    }
    if (!next.emplace(record.key, std::move(record)).second) {
      throw std::invalid_argument(
              "memory service synchronization contains duplicate keys");
    }
  }
  current_epoch_ = memory_epoch_id;
  records_ = std::move(next);
}

GetMemoryResult MemoryServiceStore::get(const GlobalObjectKey & key) const
{
  if (!key.valid()) {
    return {ServiceReason::kInvalidRequest, std::nullopt};
  }
  if (key.memory_epoch_id != current_epoch_) {
    return {ServiceReason::kStaleEpoch, std::nullopt};
  }
  const auto found = records_.find(key);
  if (found == records_.end()) {
    return {ServiceReason::kNotFound, std::nullopt};
  }
  return {ServiceReason::kOk, found->second};
}

QueryMemoryResult MemoryServiceStore::query(const QueryMemoryRequest & request) const
{
  if (request.page_size == 0U || request.page_size > 64U) {
    return {false, ServiceReason::kInvalidRequest, {}, 0U, false};
  }
  std::vector<MemoryServiceRecord> eligible;
  eligible.reserve(records_.size());
  for (const auto & item : records_) {
    const auto & record = item.second;
    if (!lifecycle_included(record.lifecycle, request)) {
      continue;
    }
    if (!request.include_inspected &&
      record.inspection != InspectionState::kNotInspected)
    {
      continue;
    }
    eligible.push_back(record);
  }
  std::sort(
    eligible.begin(), eligible.end(),
    [](const MemoryServiceRecord & left, const MemoryServiceRecord & right) {
      if (left.task_relevance != right.task_relevance) {
        return left.task_relevance > right.task_relevance;
      }
      return left.key < right.key;
    });

  if (request.page_token != 0U && request.page_token >= eligible.size()) {
    return {false, ServiceReason::kInvalidRequest, {}, 0U, false};
  }

  const std::size_t begin = request.page_token >= eligible.size() ?
    eligible.size() : static_cast<std::size_t>(request.page_token);
  const std::size_t end = std::min(eligible.size(), begin + request.page_size);
  std::vector<MemoryServiceRecord> page(
    eligible.begin() + static_cast<std::ptrdiff_t>(begin),
    eligible.begin() + static_cast<std::ptrdiff_t>(end));
  const bool has_more = end < eligible.size();
  return {true, ServiceReason::kOk, std::move(page),
    has_more ? static_cast<std::uint64_t>(end) : 0U, has_more};
}

InspectionResult MemoryServiceStore::mark_inspected(
  const GlobalObjectKey & key, InspectionState state)
{
  if (!inspection_valid(state)) {
    return {false, ServiceReason::kInvalidRequest, std::nullopt};
  }
  const auto lookup = get(key);
  if (!lookup.record.has_value()) {
    return {false, lookup.reason, std::nullopt};
  }
  auto & record = records_.at(key);
  if (record.inspection != state) {
    record.inspection = state;
    append_event(
      MemoryServiceEventType::kInspectionChanged, key,
      "inspection state changed");
  }
  return {true, ServiceReason::kOk, record};
}

ResetMemoryResult MemoryServiceStore::reset(
  std::uint64_t expected_epoch, bool require_epoch_match,
  std::string reason)
{
  if (require_epoch_match && expected_epoch != current_epoch_) {
    return {false, current_epoch_, ServiceReason::kEpochMismatch};
  }
  current_epoch_ = next_epoch(current_epoch_);
  records_.clear();
  append_event(
    MemoryServiceEventType::kMemoryReset, {current_epoch_, 0U},
    std::move(reason));
  return {true, current_epoch_, ServiceReason::kOk};
}

ResetMemoryResult MemoryServiceStore::reset_to_epoch(
  std::uint64_t new_epoch, std::string reason)
{
  if (new_epoch == 0U) {
    return {false, current_epoch_, ServiceReason::kInvalidRequest};
  }
  current_epoch_ = new_epoch;
  records_.clear();
  append_event(
    MemoryServiceEventType::kMemoryReset, {current_epoch_, 0U},
    std::move(reason));
  return {true, current_epoch_, ServiceReason::kOk};
}

BestCandidateResult MemoryServiceStore::best_candidate(
  const BestCandidateConfig & config) const
{
  if (!config.threshold_calibrated) {
    return {ServiceReason::kThresholdNotCalibrated, std::nullopt};
  }
  if (!std::isfinite(config.minimum_relevance) ||
    config.minimum_relevance < 0.0 || config.minimum_relevance > 1.0)
  {
    return {ServiceReason::kInvalidRequest, std::nullopt};
  }
  std::optional<MemoryServiceRecord> best;
  for (const auto & item : records_) {
    const auto & candidate = item.second;
    if (candidate.lifecycle != LifecycleState::kConfirmed ||
      candidate.inspection != InspectionState::kNotInspected)
    {
      continue;
    }
    if (!best.has_value() || candidate.task_relevance > best->task_relevance ||
      (candidate.task_relevance == best->task_relevance && candidate.key < best->key))
    {
      best = candidate;
    }
  }
  if (!best.has_value()) {
    return {ServiceReason::kNoEligibleCandidate, std::nullopt};
  }
  if (best->task_relevance < config.minimum_relevance) {
    return {ServiceReason::kBelowThreshold, std::nullopt};
  }
  return {ServiceReason::kOk, best};
}

std::uint64_t MemoryServiceStore::current_epoch() const noexcept
{
  return current_epoch_;
}

std::size_t MemoryServiceStore::size() const noexcept
{
  return records_.size();
}

const std::vector<MemoryServiceEvent> & MemoryServiceStore::events() const noexcept
{
  return events_;
}

std::uint64_t MemoryServiceStore::next_epoch(std::uint64_t current) noexcept
{
  return current == std::numeric_limits<std::uint64_t>::max() ? 1U : current + 1U;
}

void MemoryServiceStore::append_event(
  MemoryServiceEventType type, GlobalObjectKey key, std::string reason)
{
  if (reason.size() > 256U) {
    reason.resize(256U);
  }
  events_.push_back(MemoryServiceEvent{
    next_event_sequence_, type, key, std::move(reason)});
  if (events_.size() > kMaximumServiceEvents) {
    events_.erase(events_.begin());
  }
  if (next_event_sequence_ != std::numeric_limits<std::uint64_t>::max()) {
    ++next_event_sequence_;
  }
}

}  // namespace track_robot_semantic_memory
