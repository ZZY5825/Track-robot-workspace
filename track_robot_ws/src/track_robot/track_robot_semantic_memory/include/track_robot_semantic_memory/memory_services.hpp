#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "track_robot_semantic_memory/id_types.hpp"
#include "track_robot_semantic_memory/types.hpp"

namespace track_robot_semantic_memory
{

enum class InspectionState : std::uint8_t
{
  kNotInspected = 0U,
  kRequested = 1U,
  kComplete = 2U,
};

enum class ServiceReason : std::uint8_t
{
  kOk = 0U,
  kInvalidRequest = 1U,
  kStaleEpoch = 2U,
  kNotFound = 3U,
  kEpochMismatch = 4U,
  kThresholdNotCalibrated = 5U,
  kNoEligibleCandidate = 6U,
  kBelowThreshold = 7U,
};

struct MemoryServiceRecord
{
  GlobalObjectKey key;
  LifecycleState lifecycle{LifecycleState::kTentative};
  InspectionState inspection{InspectionState::kNotInspected};
  double task_relevance{0.0};
};

struct GetMemoryResult
{
  ServiceReason reason{ServiceReason::kNotFound};
  std::optional<MemoryServiceRecord> record;
};

struct QueryMemoryRequest
{
  std::size_t page_size{64U};
  std::uint64_t page_token{0U};
  bool include_stale{false};
  bool include_lost{false};
  bool include_archived{false};
  bool include_inspected{false};
};

struct QueryMemoryResult
{
  bool accepted{false};
  ServiceReason reason{ServiceReason::kInvalidRequest};
  std::vector<MemoryServiceRecord> records;
  std::uint64_t next_page_token{0U};
  bool has_more{false};
};

struct InspectionResult
{
  bool updated{false};
  ServiceReason reason{ServiceReason::kNotFound};
  std::optional<MemoryServiceRecord> record;
};

struct ResetMemoryResult
{
  bool reset{false};
  std::uint64_t new_epoch{0U};
  ServiceReason reason{ServiceReason::kInvalidRequest};
};

struct BestCandidateConfig
{
  bool threshold_calibrated{false};
  double minimum_relevance{1.0};
};

struct BestCandidateResult
{
  ServiceReason reason{ServiceReason::kNoEligibleCandidate};
  std::optional<MemoryServiceRecord> record;
};

enum class MemoryServiceEventType : std::uint8_t
{
  kInspectionChanged = 0U,
  kMemoryReset = 1U,
};

struct MemoryServiceEvent
{
  std::uint64_t sequence{0U};
  MemoryServiceEventType type{MemoryServiceEventType::kInspectionChanged};
  GlobalObjectKey key;
  std::string reason;
};

class MemoryServiceStore
{
public:
  explicit MemoryServiceStore(std::uint64_t initial_epoch);

  void upsert(const MemoryServiceRecord & record);
  void synchronize(
    std::uint64_t memory_epoch_id,
    const std::vector<MemoryServiceRecord> & records);
  [[nodiscard]] GetMemoryResult get(const GlobalObjectKey & key) const;
  [[nodiscard]] QueryMemoryResult query(const QueryMemoryRequest & request) const;
  InspectionResult mark_inspected(
    const GlobalObjectKey & key, InspectionState state);
  ResetMemoryResult reset(
    std::uint64_t expected_epoch, bool require_epoch_match,
    std::string reason);
  [[nodiscard]] BestCandidateResult best_candidate(
    const BestCandidateConfig & config) const;

  [[nodiscard]] std::uint64_t current_epoch() const noexcept;
  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] const std::vector<MemoryServiceEvent> & events() const noexcept;

private:
  [[nodiscard]] static std::uint64_t next_epoch(std::uint64_t current) noexcept;
  void append_event(
    MemoryServiceEventType type, GlobalObjectKey key, std::string reason);

  std::uint64_t current_epoch_;
  std::uint64_t next_event_sequence_{1U};
  std::map<GlobalObjectKey, MemoryServiceRecord> records_;
  std::vector<MemoryServiceEvent> events_;
};

}  // namespace track_robot_semantic_memory
