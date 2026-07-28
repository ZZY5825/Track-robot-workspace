#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "track_robot_semantic_memory/memory_core.hpp"
#include "track_robot_semantic_memory/memory_services.hpp"
#include "track_robot_semantic_memory/task_relevance_scorer.hpp"

namespace track_robot_semantic_memory
{

struct RuntimeObjectView
{
  MemoryObject object;
  InspectionState inspection{InspectionState::kNotInspected};
  std::optional<SemanticTaskKey> active_task;
  double task_relevance{0.0};
};

struct GetRuntimeObjectResult
{
  ServiceReason reason{ServiceReason::kNotFound};
  std::optional<RuntimeObjectView> object;
};

struct RuntimeObjectQueryResult
{
  bool accepted{false};
  ServiceReason reason{ServiceReason::kInvalidRequest};
  std::vector<RuntimeObjectView> objects;
  std::uint64_t next_page_token{0U};
  bool has_more{false};
};

struct BestRuntimeCandidateResult
{
  ServiceReason reason{ServiceReason::kNoEligibleCandidate};
  std::optional<RuntimeObjectView> object;
};

struct DiagnosticRuntimeCandidate
{
  RuntimeObjectView view;
  TaskRelevanceResult relevance;
};

class RuntimeTaskServiceCoordinator
{
public:
  RuntimeTaskServiceCoordinator(
    TaskRelevanceConfig relevance_config,
    BestCandidateConfig best_candidate_config,
    std::uint64_t initial_epoch);

  void synchronize(
    const MemoryUpdateResult & snapshot,
    const std::map<GlobalObjectKey,
      std::vector<AppearancePrototype>> & appearance);
  bool accept_task(
    const SemanticTaskEvidence & task,
    std::string query_text,
    std::uint64_t producer_epoch_id,
    std::int64_t source_stamp_ns);
  void clear_task() noexcept;

  [[nodiscard]] GetRuntimeObjectResult get(
    const GlobalObjectKey & key) const;
  [[nodiscard]] RuntimeObjectQueryResult query_active(
    const SemanticTaskKey & expected_task,
    const QueryMemoryRequest & request) const;
  [[nodiscard]] RuntimeObjectQueryResult query_descriptor(
    const SemanticTaskEvidence & task,
    const std::string & query_text,
    const QueryMemoryRequest & request) const;
  InspectionResult mark_inspected(
    const GlobalObjectKey & key,
    InspectionState state);
  [[nodiscard]] BestRuntimeCandidateResult best_candidate() const;
  [[nodiscard]] std::vector<DiagnosticRuntimeCandidate>
  diagnostic_ranking() const;
  void reset_to_epoch(std::uint64_t new_epoch, std::string reason);

  [[nodiscard]] const std::optional<SemanticTaskKey> & active_task() const noexcept;
  [[nodiscard]] std::uint64_t current_epoch() const noexcept;
  [[nodiscard]] const std::vector<MemoryServiceEvent> & service_events() const noexcept;
  [[nodiscard]] std::vector<RuntimeObjectView> active_objects() const;

private:
  [[nodiscard]] std::vector<ObjectTaskEvidence> task_evidence(
    const std::string & query_text) const;
  [[nodiscard]] std::vector<MemoryServiceRecord> records_for_overlay(
    const TaskRelevanceOverlay & overlay,
    bool scored_only) const;
  [[nodiscard]] RuntimeObjectView view_from_record(
    const MemoryServiceRecord & record,
    const std::optional<SemanticTaskKey> & task) const;
  [[nodiscard]] RuntimeObjectQueryResult query_with_overlay(
    const TaskRelevanceOverlay & overlay,
    const std::optional<SemanticTaskKey> & task,
    const QueryMemoryRequest & request) const;
  void refresh_active_overlay();

  TaskRelevanceConfig relevance_config_;
  BestCandidateConfig best_candidate_config_;
  MemoryServiceStore store_;
  TaskRelevanceOverlay overlay_;
  std::map<GlobalObjectKey, MemoryObject> objects_;
  std::map<GlobalObjectKey, std::vector<AppearancePrototype>> appearance_;
  std::optional<SemanticTaskEvidence> active_task_evidence_;
  std::string active_query_text_;
  std::uint64_t task_producer_epoch_id_{0U};
  std::optional<std::int64_t> last_task_source_stamp_ns_;
};

}  // namespace track_robot_semantic_memory
