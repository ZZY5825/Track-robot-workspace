#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "track_robot_semantic_memory/appearance_memory.hpp"
#include "track_robot_semantic_memory/id_types.hpp"
#include "track_robot_semantic_memory/types.hpp"

namespace track_robot_semantic_memory
{

struct SemanticTaskKey
{
  std::uint64_t query_id{0U};
  std::uint64_t query_version{0U};

  [[nodiscard]] bool valid() const noexcept;
};

struct SemanticTaskEvidence
{
  SemanticTaskKey key;
  AppearanceDescriptor descriptor;
};

struct PermanentSemanticEvidence
{
  std::string label;
  double confidence{0.0};
  double task_similarity{0.0};
  bool permanent{true};
};

struct ObjectTaskEvidence
{
  GlobalObjectKey key;
  LifecycleState lifecycle{LifecycleState::kTentative};
  std::vector<AppearancePrototype> prototypes;
  std::vector<PermanentSemanticEvidence> permanent_semantics;
};

struct TaskRelevanceConfig
{
  double appearance_weight{0.75};
  double semantic_weight{0.25};
  double normalization_tolerance{1e-4};
  std::size_t maximum_semantic_evidence{16U};

  void validate() const;
};

struct TaskRelevanceResult
{
  bool eligible{false};
  double relevance{0.0};
  double appearance_similarity{0.0};
  double semantic_similarity{0.0};
  std::string reason;
};

class TaskRelevanceScorer
{
public:
  explicit TaskRelevanceScorer(TaskRelevanceConfig config);

  [[nodiscard]] TaskRelevanceResult score(
    const SemanticTaskEvidence & task,
    const ObjectTaskEvidence & object) const;

private:
  TaskRelevanceConfig config_;
};

class TaskRelevanceOverlay
{
public:
  explicit TaskRelevanceOverlay(TaskRelevanceConfig config);

  std::size_t recompute(
    const SemanticTaskEvidence & task,
    const std::vector<ObjectTaskEvidence> & objects);
  void clear() noexcept;

  [[nodiscard]] const std::optional<SemanticTaskKey> & active_task() const noexcept;
  [[nodiscard]] std::optional<double> relevance(
    const GlobalObjectKey & key) const noexcept;

private:
  double normalization_tolerance_;
  TaskRelevanceScorer scorer_;
  std::optional<SemanticTaskKey> active_task_;
  std::map<GlobalObjectKey, double> relevance_by_object_;
};

}  // namespace track_robot_semantic_memory
