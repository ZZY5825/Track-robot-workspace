#include "track_robot_semantic_memory/task_relevance_scorer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace track_robot_semantic_memory
{

bool SemanticTaskKey::valid() const noexcept
{
  return query_id != 0U && query_version != 0U;
}

void TaskRelevanceConfig::validate() const
{
  const double total_weight = appearance_weight + semantic_weight;
  if (!std::isfinite(appearance_weight) || appearance_weight < 0.0 ||
    !std::isfinite(semantic_weight) || semantic_weight < 0.0 ||
    !std::isfinite(total_weight) || total_weight <= 0.0 ||
    !std::isfinite(normalization_tolerance) || normalization_tolerance < 0.0 ||
    normalization_tolerance >= 1.0 ||
    maximum_semantic_evidence == 0U || maximum_semantic_evidence > 16U)
  {
    throw std::invalid_argument("task relevance config is invalid");
  }
}

TaskRelevanceScorer::TaskRelevanceScorer(TaskRelevanceConfig config)
: config_(std::move(config))
{
  config_.validate();
}

TaskRelevanceResult TaskRelevanceScorer::score(
  const SemanticTaskEvidence & task,
  const ObjectTaskEvidence & object) const
{
  if (!task.key.valid()) {
    return {false, 0.0, 0.0, 0.0, "task key is invalid"};
  }
  const auto task_shape = descriptor_compatibility_gate(
    task.descriptor, task.descriptor, config_.normalization_tolerance);
  if (!task_shape.gate_passed) {
    return {false, 0.0, 0.0, 0.0, task_shape.reason};
  }
  if (!object.key.valid()) {
    return {false, 0.0, 0.0, 0.0, "object key is invalid"};
  }
  if (object.lifecycle == LifecycleState::kArchived) {
    return {false, 0.0, 0.0, 0.0, "archived object is not task-rank eligible"};
  }
  if (object.prototypes.size() > 4U ||
    object.permanent_semantics.size() > config_.maximum_semantic_evidence)
  {
    return {false, 0.0, 0.0, 0.0, "object evidence exceeds bounded contract"};
  }

  bool appearance_valid = false;
  double best_appearance = 0.0;
  for (const auto & prototype : object.prototypes) {
    const auto gate = descriptor_compatibility_gate(
      task.descriptor, prototype.descriptor, config_.normalization_tolerance);
    if (!gate.gate_passed) {
      continue;
    }
    const double cosine = std::inner_product(
      task.descriptor.values.begin(), task.descriptor.values.end(),
      prototype.descriptor.values.begin(), 0.0);
    if (std::isfinite(cosine)) {
      appearance_valid = true;
      best_appearance = std::max(best_appearance, std::clamp(cosine, 0.0, 1.0));
    }
  }

  bool semantic_valid = false;
  double best_semantic = 0.0;
  for (const auto & semantic : object.permanent_semantics) {
    if (!semantic.permanent || semantic.label.empty() || semantic.label.size() > 128U ||
      !std::isfinite(semantic.confidence) || semantic.confidence < 0.0 ||
      semantic.confidence > 1.0 || !std::isfinite(semantic.task_similarity) ||
      semantic.task_similarity < 0.0 || semantic.task_similarity > 1.0)
    {
      continue;
    }
    semantic_valid = true;
    best_semantic = std::max(
      best_semantic, semantic.confidence * semantic.task_similarity);
  }
  if (!appearance_valid && !semantic_valid) {
    return {false, 0.0, 0.0, 0.0,
      "no compatible appearance or bounded permanent semantic evidence"};
  }

  const double appearance_weight = appearance_valid ? config_.appearance_weight : 0.0;
  const double semantic_weight = semantic_valid ? config_.semantic_weight : 0.0;
  const double active_weight = appearance_weight + semantic_weight;
  if (active_weight <= 0.0) {
    return {false, 0.0, 0.0, 0.0, "available evidence has zero configured weight"};
  }
  const double relevance = std::clamp(
    (appearance_weight * best_appearance + semantic_weight * best_semantic) /
    active_weight, 0.0, 1.0);
  return {true, relevance, best_appearance, best_semantic,
    "compatible task relevance computed"};
}

TaskRelevanceOverlay::TaskRelevanceOverlay(TaskRelevanceConfig config)
: normalization_tolerance_(config.normalization_tolerance),
  scorer_(std::move(config))
{
}

std::size_t TaskRelevanceOverlay::recompute(
  const SemanticTaskEvidence & task,
  const std::vector<ObjectTaskEvidence> & objects)
{
  const auto task_shape = descriptor_compatibility_gate(
    task.descriptor, task.descriptor, normalization_tolerance_);
  if (!task.key.valid() || !task_shape.gate_passed) {
    clear();
    return 0U;
  }
  std::map<GlobalObjectKey, double> next;
  for (const auto & object : objects) {
    const auto result = scorer_.score(task, object);
    if (result.eligible) {
      next[object.key] = result.relevance;
    }
  }
  relevance_by_object_ = std::move(next);
  active_task_ = task.key;
  return relevance_by_object_.size();
}

void TaskRelevanceOverlay::clear() noexcept
{
  active_task_.reset();
  relevance_by_object_.clear();
}

const std::optional<SemanticTaskKey> & TaskRelevanceOverlay::active_task() const noexcept
{
  return active_task_;
}

std::optional<double> TaskRelevanceOverlay::relevance(
  const GlobalObjectKey & key) const noexcept
{
  const auto found = relevance_by_object_.find(key);
  if (found == relevance_by_object_.end()) {
    return std::nullopt;
  }
  return found->second;
}

}  // namespace track_robot_semantic_memory
