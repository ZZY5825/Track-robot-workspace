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
  const double total_weight =
    grounding_weight + stability_weight + support_weight + semantic_weight;
  if (!std::isfinite(appearance_weight) || appearance_weight != 0.0 ||
    !std::isfinite(semantic_weight) || semantic_weight < 0.0 ||
    !std::isfinite(grounding_weight) || grounding_weight < 0.0 ||
    !std::isfinite(stability_weight) || stability_weight < 0.0 ||
    !std::isfinite(support_weight) || support_weight < 0.0 ||
    !std::isfinite(total_weight) || total_weight <= 0.0 ||
    !std::isfinite(normalization_tolerance) || normalization_tolerance < 0.0 ||
    normalization_tolerance >= 1.0 ||
    maximum_semantic_evidence == 0U || maximum_semantic_evidence > 16U ||
    maximum_grounding_age_ns <= 0)
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
  const auto reject = [](std::string reason) {
      TaskRelevanceResult result;
      result.reason = std::move(reason);
      return result;
    };
  if (!task.key.valid()) {
    return reject("task key is invalid");
  }
  const auto task_shape = descriptor_compatibility_gate(
    task.descriptor, task.descriptor, config_.normalization_tolerance);
  if (!task_shape.gate_passed) {
    return reject(task_shape.reason);
  }
  if (!object.key.valid()) {
    return reject("object key is invalid");
  }
  if (object.lifecycle == LifecycleState::kArchived) {
    return reject("archived object is not task-rank eligible");
  }
  if (object.prototypes.size() > 4U ||
    object.permanent_semantics.size() > config_.maximum_semantic_evidence)
  {
    return reject("object evidence exceeds bounded contract");
  }

  bool grounding_valid = false;
  double grounding_confidence = 0.0;
  double stability = 0.0;
  if (object.active_grounding.has_value()) {
    const auto & grounding = *object.active_grounding;
    const bool values_valid =
      grounding.key.valid() && grounding.producer_epoch_id != 0U &&
      grounding.source_stamp_ns >= 0 &&
      std::isfinite(grounding.confidence) &&
      grounding.confidence >= 0.0 && grounding.confidence <= 1.0 &&
      std::isfinite(grounding.stability) &&
      grounding.stability >= 0.0 && grounding.stability <= 1.0;
    const bool task_matches =
      grounding.key.query_id == task.key.query_id &&
      grounding.key.query_version == task.key.query_version &&
      task.producer_epoch_id != 0U &&
      grounding.producer_epoch_id == task.producer_epoch_id;
    const bool source_time_valid =
      task.source_stamp_ns >= 0 && task.evaluation_stamp_ns >= task.source_stamp_ns &&
      grounding.source_stamp_ns >= task.source_stamp_ns &&
      grounding.source_stamp_ns <= task.evaluation_stamp_ns &&
      task.evaluation_stamp_ns - grounding.source_stamp_ns <=
      config_.maximum_grounding_age_ns;
    if (values_valid && task_matches && source_time_valid) {
      grounding_valid = true;
      grounding_confidence = grounding.confidence;
      stability = grounding.stability;
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
  if (!std::isfinite(object.support_quality) ||
    object.support_quality < 0.0 || object.support_quality > 1.0)
  {
    return reject("object support quality is invalid");
  }
  if (!grounding_valid && !semantic_valid) {
    return reject(
      "no matching fresh grounding or bounded permanent semantic evidence");
  }

  const double grounding_weight =
    grounding_valid ? config_.grounding_weight : 0.0;
  const double stability_weight =
    grounding_valid ? config_.stability_weight : 0.0;
  const double support_weight =
    grounding_valid ? config_.support_weight : 0.0;
  const double semantic_weight = semantic_valid ? config_.semantic_weight : 0.0;
  const double active_weight =
    grounding_weight + stability_weight + support_weight + semantic_weight;
  if (active_weight <= 0.0) {
    return reject("available evidence has zero configured weight");
  }
  const double relevance = std::clamp(
    (grounding_weight * grounding_confidence +
    stability_weight * stability +
    support_weight * object.support_quality +
    semantic_weight * best_semantic) /
    active_weight, 0.0, 1.0);
  TaskRelevanceResult result;
  result.eligible = true;
  result.relevance = relevance;
  result.appearance_similarity = 0.0;
  result.semantic_similarity = best_semantic;
  result.grounding_confidence = grounding_confidence;
  result.stability = stability;
  result.support_quality = object.support_quality;
  result.reason = grounding_valid ?
    "fresh task-conditioned grounding relevance computed" :
    "bounded permanent semantic relevance computed";
  return result;
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
  std::map<GlobalObjectKey, TaskRelevanceResult> next;
  for (const auto & object : objects) {
    const auto result = scorer_.score(task, object);
    if (result.eligible) {
      next[object.key] = result;
    }
  }
  results_by_object_ = std::move(next);
  active_task_ = task.key;
  return results_by_object_.size();
}

void TaskRelevanceOverlay::clear() noexcept
{
  active_task_.reset();
  results_by_object_.clear();
}

const std::optional<SemanticTaskKey> & TaskRelevanceOverlay::active_task() const noexcept
{
  return active_task_;
}

std::optional<double> TaskRelevanceOverlay::relevance(
  const GlobalObjectKey & key) const noexcept
{
  const auto found = results_by_object_.find(key);
  if (found == results_by_object_.end()) {
    return std::nullopt;
  }
  return found->second.relevance;
}

std::optional<TaskRelevanceResult> TaskRelevanceOverlay::result(
  const GlobalObjectKey & key) const noexcept
{
  const auto found = results_by_object_.find(key);
  return found == results_by_object_.end() ?
    std::nullopt : std::optional<TaskRelevanceResult>{found->second};
}

}  // namespace track_robot_semantic_memory
