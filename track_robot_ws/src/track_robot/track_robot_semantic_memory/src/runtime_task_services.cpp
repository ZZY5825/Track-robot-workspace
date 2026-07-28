#include "track_robot_semantic_memory/runtime_task_services.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace track_robot_semantic_memory
{
namespace
{

bool same_task(
  const SemanticTaskKey & left,
  const SemanticTaskKey & right) noexcept
{
  return left.query_id == right.query_id &&
         left.query_version == right.query_version;
}

bool ascii_space(unsigned char character) noexcept
{
  return character == ' ' || character == '\t' || character == '\n' ||
         character == '\v' || character == '\f' || character == '\r';
}

std::optional<std::string> normalized_text(const std::string & input)
{
  if (input.empty() || input.size() > 512U) {
    return std::nullopt;
  }
  std::string output;
  output.reserve(input.size());
  bool pending_space = false;
  for (const unsigned char character : input) {
    if (ascii_space(character)) {
      pending_space = !output.empty();
      continue;
    }
    if (character < 0x20U || character > 0x7eU) {
      return std::nullopt;
    }
    if (pending_space) {
      output.push_back(' ');
      pending_space = false;
    }
    output.push_back(static_cast<char>(
        character >= 'A' && character <= 'Z' ? character + ('a' - 'A') :
        character));
  }
  return output.empty() ? std::nullopt :
         std::optional<std::string>{std::move(output)};
}

bool active_lifecycle(LifecycleState lifecycle) noexcept
{
  return lifecycle == LifecycleState::kTentative ||
         lifecycle == LifecycleState::kConfirmed ||
         lifecycle == LifecycleState::kStale;
}

double support_quality(SupportState support) noexcept
{
  switch (support) {
    case SupportState::kCameraLidar: return 1.0;
    case SupportState::kCameraOnly: return 0.9;
    case SupportState::kLidarOnly: return 0.5;
    case SupportState::kPredictionOnly: return 0.35;
    case SupportState::kNone: return 0.0;
  }
  return 0.0;
}

}  // namespace

RuntimeTaskServiceCoordinator::RuntimeTaskServiceCoordinator(
  TaskRelevanceConfig relevance_config,
  BestCandidateConfig best_candidate_config,
  std::uint64_t initial_epoch)
: relevance_config_(std::move(relevance_config)),
  best_candidate_config_(best_candidate_config),
  store_(initial_epoch),
  overlay_(relevance_config_)
{
  relevance_config_.validate();
  if (!std::isfinite(best_candidate_config_.minimum_relevance) ||
    best_candidate_config_.minimum_relevance < 0.0 ||
    best_candidate_config_.minimum_relevance > 1.0)
  {
    throw std::invalid_argument("best-candidate configuration is invalid");
  }
}

void RuntimeTaskServiceCoordinator::synchronize(
  const MemoryUpdateResult & snapshot,
  const std::map<GlobalObjectKey,
    std::vector<AppearancePrototype>> & appearance)
{
  if (snapshot.memory_epoch_id == 0U || snapshot.objects.size() > 256U) {
    throw std::invalid_argument("runtime task snapshot violates its bound");
  }
  std::map<GlobalObjectKey, MemoryObject> next_objects;
  for (const auto & object : snapshot.objects) {
    if (!object.key.valid() ||
      object.key.memory_epoch_id != snapshot.memory_epoch_id ||
      !next_objects.emplace(object.key, object).second)
    {
      throw std::invalid_argument("runtime task snapshot contains invalid keys");
    }
  }
  std::map<GlobalObjectKey, std::vector<AppearancePrototype>> next_appearance;
  for (const auto & item : appearance) {
    if (next_objects.count(item.first) == 0U || item.second.size() > 4U) {
      throw std::invalid_argument("runtime task appearance evidence is invalid");
    }
    next_appearance.emplace(item.first, item.second);
  }

  auto next_store = store_;
  auto next_overlay = overlay_;
  auto previous_objects = std::move(objects_);
  auto previous_appearance = std::move(appearance_);
  objects_ = std::move(next_objects);
  appearance_ = std::move(next_appearance);
  auto next_active_task = active_task_evidence_;
  try {
    if (next_active_task.has_value()) {
      next_active_task->evaluation_stamp_ns =
        next_active_task->source_stamp_ns;
      for (const auto & item : objects_) {
        next_active_task->evaluation_stamp_ns = std::max(
          next_active_task->evaluation_stamp_ns,
          item.second.grounding_source_stamp_ns);
      }
      (void)next_overlay.recompute(
        *next_active_task, task_evidence(active_query_text_));
    } else {
      next_overlay.clear();
    }
    const auto records = records_for_overlay(next_overlay, false);
    next_store.synchronize(snapshot.memory_epoch_id, records);
  } catch (...) {
    objects_ = std::move(previous_objects);
    appearance_ = std::move(previous_appearance);
    throw;
  }
  store_ = std::move(next_store);
  overlay_ = std::move(next_overlay);
  active_task_evidence_ = std::move(next_active_task);
}

bool RuntimeTaskServiceCoordinator::accept_task(
  const SemanticTaskEvidence & task,
  std::string query_text,
  std::uint64_t producer_epoch_id,
  std::int64_t source_stamp_ns)
{
  const auto canonical_query = normalized_text(query_text);
  const auto descriptor_gate = descriptor_compatibility_gate(
    task.descriptor, task.descriptor,
    relevance_config_.normalization_tolerance);
  if (!task.key.valid() || producer_epoch_id == 0U || source_stamp_ns < 0 ||
    !canonical_query.has_value() ||
    !descriptor_gate.gate_passed)
  {
    clear_task();
    return false;
  }
  if (task_producer_epoch_id_ == producer_epoch_id &&
    last_task_source_stamp_ns_.has_value() &&
    source_stamp_ns < *last_task_source_stamp_ns_)
  {
    clear_task();
    task_producer_epoch_id_ = producer_epoch_id;
    last_task_source_stamp_ns_ = source_stamp_ns;
    return false;
  }

  SemanticTaskEvidence evaluated_task = task;
  evaluated_task.producer_epoch_id = producer_epoch_id;
  evaluated_task.source_stamp_ns = source_stamp_ns;
  evaluated_task.evaluation_stamp_ns = source_stamp_ns;
  for (const auto & item : objects_) {
    evaluated_task.evaluation_stamp_ns = std::max(
      evaluated_task.evaluation_stamp_ns,
      item.second.grounding_source_stamp_ns);
  }
  TaskRelevanceOverlay next_overlay(relevance_config_);
  (void)next_overlay.recompute(
    evaluated_task, task_evidence(*canonical_query));
  auto next_store = store_;
  next_store.synchronize(
    store_.current_epoch(), records_for_overlay(next_overlay, false));
  active_task_evidence_ = evaluated_task;
  active_query_text_ = *canonical_query;
  task_producer_epoch_id_ = producer_epoch_id;
  last_task_source_stamp_ns_ = source_stamp_ns;
  overlay_ = std::move(next_overlay);
  store_ = std::move(next_store);
  return true;
}

void RuntimeTaskServiceCoordinator::clear_task() noexcept
{
  active_task_evidence_.reset();
  active_query_text_.clear();
  task_producer_epoch_id_ = 0U;
  last_task_source_stamp_ns_.reset();
  overlay_.clear();
  try {
    store_.synchronize(
      store_.current_epoch(), records_for_overlay(overlay_, false));
  } catch (...) {
  }
}

GetRuntimeObjectResult RuntimeTaskServiceCoordinator::get(
  const GlobalObjectKey & key) const
{
  const auto result = store_.get(key);
  if (!result.record.has_value()) {
    return {result.reason, std::nullopt};
  }
  return {ServiceReason::kOk,
    view_from_record(*result.record, overlay_.active_task())};
}

RuntimeObjectQueryResult RuntimeTaskServiceCoordinator::query_active(
  const SemanticTaskKey & expected_task,
  const QueryMemoryRequest & request) const
{
  if (!overlay_.active_task().has_value() || !expected_task.valid() ||
    !same_task(expected_task, *overlay_.active_task()))
  {
    return {};
  }
  return query_with_overlay(overlay_, overlay_.active_task(), request);
}

RuntimeObjectQueryResult RuntimeTaskServiceCoordinator::query_descriptor(
  const SemanticTaskEvidence & task,
  const std::string & query_text,
  const QueryMemoryRequest & request) const
{
  const auto canonical_query = normalized_text(query_text);
  const auto descriptor_gate = descriptor_compatibility_gate(
    task.descriptor, task.descriptor,
    relevance_config_.normalization_tolerance);
  if (!task.key.valid() || !canonical_query.has_value() ||
    !descriptor_gate.gate_passed)
  {
    return {};
  }
  TaskRelevanceOverlay temporary(relevance_config_);
  (void)temporary.recompute(task, task_evidence(*canonical_query));
  return query_with_overlay(temporary, task.key, request);
}

InspectionResult RuntimeTaskServiceCoordinator::mark_inspected(
  const GlobalObjectKey & key,
  InspectionState state)
{
  return store_.mark_inspected(key, state);
}

BestRuntimeCandidateResult RuntimeTaskServiceCoordinator::best_candidate() const
{
  if (!best_candidate_config_.threshold_calibrated) {
    return {ServiceReason::kThresholdNotCalibrated, std::nullopt};
  }
  if (!overlay_.active_task().has_value()) {
    return {ServiceReason::kNoEligibleCandidate, std::nullopt};
  }
  MemoryServiceStore scored(store_.current_epoch());
  scored.synchronize(
    store_.current_epoch(), records_for_overlay(overlay_, true));
  for (const auto & item : records_for_overlay(overlay_, true)) {
    const auto current = store_.get(item.key);
    if (current.record.has_value() &&
      current.record->inspection != InspectionState::kNotInspected)
    {
      (void)scored.mark_inspected(item.key, current.record->inspection);
    }
  }
  const auto result = scored.best_candidate(best_candidate_config_);
  if (!result.record.has_value()) {
    return {result.reason, std::nullopt};
  }
  return {ServiceReason::kOk,
    view_from_record(*result.record, overlay_.active_task())};
}

std::vector<DiagnosticRuntimeCandidate>
RuntimeTaskServiceCoordinator::diagnostic_ranking() const
{
  std::vector<DiagnosticRuntimeCandidate> output;
  if (!overlay_.active_task().has_value()) {
    return output;
  }
  for (const auto & item : objects_) {
    if (!active_lifecycle(item.second.lifecycle)) {
      continue;
    }
    const auto relevance = overlay_.result(item.first);
    const auto record = store_.get(item.first);
    if (!relevance.has_value() || !record.record.has_value()) {
      continue;
    }
    output.push_back({
        view_from_record(*record.record, overlay_.active_task()),
        *relevance});
  }
  std::sort(
    output.begin(), output.end(),
    [](const auto & left, const auto & right) {
      if (left.relevance.relevance != right.relevance.relevance) {
        return left.relevance.relevance > right.relevance.relevance;
      }
      return left.view.object.key < right.view.object.key;
    });
  return output;
}

void RuntimeTaskServiceCoordinator::reset_to_epoch(
  std::uint64_t new_epoch,
  std::string reason)
{
  auto next_store = store_;
  const auto reset = next_store.reset(store_.current_epoch(), true, std::move(reason));
  if (!reset.reset || reset.new_epoch != new_epoch) {
    throw std::invalid_argument("runtime task reset epoch does not match memory core");
  }
  objects_.clear();
  appearance_.clear();
  store_ = std::move(next_store);
  if (active_task_evidence_.has_value()) {
    (void)overlay_.recompute(*active_task_evidence_, {});
  } else {
    overlay_.clear();
  }
}

const std::optional<SemanticTaskKey> &
RuntimeTaskServiceCoordinator::active_task() const noexcept
{
  return overlay_.active_task();
}

std::uint64_t RuntimeTaskServiceCoordinator::current_epoch() const noexcept
{
  return store_.current_epoch();
}

const std::vector<MemoryServiceEvent> &
RuntimeTaskServiceCoordinator::service_events() const noexcept
{
  return store_.events();
}

std::vector<RuntimeObjectView>
RuntimeTaskServiceCoordinator::active_objects() const
{
  std::vector<RuntimeObjectView> result;
  for (const auto & item : objects_) {
    if (!active_lifecycle(item.second.lifecycle)) {
      continue;
    }
    const auto record = store_.get(item.first);
    if (record.record.has_value()) {
      result.push_back(view_from_record(*record.record, overlay_.active_task()));
    }
  }
  return result;
}

std::vector<ObjectTaskEvidence>
RuntimeTaskServiceCoordinator::task_evidence(
  const std::string & query_text) const
{
  std::vector<ObjectTaskEvidence> result;
  result.reserve(objects_.size());
  const auto canonical_query = normalized_text(query_text);
  if (!canonical_query.has_value()) {
    return result;
  }
  for (const auto & item : objects_) {
    ObjectTaskEvidence evidence;
    evidence.key = item.first;
    evidence.lifecycle = item.second.lifecycle;
    evidence.support_quality = support_quality(item.second.support);
    const auto prototypes = appearance_.find(item.first);
    if (prototypes != appearance_.end()) {
      evidence.prototypes = prototypes->second;
    }
    for (const auto & semantic : item.second.semantic_labels) {
      const auto canonical_label = normalized_text(semantic.label);
      if (semantic.evidence_kind == 1U || !canonical_label.has_value() ||
        *canonical_label != *canonical_query)
      {
        continue;
      }
      evidence.permanent_semantics.push_back({
          semantic.label, semantic.confidence, 1.0, true});
    }
    if (item.second.grounding_query_id != 0U &&
      item.second.grounding_query_version != 0U &&
      item.second.grounding_producer_epoch_id != 0U &&
      item.second.grounding_source_stamp_ns >= 0)
    {
      evidence.active_grounding = TaskConditionedGroundingEvidence{
        {item.second.grounding_query_id,
          item.second.grounding_query_version},
        item.second.grounding_producer_epoch_id,
        item.second.grounding_source_stamp_ns,
        item.second.grounding_confidence,
        item.second.grounding_stability};
    }
    result.push_back(std::move(evidence));
  }
  return result;
}

std::vector<MemoryServiceRecord>
RuntimeTaskServiceCoordinator::records_for_overlay(
  const TaskRelevanceOverlay & overlay,
  bool scored_only) const
{
  std::vector<MemoryServiceRecord> records;
  records.reserve(objects_.size());
  for (const auto & item : objects_) {
    const auto relevance = overlay.relevance(item.first);
    if (scored_only && !relevance.has_value()) {
      continue;
    }
    InspectionState inspection = InspectionState::kNotInspected;
    const auto current = store_.get(item.first);
    if (current.record.has_value()) {
      inspection = current.record->inspection;
    }
    records.push_back({
        item.first, item.second.lifecycle, inspection,
        relevance.value_or(0.0)});
  }
  return records;
}

RuntimeObjectView RuntimeTaskServiceCoordinator::view_from_record(
  const MemoryServiceRecord & record,
  const std::optional<SemanticTaskKey> & task) const
{
  const auto object = objects_.find(record.key);
  if (object == objects_.end()) {
    throw std::logic_error("runtime task record has no object view");
  }
  const bool scored = task.has_value() &&
    ((overlay_.active_task().has_value() &&
    same_task(*task, *overlay_.active_task()) &&
    overlay_.relevance(record.key).has_value()) ||
    (!overlay_.active_task().has_value()));
  return RuntimeObjectView{
    object->second, record.inspection,
    scored ? task : std::optional<SemanticTaskKey>{},
    record.task_relevance};
}

RuntimeObjectQueryResult RuntimeTaskServiceCoordinator::query_with_overlay(
  const TaskRelevanceOverlay & overlay,
  const std::optional<SemanticTaskKey> & task,
  const QueryMemoryRequest & request) const
{
  MemoryServiceStore temporary(store_.current_epoch());
  temporary.synchronize(
    store_.current_epoch(), records_for_overlay(overlay, true));
  for (const auto & item : records_for_overlay(overlay, true)) {
    const auto current = store_.get(item.key);
    if (current.record.has_value() &&
      current.record->inspection != InspectionState::kNotInspected)
    {
      (void)temporary.mark_inspected(item.key, current.record->inspection);
    }
  }
  const auto page = temporary.query(request);
  RuntimeObjectQueryResult result{
    page.accepted, page.reason, {}, page.next_page_token, page.has_more};
  result.objects.reserve(page.records.size());
  for (const auto & record : page.records) {
    const auto object = objects_.find(record.key);
    if (object == objects_.end()) {
      throw std::logic_error("runtime task query record has no object view");
    }
    result.objects.push_back({
        object->second, record.inspection, task, record.task_relevance});
  }
  return result;
}

void RuntimeTaskServiceCoordinator::refresh_active_overlay()
{
  if (!active_task_evidence_.has_value()) {
    overlay_.clear();
    return;
  }
  (void)overlay_.recompute(
    *active_task_evidence_, task_evidence(active_query_text_));
}

}  // namespace track_robot_semantic_memory
