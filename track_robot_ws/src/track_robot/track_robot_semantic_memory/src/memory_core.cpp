#include "track_robot_semantic_memory/memory_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

#include "track_robot_semantic_memory/multisensor_update.hpp"

namespace track_robot_semantic_memory
{
namespace
{

template<typename ContainerT>
bool finite_array(const ContainerT & values)
{
  return std::all_of(values.begin(), values.end(),
    [](double value) {return std::isfinite(value);});
}

double speed(const std::array<double, 3> & velocity)
{
  return std::sqrt(
    velocity[0] * velocity[0] + velocity[1] * velocity[1] +
    velocity[2] * velocity[2]);
}

bool contains_key(
  const std::vector<ProducerObjectKey> & keys, const ProducerObjectKey & key)
{
  return std::binary_search(keys.begin(), keys.end(), key);
}

void increment_saturating(std::uint32_t * value) noexcept
{
  if (*value != std::numeric_limits<std::uint32_t>::max()) {
    ++(*value);
  }
}

std::uint32_t saturating_sum(
  std::uint32_t first, std::uint32_t second) noexcept
{
  return std::numeric_limits<std::uint32_t>::max() - first < second ?
         std::numeric_limits<std::uint32_t>::max() : first + second;
}

double spatial_distance(
  const std::array<double, 3> & first,
  const std::array<double, 3> & second) noexcept
{
  double squared = 0.0;
  for (std::size_t index = 0U; index < first.size(); ++index) {
    const double delta = first[index] - second[index];
    squared += delta * delta;
  }
  return std::sqrt(squared);
}

double extent_similarity(
  const std::array<double, 3> & first,
  const std::array<double, 3> & second) noexcept
{
  double total = 0.0;
  for (std::size_t index = 0U; index < first.size(); ++index) {
    const double maximum = std::max(first[index], second[index]);
    total += maximum > 0.0 ? std::min(first[index], second[index]) / maximum : 0.0;
  }
  return total / 3.0;
}

double semantic_similarity(
  const MemoryObject & first,
  const MemoryObject & second)
{
  using SemanticKey = std::tuple<std::string, std::string, std::uint8_t>;
  std::set<SemanticKey> first_keys;
  std::set<SemanticKey> second_keys;
  for (const auto & label : first.semantic_labels) {
    if (label.evidence_kind != 1U) {
      first_keys.emplace(label.label, label.provenance, label.evidence_kind);
    }
  }
  for (const auto & label : second.semantic_labels) {
    if (label.evidence_kind != 1U) {
      second_keys.emplace(label.label, label.provenance, label.evidence_kind);
    }
  }
  if (first_keys.empty() || second_keys.empty()) {
    return 0.0;
  }
  std::size_t intersection = 0U;
  for (const auto & key : first_keys) {
    intersection += second_keys.count(key);
  }
  const auto union_size = first_keys.size() + second_keys.size() - intersection;
  return union_size == 0U ? 0.0 :
    static_cast<double>(intersection) / static_cast<double>(union_size);
}

}  // namespace

void LidarObservation::validate() const
{
  if (!source_key.valid() || source_stamp_ns < 0) {
    throw std::invalid_argument("LiDAR observation source identity is invalid");
  }
  if (!finite_array(position) || !finite_array(velocity) ||
    !finite_array(extent) || !finite_array(position_covariance))
  {
    throw std::invalid_argument("LiDAR observation geometry must be finite");
  }
  if (std::any_of(extent.begin(), extent.end(), [](double value) {return value < 0.0;})) {
    throw std::invalid_argument("LiDAR observation extent must be non-negative");
  }
  if (!std::isfinite(confidence) || confidence < 0.0 || confidence > 1.0) {
    throw std::invalid_argument("LiDAR observation confidence must be in [0,1]");
  }
}

void VisualMemorySupplement::validate() const
{
  if (!lidar_key.valid() || !visual_key.valid() ||
    observation_producer_epoch_id == 0U || observation_id == 0U ||
    visual_candidate_id == 0U || camera_stamp_ns < 0 ||
    !std::isfinite(association_confidence) || association_confidence < 0.0 ||
    association_confidence > 1.0 || semantic_labels.size() > 16U)
  {
    throw std::invalid_argument("visual memory supplement identity is invalid");
  }
  for (const auto & label : semantic_labels) {
    if (label.label.empty() || label.label.size() > 128U ||
      label.provenance.empty() || label.provenance.size() > 128U ||
      !std::isfinite(label.confidence) || label.confidence < 0.0 ||
      label.confidence > 1.0 || label.evidence_kind > 3U)
    {
      throw std::invalid_argument("visual semantic evidence is invalid");
    }
  }
}

void MemoryCoreConfig::validate() const
{
  if (max_objects == 0U || max_objects > 256U || max_history > 16U ||
    max_feature_prototypes == 0U || max_feature_prototypes > 4U)
  {
    throw std::invalid_argument("MemoryCore bounds exceed the Phase 2 contract");
  }
  if (rollback_tolerance_ns < 0 || !std::isfinite(static_process_noise) ||
    !std::isfinite(dynamic_process_noise) || static_process_noise < 0.0 ||
    dynamic_process_noise < 0.0 ||
    !std::isfinite(appearance_minimum_quality) ||
    appearance_minimum_quality < 0.0 || appearance_minimum_quality > 1.0 ||
    !std::isfinite(appearance_new_prototype_similarity_threshold) ||
    appearance_new_prototype_similarity_threshold < -1.0 ||
    appearance_new_prototype_similarity_threshold > 1.0 ||
    !std::isfinite(appearance_normalization_tolerance) ||
    appearance_normalization_tolerance < 0.0 ||
    appearance_normalization_tolerance >= 1.0)
  {
    throw std::invalid_argument("MemoryCore timing or noise configuration is invalid");
  }
  static_lifecycle.validate();
  dynamic_lifecycle.validate();
  (void)MotionClassifier(static_max_speed_mps, dynamic_min_speed_mps);
}

MemoryCore::MemoryCore(
  MemoryCoreConfig config, std::uint64_t initial_memory_epoch_id)
: config_(config),
  domain_tracker_(initial_memory_epoch_id),
  clock_(config.rollback_tolerance_ns),
  static_lifecycle_(config.static_lifecycle),
  dynamic_lifecycle_(config.dynamic_lifecycle),
  motion_classifier_(config.static_max_speed_mps, config.dynamic_min_speed_mps)
{
  config_.validate();
}

MemoryUpdateResult MemoryCore::update(
  const MemoryDomainKey & domain,
  std::int64_t batch_stamp_ns,
  std::vector<LidarObservation> observations)
{
  if (batch_stamp_ns < 0) {
    throw std::invalid_argument("batch source stamp must be non-negative");
  }
  for (const auto & observation : observations) {
    observation.validate();
    if (observation.source_stamp_ns > batch_stamp_ns) {
      throw std::invalid_argument("LiDAR observation cannot be newer than its batch");
    }
  }
  MemoryUpdateResult result;
  const auto domain_transition = domain_tracker_.update(domain);
  if (domain_transition.changed) {
    clear_for_new_epoch();
    clock_.reset(batch_stamp_ns);
    result.events.push_back({MemoryEventType::kDomainChanged, {}});
  } else if (domain.mode() == MemoryMode::kObservationOnly) {
    (void)domain_tracker_.advance_epoch();
    clear_for_new_epoch();
    clock_.reset(batch_stamp_ns);
    result.events.push_back({MemoryEventType::kMemoryReset, {}});
  } else {
    const auto clock_result = clock_.observe(batch_stamp_ns);
    if (clock_result == ClockObservation::kRollback) {
      (void)domain_tracker_.advance_epoch();
      clear_for_new_epoch();
      clock_.reset(batch_stamp_ns);
      result.events.push_back({MemoryEventType::kDomainChanged, {}});
    } else if (clock_result == ClockObservation::kOutOfOrder) {
      result.rejected_observations = static_cast<std::uint32_t>(observations.size());
      for (std::size_t index = 0U; index < observations.size(); ++index) {
        result.events.push_back({MemoryEventType::kObservationRejected, {}});
      }
      return finish_result(std::move(result));
    }
  }
  for (auto & object : objects_) {
    if (object.reidentification_state == ReidentificationState::kConfirmed) {
      object.reidentification_state = ReidentificationState::kNotRequired;
    }
  }
  std::sort(observations.begin(), observations.end(),
    [](const auto & left, const auto & right) {
      return left.source_key < right.source_key;
    });
  std::vector<ProducerObjectKey> observed_keys;
  for (std::size_t begin = 0U; begin < observations.size();) {
    std::size_t end = begin + 1U;
    while (end < observations.size() &&
      observations[end].source_key == observations[begin].source_key)
    {
      ++end;
    }
    if (end - begin > 1U) {
      result.rejected_observations += static_cast<std::uint32_t>(end - begin);
      for (std::size_t index = begin; index < end; ++index) {
        result.events.push_back({MemoryEventType::kObservationRejected, {}});
      }
      begin = end;
      continue;
    }
    const auto & observation = observations[begin];
    observed_keys.push_back(observation.source_key);
    auto * object = find_by_source(observation.source_key);
    if (object == nullptr) {
      if (!make_capacity(result)) {
        ++result.rejected_observations;
        result.events.push_back({MemoryEventType::kObservationRejected, {}});
        begin = end;
        continue;
      }
      object = &create_object(observation, result);
    }
    apply_observation(*object, observation, result);
    begin = end;
  }
  advance_unobserved(batch_stamp_ns, observed_keys, result);
  return finish_result(std::move(result));
}

MemoryUpdateResult MemoryCore::reset(const MemoryDomainKey & domain)
{
  if (!domain_tracker_.domain().has_value() ||
    *domain_tracker_.domain() != domain)
  {
    throw std::invalid_argument(
            "memory reset requires the current spatial domain");
  }
  (void)domain_tracker_.advance_epoch();
  clear_for_new_epoch();
  MemoryUpdateResult result;
  result.events.push_back({MemoryEventType::kMemoryReset, {}});
  return finish_result(std::move(result));
}

VisualSupplementResult MemoryCore::supplement_visual(
  const MemoryDomainKey & domain,
  const VisualMemorySupplement & supplement)
{
  supplement.validate();
  MemoryUpdateResult result;
  bool appearance_accepted = false;
  std::string appearance_reason;
  std::optional<CurrentAppearanceEvidence> current_appearance_evidence;
  auto finish = [
    this, &result, &appearance_accepted, &appearance_reason,
    &current_appearance_evidence](
    bool accepted, std::string reason) mutable {
      return VisualSupplementResult{
        accepted, appearance_accepted, std::move(reason),
        std::move(appearance_reason), std::move(current_appearance_evidence),
        finish_result(std::move(result))};
    };
  if (!domain_tracker_.domain().has_value() ||
    *domain_tracker_.domain() != domain)
  {
    return finish(false, "visual supplement domain is not current");
  }
  if (!supplement.association_confirmed || supplement.ambiguous) {
    return finish(false, "visual supplement is not an unambiguous confirmed association");
  }
  auto * object = find_by_source(supplement.lidar_key);
  if (object == nullptr || !domain_tracker_.accepts(object->key) ||
    object->lifecycle == LifecycleState::kLost ||
    object->lifecycle == LifecycleState::kArchived)
  {
    return finish(false, "attached LiDAR object is not active in this memory epoch");
  }
  if (object->visual_observation_producer_epoch_id ==
    supplement.observation_producer_epoch_id &&
    object->visual_observation_id == supplement.observation_id)
  {
    return finish(false, "duplicate visual observation");
  }
  if (object->last_camera_seen_ns >= 0 &&
    supplement.camera_stamp_ns <= object->last_camera_seen_ns)
  {
    return finish(false, "visual source time did not advance");
  }

  const auto permissions = multisensor_update_permissions(
    object->lifecycle, SupportState::kCameraLidar);
  const bool attachment_changed = !object->attached_visual_key.has_value() ||
    *object->attached_visual_key != supplement.visual_key;
  auto * previous_owner = find_by_visual(supplement.visual_key);
  if (previous_owner != nullptr && previous_owner != object) {
    previous_owner->attached_visual_key.reset();
    previous_owner->association_confidence = 0.0;
    previous_owner->visibility = VisibilityState::kUnknown;
    if (previous_owner->support == SupportState::kCameraLidar) {
      previous_owner->support = SupportState::kLidarOnly;
    } else if (previous_owner->support == SupportState::kCameraOnly) {
      previous_owner->support = SupportState::kNone;
    }
    result.events.push_back({
        MemoryEventType::kAssociationDetached, previous_owner->key,
        previous_owner->lifecycle, previous_owner->lifecycle});
  }
  object->attached_visual_key = supplement.visual_key;
  object->visual_observation_producer_epoch_id =
    supplement.observation_producer_epoch_id;
  object->visual_observation_id = supplement.observation_id;
  object->visual_candidate_id = supplement.visual_candidate_id;
  object->last_camera_seen_ns = supplement.camera_stamp_ns;
  object->support = SupportState::kCameraLidar;
  object->visibility = VisibilityState::kVisible;
  increment_saturating(&object->camera_observation_count);
  object->association_confidence = supplement.association_confidence;
  if (permissions.confidence_increase) {
    object->confidence += 0.25 * (
      supplement.association_confidence - object->confidence);
    object->confidence = std::clamp(object->confidence, 0.0, 1.0);
  }
  if (permissions.semantics && supplement.semantic_evidence_valid) {
    for (const auto & label : supplement.semantic_labels) {
      if (label.evidence_kind == 1U) {
        continue;
      }
      const auto found = std::find_if(
        object->semantic_labels.begin(), object->semantic_labels.end(),
        [&label](const auto & existing) {
          return existing.label == label.label &&
                 existing.provenance == label.provenance &&
                 existing.evidence_kind == label.evidence_kind;
        });
      if (found != object->semantic_labels.end()) {
        if (label.confidence >= found->confidence) {
          *found = label;
        }
      } else if (object->semantic_labels.size() < 16U) {
        object->semantic_labels.push_back(label);
      }
    }
    std::sort(
      object->semantic_labels.begin(), object->semantic_labels.end(),
      [](const auto & left, const auto & right) {
        return std::tie(left.label, left.provenance, left.evidence_kind) <
               std::tie(right.label, right.provenance, right.evidence_kind);
      });
    increment_saturating(&object->semantic_update_count);
  }
  if (permissions.appearance && supplement.appearance_evidence_valid) {
    if (!supplement.appearance_descriptor.has_value()) {
      appearance_reason = "appearance descriptor is unavailable";
    } else {
      const AppearanceMemoryConfig appearance_config{
        config_.max_feature_prototypes,
        config_.appearance_minimum_quality,
        config_.appearance_new_prototype_similarity_threshold,
        config_.appearance_normalization_tolerance};
      auto bank_found = appearance_banks_.find(object->key);
      AppearanceMemory pending = bank_found == appearance_banks_.end() ?
        AppearanceMemory(appearance_config) : bank_found->second;
      const auto appearance_result = pending.update(AppearanceObservation{
          *supplement.appearance_descriptor, supplement.appearance_quality,
          supplement.association_confirmed, supplement.ambiguous,
          supplement.prediction_only});
      appearance_reason = appearance_result.reason;
      if (appearance_result.decision != AppearanceUpdateDecision::kRejected) {
        if (bank_found == appearance_banks_.end()) {
          appearance_banks_.emplace(object->key, std::move(pending));
        } else {
          bank_found->second = std::move(pending);
        }
        increment_saturating(&object->appearance_update_count);
        refresh_appearance_summary(*object);
        appearance_accepted = true;
        current_appearance_evidence = CurrentAppearanceEvidence{
          object->key,
          *supplement.appearance_descriptor,
          supplement.camera_stamp_ns,
          supplement.observation_producer_epoch_id,
          supplement.observation_id,
          object->appearance_update_count};
      }
    }
  } else if (supplement.appearance_evidence_valid) {
    appearance_reason = "appearance mutation is not permitted for this object state";
  }
  if (attachment_changed) {
    result.events.push_back({
        MemoryEventType::kAssociationAttached, object->key,
        object->lifecycle, object->lifecycle});
  }
  return finish(true, "confirmed delayed visual supplement applied");
}

RuntimeReidentificationFrame MemoryCore::make_reidentification_frame(
  const MemoryDomainKey & domain,
  std::uint64_t frame_index,
  const ReidentificationConfig & config,
  const std::vector<CurrentAppearanceEvidence> & current_appearance_evidence) const
{
  config.validate();
  if (frame_index == 0U || !domain_tracker_.domain().has_value() ||
    *domain_tracker_.domain() != domain ||
    current_appearance_evidence.size() > config.maximum_candidates)
  {
    throw std::invalid_argument(
            "reidentification frame requires the current domain and frame index");
  }
  RuntimeReidentificationFrame frame;
  frame.frame_index = frame_index;
  frame.memory_epoch_id = domain_tracker_.memory_epoch_id();
  std::vector<const MemoryObject *> candidates;
  std::vector<const MemoryObject *> targets;
  std::map<GlobalObjectKey, CurrentAppearanceEvidence> current_candidates;
  for (const auto & evidence : current_appearance_evidence) {
    const auto & key = evidence.object_key;
    if (!key.valid() || !domain_tracker_.accepts(key) ||
      !current_candidates.emplace(key, evidence).second)
    {
      throw std::invalid_argument(
              "reidentification candidates must be unique and current");
    }
    const auto found = std::find_if(
      objects_.begin(), objects_.end(),
      [&key](const auto & object) {return object.key == key;});
    const auto bank = appearance_banks_.find(key);
    const auto descriptor_gate = descriptor_compatibility_gate(
      evidence.descriptor, evidence.descriptor,
      config_.appearance_normalization_tolerance);
    if (found == objects_.end() || bank == appearance_banks_.end() ||
      bank->second.prototypes().empty() ||
      found->lifecycle == LifecycleState::kLost ||
      found->lifecycle == LifecycleState::kArchived ||
      found->support == SupportState::kPredictionOnly ||
      !found->attached_visual_key.has_value() ||
      found->last_camera_seen_ns != evidence.camera_stamp_ns ||
      found->visual_observation_producer_epoch_id !=
      evidence.observation_producer_epoch_id ||
      found->visual_observation_id != evidence.observation_id ||
      found->appearance_update_count != evidence.appearance_update_count ||
      !descriptor_gate.gate_passed)
    {
      throw std::invalid_argument(
              "reidentification candidate lacks current accepted appearance evidence");
    }
    candidates.push_back(&*found);
  }
  for (const auto & object : objects_) {
    const auto bank = appearance_banks_.find(object.key);
    if (bank == appearance_banks_.end() || bank->second.prototypes().empty()) {
      continue;
    }
    if (object.lifecycle == LifecycleState::kLost) {
      targets.push_back(&object);
    }
  }
  auto by_key = [](const auto * left, const auto * right) {
      return left->key < right->key;
    };
  std::sort(candidates.begin(), candidates.end(), by_key);
  std::sort(targets.begin(), targets.end(), by_key);
  if (candidates.size() > config.maximum_candidates ||
    targets.size() > config.maximum_lost_targets ||
    candidates.size() * targets.size() > config.maximum_pairs)
  {
    throw std::invalid_argument("reidentification evidence frame exceeds bounds");
  }
  for (const auto * candidate : candidates) {
    frame.candidates.push_back(candidate->key);
  }
  for (const auto * target : targets) {
    frame.lost_targets.push_back(target->key);
  }
  for (const auto * target : targets) {
    const auto & target_bank = appearance_banks_.at(target->key);
    for (const auto * candidate : candidates) {
      const auto & candidate_descriptor =
        current_candidates.at(candidate->key).descriptor;
      double appearance = 0.0;
      for (const auto & target_prototype : target_bank.prototypes()) {
        const auto term = descriptor_cosine_term(
          candidate_descriptor, target_prototype.descriptor, 1.0,
          config_.appearance_normalization_tolerance);
        if (term.valid) {
          appearance = std::max(appearance, term.normalized_value);
        }
      }
      const auto current_stamp = std::max(
        candidate->state_stamp_ns, candidate->last_camera_seen_ns);
      frame.pairs.push_back(RuntimeReidentificationPair{
          target->key,
          candidate->key,
          candidate->lidar_key,
          candidate->attached_visual_key,
          target->lifecycle,
          target->key.memory_epoch_id == frame.memory_epoch_id &&
          candidate->key.memory_epoch_id == frame.memory_epoch_id,
          std::max<std::int64_t>(0, current_stamp - target->last_seen_ns),
          spatial_distance(target->position, candidate->position),
          appearance,
          extent_similarity(target->extent, candidate->extent),
          semantic_similarity(*target, *candidate)});
    }
  }
  return frame;
}

const std::vector<AppearancePrototype> * MemoryCore::appearance_prototypes(
  const GlobalObjectKey & key) const noexcept
{
  if (!domain_tracker_.accepts(key)) {
    return nullptr;
  }
  const auto found = appearance_banks_.find(key);
  return found == appearance_banks_.end() ? nullptr : &found->second.prototypes();
}

MemoryUpdateResult MemoryCore::apply_reidentification_states(
  const MemoryDomainKey & domain,
  const std::vector<ReidentificationStateUpdate> & updates)
{
  if (!domain_tracker_.domain().has_value() || *domain_tracker_.domain() != domain) {
    throw std::invalid_argument("reidentification states require the current domain");
  }
  std::set<GlobalObjectKey> keys;
  for (const auto & update : updates) {
    const auto * object = find_by_key(update.key);
    if (object == nullptr || !domain_tracker_.accepts(update.key) ||
      !keys.insert(update.key).second ||
      update.state == ReidentificationState::kConfirmed)
    {
      throw std::invalid_argument("reidentification state update is invalid");
    }
  }
  for (auto & object : objects_) {
    if (object.lifecycle == LifecycleState::kLost) {
      object.reidentification_state = ReidentificationState::kNotRequired;
    }
  }
  for (const auto & update : updates) {
    find_by_key(update.key)->reidentification_state = update.state;
  }
  return finish_result({});
}

ReidentificationTransferResult MemoryCore::reidentify(
  const MemoryDomainKey & domain,
  const GlobalObjectKey & old_key,
  const GlobalObjectKey & replacement_key,
  const ProducerObjectKey & expected_replacement_lidar_key,
  const std::optional<VisualAssociationKey> & expected_replacement_visual_key)
{
  MemoryUpdateResult result;
  auto reject = [this, &result](std::string reason) mutable {
      return ReidentificationTransferResult{
        false, std::move(reason), {}, finish_result(std::move(result))};
    };
  if (!domain_tracker_.domain().has_value() || *domain_tracker_.domain() != domain ||
    old_key == replacement_key || !domain_tracker_.accepts(old_key) ||
    !domain_tracker_.accepts(replacement_key))
  {
    return reject("reidentification keys or domain are not current");
  }
  auto * old_object = find_by_key(old_key);
  auto * replacement = find_by_key(replacement_key);
  if (old_object == nullptr || replacement == nullptr ||
    old_object->lifecycle != LifecycleState::kLost ||
    replacement->lifecycle == LifecycleState::kLost ||
    replacement->lifecycle == LifecycleState::kArchived)
  {
    return reject("reidentification lifecycle preconditions failed");
  }
  if (replacement->lidar_key != expected_replacement_lidar_key ||
    replacement->attached_visual_key != expected_replacement_visual_key ||
    !expected_replacement_lidar_key.valid() ||
    !expected_replacement_visual_key.has_value() ||
    !expected_replacement_visual_key->valid())
  {
    return reject("reidentification replacement evidence changed after scoring");
  }

  MemoryObject merged = *replacement;
  merged.key = old_object->key;
  merged.first_seen_ns = old_object->first_seen_ns;
  merged.reidentification_state = ReidentificationState::kConfirmed;
  merged.short_history = old_object->short_history;
  for (const auto & sample : replacement->short_history) {
    merged.short_history.push_back(sample);
    if (merged.short_history.size() > config_.max_history) {
      merged.short_history.erase(merged.short_history.begin());
    }
  }
  merged.semantic_labels = old_object->semantic_labels;
  for (const auto & label : replacement->semantic_labels) {
    if (label.evidence_kind == 1U) {
      continue;
    }
    const auto existing = std::find_if(
      merged.semantic_labels.begin(), merged.semantic_labels.end(),
      [&label](const auto & value) {
        return std::tie(value.label, value.provenance, value.evidence_kind) ==
               std::tie(label.label, label.provenance, label.evidence_kind);
      });
    if (existing != merged.semantic_labels.end()) {
      if (label.confidence >= existing->confidence) {
        *existing = label;
      }
    } else if (merged.semantic_labels.size() < 16U) {
      merged.semantic_labels.push_back(label);
    }
  }
  std::sort(
    merged.semantic_labels.begin(), merged.semantic_labels.end(),
    [](const auto & left, const auto & right) {
      return std::tie(left.label, left.provenance, left.evidence_kind) <
             std::tie(right.label, right.provenance, right.evidence_kind);
    });

  const AppearanceMemoryConfig appearance_config{
    config_.max_feature_prototypes,
    config_.appearance_minimum_quality,
    config_.appearance_new_prototype_similarity_threshold,
    config_.appearance_normalization_tolerance};
  AppearanceMemory merged_bank(appearance_config);
  const auto old_bank = appearance_banks_.find(old_key);
  if (old_bank != appearance_banks_.end()) {
    merged_bank = old_bank->second;
  }
  const auto replacement_bank = appearance_banks_.find(replacement_key);
  if (replacement_bank != appearance_banks_.end()) {
    (void)merged_bank.merge_from(replacement_bank->second);
  }
  merged.compatible_hit_count = saturating_sum(
    old_object->compatible_hit_count, replacement->compatible_hit_count);
  merged.observation_count = saturating_sum(
    old_object->observation_count, replacement->observation_count);
  merged.camera_observation_count = saturating_sum(
    old_object->camera_observation_count, replacement->camera_observation_count);
  merged.semantic_update_count = saturating_sum(
    old_object->semantic_update_count, replacement->semantic_update_count);
  merged.appearance_update_count = saturating_sum(
    old_object->appearance_update_count, replacement->appearance_update_count);

  const auto old_lidar_key = old_object->lidar_key;
  const auto replacement_lidar_key = replacement->lidar_key;
  source_index_.erase(old_lidar_key);
  source_index_.erase(replacement_lidar_key);
  appearance_banks_.erase(old_key);
  appearance_banks_.erase(replacement_key);
  objects_.erase(std::remove_if(
      objects_.begin(), objects_.end(),
      [&old_key, &replacement_key](const auto & object) {
        return object.key == old_key || object.key == replacement_key;
      }), objects_.end());
  objects_.push_back(std::move(merged));
  source_index_[replacement_lidar_key] = old_key;
  if (!merged_bank.prototypes().empty()) {
    appearance_banks_.emplace(old_key, std::move(merged_bank));
  }
  auto * preserved = find_by_key(old_key);
  refresh_appearance_summary(*preserved);
  result.events.push_back({
      MemoryEventType::kReidentified, old_key,
      LifecycleState::kLost, preserved->lifecycle});
  return ReidentificationTransferResult{
    true, "replacement identity transferred to the preserved global object",
    old_key, finish_result(std::move(result))};
}

void MemoryCore::clear_for_new_epoch()
{
  objects_.clear();
  source_index_.clear();
  appearance_banks_.clear();
  next_global_object_id_ = 1U;
}

MemoryObject * MemoryCore::find_by_key(const GlobalObjectKey & key)
{
  const auto found = std::find_if(objects_.begin(), objects_.end(),
    [&key](const auto & object) {return object.key == key;});
  return found == objects_.end() ? nullptr : &*found;
}

MemoryObject * MemoryCore::find_by_source(const ProducerObjectKey & key)
{
  const auto indexed = source_index_.find(key);
  return indexed == source_index_.end() ? nullptr : find_by_key(indexed->second);
}

MemoryObject * MemoryCore::find_by_visual(const VisualAssociationKey & key)
{
  const auto found = std::find_if(objects_.begin(), objects_.end(),
    [&key](const auto & object) {
      return object.attached_visual_key.has_value() &&
             *object.attached_visual_key == key;
    });
  return found == objects_.end() ? nullptr : &*found;
}

bool MemoryCore::make_capacity(MemoryUpdateResult & result)
{
  if (objects_.size() < config_.max_objects) {
    return true;
  }
  auto candidate = objects_.end();
  for (auto lifecycle : {LifecycleState::kArchived, LifecycleState::kLost}) {
    candidate = std::min_element(objects_.begin(), objects_.end(),
      [lifecycle](const auto & left, const auto & right) {
        const bool left_match = left.lifecycle == lifecycle;
        const bool right_match = right.lifecycle == lifecycle;
        if (left_match != right_match) {
          return left_match;
        }
        if (!left_match) {
          return false;
        }
        return std::tie(left.last_seen_ns, left.key.global_object_id) <
               std::tie(right.last_seen_ns, right.key.global_object_id);
      });
    if (candidate != objects_.end() && candidate->lifecycle == lifecycle) {
      break;
    }
    candidate = objects_.end();
  }
  if (candidate == objects_.end()) {
    return false;
  }
  source_index_.erase(candidate->lidar_key);
  appearance_banks_.erase(candidate->key);
  result.events.push_back({MemoryEventType::kCapacityEvicted, candidate->key});
  objects_.erase(candidate);
  return true;
}

MemoryObject & MemoryCore::create_object(
  const LidarObservation & observation, MemoryUpdateResult & result)
{
  if (next_global_object_id_ == 0U ||
    next_global_object_id_ == std::numeric_limits<std::uint64_t>::max())
  {
    throw std::overflow_error("global object ID space exhausted");
  }
  MemoryObject object;
  object.key = {domain_tracker_.memory_epoch_id(), next_global_object_id_++};
  object.lidar_key = observation.source_key;
  object.first_seen_ns = observation.source_stamp_ns;
  object.last_seen_ns = observation.source_stamp_ns;
  object.state_stamp_ns = observation.source_stamp_ns;
  objects_.push_back(object);
  source_index_[observation.source_key] = object.key;
  result.events.push_back({MemoryEventType::kCreated, object.key});
  return objects_.back();
}

void MemoryCore::apply_observation(
  MemoryObject & object, const LidarObservation & observation,
  MemoryUpdateResult & result)
{
  const bool first = object.observation_count == 0U;
  const double alpha = first ? 1.0 : std::max(0.05, observation.confidence);
  for (std::size_t i = 0; i < 3U; ++i) {
    object.position[i] = first ? observation.position[i] :
      alpha * observation.position[i] + (1.0 - alpha) * object.position[i];
    object.velocity[i] = observation.velocity[i];
    object.extent[i] = first ? observation.extent[i] :
      alpha * observation.extent[i] + (1.0 - alpha) * object.extent[i];
  }
  for (std::size_t i = 0; i < 9U; ++i) {
    object.position_covariance[i] = observation.position_covariance[i];
  }
  object.motion = motion_classifier_.classify(object.motion, speed(object.velocity));
  if (object.motion == MotionState::kStatic) {
    object.velocity = {0.0, 0.0, 0.0};
  }
  object.support = SupportState::kLidarOnly;
  object.last_seen_ns = observation.source_stamp_ns;
  object.state_stamp_ns = observation.source_stamp_ns;
  object.confidence = observation.confidence;
  ++object.compatible_hit_count;
  ++object.observation_count;
  const auto & policy = object.motion == MotionState::kDynamic ?
    dynamic_lifecycle_ : static_lifecycle_;
  transition_lifecycle(object,
    policy.evaluate(object.lifecycle, object.compatible_hit_count, 0), result);
  append_history(object);
}

void MemoryCore::advance_unobserved(
  std::int64_t batch_stamp_ns,
  const std::vector<ProducerObjectKey> & observed_keys,
  MemoryUpdateResult & result)
{
  for (auto & object : objects_) {
    if (contains_key(observed_keys, object.lidar_key) ||
      object.lifecycle == LifecycleState::kArchived)
    {
      continue;
    }
    const auto delta_ns = std::max<std::int64_t>(0, batch_stamp_ns - object.state_stamp_ns);
    if (object.motion == MotionState::kDynamic && delta_ns > 0) {
      const double delta_seconds = static_cast<double>(delta_ns) * 1e-9;
      for (std::size_t i = 0; i < 3U; ++i) {
        object.position[i] += object.velocity[i] * delta_seconds;
      }
    }
    const double process_noise = object.motion == MotionState::kDynamic ?
      config_.dynamic_process_noise : config_.static_process_noise;
    const double elapsed_seconds = static_cast<double>(delta_ns) * 1e-9;
    object.position_covariance[0] += process_noise * elapsed_seconds;
    object.position_covariance[4] += process_noise * elapsed_seconds;
    object.position_covariance[8] += process_noise * elapsed_seconds;
    object.state_stamp_ns = batch_stamp_ns;
    const auto age = std::max<std::int64_t>(0, batch_stamp_ns - object.last_seen_ns);
    const auto & policy = object.motion == MotionState::kDynamic ?
      dynamic_lifecycle_ : static_lifecycle_;
    transition_lifecycle(object,
      policy.evaluate(object.lifecycle, object.compatible_hit_count, age), result);
    object.support = object.lifecycle == LifecycleState::kLost ?
      SupportState::kNone : SupportState::kPredictionOnly;
  }
}

void MemoryCore::transition_lifecycle(
  MemoryObject & object, LifecycleState next, MemoryUpdateResult & result)
{
  if (object.lifecycle == next) {
    return;
  }
  const auto previous = object.lifecycle;
  object.lifecycle = next;
  result.events.push_back(
    {MemoryEventType::kLifecycleChanged, object.key, previous, next});
  if (next == LifecycleState::kConfirmed) {
    result.events.push_back({MemoryEventType::kConfirmed, object.key, previous, next});
  } else if (next == LifecycleState::kLost) {
    result.events.push_back({MemoryEventType::kLost, object.key, previous, next});
  } else if (next == LifecycleState::kArchived) {
    source_index_.erase(object.lidar_key);
    result.events.push_back({MemoryEventType::kArchived, object.key, previous, next});
  }
}

void MemoryCore::append_history(MemoryObject & object)
{
  if (config_.max_history == 0U) {
    return;
  }
  object.short_history.push_back(
    {object.last_seen_ns, object.position, object.support, object.lifecycle});
  if (object.short_history.size() > config_.max_history) {
    object.short_history.erase(object.short_history.begin());
  }
}

void MemoryCore::refresh_appearance_summary(MemoryObject & object)
{
  const auto found = appearance_banks_.find(object.key);
  if (found == appearance_banks_.end() || found->second.prototypes().empty()) {
    object.appearance_summary_id.clear();
    object.appearance_prototype_count = 0U;
    object.appearance_encoder_id.clear();
    object.appearance_checkpoint_id.clear();
    object.appearance_descriptor_version = 0U;
    return;
  }
  const auto & descriptor = found->second.prototypes().front().descriptor;
  object.appearance_summary_id = found->second.summary_id();
  object.appearance_prototype_count = static_cast<std::uint8_t>(
    found->second.prototypes().size());
  object.appearance_encoder_id = descriptor.encoder_id;
  object.appearance_checkpoint_id = descriptor.checkpoint_id;
  object.appearance_descriptor_version = descriptor.version;
}

MemoryUpdateResult MemoryCore::finish_result(MemoryUpdateResult result) const
{
  result.memory_epoch_id = domain_tracker_.memory_epoch_id();
  result.objects = objects_;
  std::sort(result.objects.begin(), result.objects.end(),
    [](const auto & left, const auto & right) {return left.key < right.key;});
  for (const auto & object : result.objects) {
    if (object.lifecycle != LifecycleState::kLost &&
      object.lifecycle != LifecycleState::kArchived)
    {
      result.active_objects.push_back(object);
    }
  }
  return result;
}

}  // namespace track_robot_semantic_memory
