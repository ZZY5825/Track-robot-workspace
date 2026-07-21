#include "track_robot_semantic_memory/reidentification.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "track_robot_semantic_memory/hungarian_assignment.hpp"

namespace track_robot_semantic_memory
{
namespace
{

bool unit_value(double value) noexcept
{
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

double combined_score(const RuntimeReidentificationPair & pair) noexcept
{
  return 0.60 * pair.appearance_similarity +
         0.25 * pair.geometry_similarity +
         0.15 * pair.semantic_similarity;
}

bool pair_values_valid(const RuntimeReidentificationPair & pair) noexcept
{
  return pair.lost_key.valid() && pair.candidate_key.valid() &&
         pair.expected_candidate_lidar_key.valid() &&
         (!pair.expected_candidate_visual_key.has_value() ||
         pair.expected_candidate_visual_key->valid()) &&
         pair.lost_lifecycle == LifecycleState::kLost &&
         pair.age_ns >= 0 && std::isfinite(pair.spatial_distance_m) &&
         pair.spatial_distance_m >= 0.0 &&
         unit_value(pair.appearance_similarity) &&
         unit_value(pair.geometry_similarity) &&
         unit_value(pair.semantic_similarity);
}

bool pair_passes(
  const RuntimeReidentificationPair & pair,
  const ReidentificationConfig & config) noexcept
{
  return pair.lost_lifecycle == LifecycleState::kLost &&
         pair.domain_compatible && pair.age_ns <= config.maximum_age_ns &&
         pair.spatial_distance_m <= config.maximum_spatial_distance_m &&
         pair.appearance_similarity >= config.minimum_appearance_similarity &&
         combined_score(pair) >= config.minimum_combined_score;
}

ReidentificationResult result_for(
  const ReidentificationEvidence & evidence,
  ReidentificationDecision decision,
  std::uint32_t hits,
  double score,
  bool event_emitted,
  std::string reason)
{
  return ReidentificationResult{
    decision, evidence.object_key, evidence.candidate_id, hits, score,
    event_emitted, std::move(reason)};
}

}  // namespace

void ReidentificationConfig::validate() const
{
  if (!std::isfinite(maximum_spatial_distance_m) ||
    maximum_spatial_distance_m < 0.0 || maximum_age_ns < 0 ||
    !unit_value(minimum_appearance_similarity) ||
    !unit_value(minimum_combined_score) || confirmation_frames == 0U ||
    !std::isfinite(ambiguity_margin) || ambiguity_margin < 0.0 ||
    ambiguity_margin > 1.0 || maximum_candidates == 0U ||
    maximum_candidates > 64U || maximum_lost_targets == 0U ||
    maximum_lost_targets > 256U || maximum_pairs == 0U ||
    maximum_pairs > 1024U)
  {
    throw std::invalid_argument("reidentification config is invalid");
  }
}

ReidentificationTracker::ReidentificationTracker(
  ReidentificationConfig config)
: config_(std::move(config))
{
  config_.validate();
}

ReidentificationResult ReidentificationTracker::update(
  const ReidentificationEvidence & evidence)
{
  if (!evidence.object_key.valid() || evidence.candidate_id == 0U ||
    !std::isfinite(evidence.spatial_distance_m) ||
    evidence.spatial_distance_m < 0.0 || evidence.age_ns < 0 ||
    !unit_value(evidence.appearance_similarity) ||
    !unit_value(evidence.geometry_similarity) ||
    !unit_value(evidence.semantic_similarity))
  {
    throw std::invalid_argument("reidentification evidence is invalid");
  }

  auto & state = states_[evidence.object_key];
  if (state.seen && evidence.frame_index <= state.last_frame) {
    throw std::invalid_argument(
            "reidentification frame indexes must increase per object");
  }
  const bool consecutive = !state.seen ||
    evidence.frame_index == state.last_frame + 1U;
  state.seen = true;
  state.last_frame = evidence.frame_index;

  if (evidence.lifecycle == LifecycleState::kArchived) {
    state.consecutive_hits = 0U;
    state.confirmed = false;
    return result_for(
      evidence, ReidentificationDecision::kArchivedBlocked, 0U, 0.0, false,
      "archived objects require an explicit service-level decision");
  }
  if ((evidence.lifecycle != LifecycleState::kLost &&
    evidence.lifecycle != LifecycleState::kStale) ||
    !evidence.domain_compatible || evidence.age_ns > config_.maximum_age_ns ||
    evidence.spatial_distance_m > config_.maximum_spatial_distance_m)
  {
    state.consecutive_hits = 0U;
    state.confirmed = false;
    return result_for(
      evidence, ReidentificationDecision::kRejectedGate, 0U, 0.0, false,
      "lifecycle, domain, age, or spatial gate rejected reidentification");
  }

  const double combined_score =
    0.60 * evidence.appearance_similarity +
    0.25 * evidence.geometry_similarity +
    0.15 * evidence.semantic_similarity;
  if (evidence.appearance_similarity < config_.minimum_appearance_similarity ||
    combined_score < config_.minimum_combined_score)
  {
    state.consecutive_hits = 0U;
    state.confirmed = false;
    return result_for(
      evidence, ReidentificationDecision::kRejectedScore, 0U,
      combined_score, false,
      "appearance or combined evidence is below threshold");
  }

  if (!consecutive || state.candidate_id != evidence.candidate_id) {
    state.candidate_id = evidence.candidate_id;
    state.consecutive_hits = 1U;
    state.confirmed = false;
  } else if (!state.confirmed) {
    ++state.consecutive_hits;
  }
  if (state.consecutive_hits < config_.confirmation_frames) {
    return result_for(
      evidence, ReidentificationDecision::kTentative,
      state.consecutive_hits, combined_score, false,
      "reidentification is awaiting consecutive confirmation");
  }

  const bool emit_event = !state.confirmed;
  state.confirmed = true;
  return result_for(
    evidence, ReidentificationDecision::kConfirmed,
    state.consecutive_hits, combined_score, emit_event,
    emit_event ? "reidentification confirmed" : "reidentification already confirmed");
}

void ReidentificationTracker::reset() noexcept
{
  states_.clear();
}

RuntimeReidentificationCoordinator::RuntimeReidentificationCoordinator(
  ReidentificationConfig config)
: config_(std::move(config))
{
  config_.validate();
}

RuntimeReidentificationResult RuntimeReidentificationCoordinator::process(
  const RuntimeReidentificationFrame & frame)
{
  if (frame.frame_index == 0U || frame.memory_epoch_id == 0U ||
    (last_frame_index_ != 0U && frame.frame_index <= last_frame_index_) ||
    frame.candidates.size() > config_.maximum_candidates ||
    frame.lost_targets.size() > config_.maximum_lost_targets ||
    frame.pairs.size() > config_.maximum_pairs ||
    frame.candidates.size() * frame.lost_targets.size() > config_.maximum_pairs ||
    frame.pairs.size() != frame.candidates.size() * frame.lost_targets.size())
  {
    throw std::invalid_argument(
            "runtime reidentification frame identity or bounds are invalid");
  }

  std::set<GlobalObjectKey> candidate_set;
  std::set<GlobalObjectKey> target_set;
  for (const auto & key : frame.candidates) {
    if (!key.valid() || key.memory_epoch_id != frame.memory_epoch_id ||
      !candidate_set.insert(key).second)
    {
      throw std::invalid_argument(
              "runtime reidentification candidates must be unique and current");
    }
  }
  for (const auto & key : frame.lost_targets) {
    if (!key.valid() || key.memory_epoch_id != frame.memory_epoch_id ||
      candidate_set.count(key) != 0U || !target_set.insert(key).second)
    {
      throw std::invalid_argument(
              "runtime reidentification targets must be unique and current");
    }
  }

  using PairKey = std::pair<GlobalObjectKey, GlobalObjectKey>;
  std::map<PairKey, RuntimeReidentificationPair> pairs;
  using CandidateGuard =
    std::pair<ProducerObjectKey, std::optional<VisualAssociationKey>>;
  std::map<GlobalObjectKey, CandidateGuard> candidate_guards;
  for (const auto & pair : frame.pairs) {
    if (!pair_values_valid(pair) ||
      pair.lost_key.memory_epoch_id != frame.memory_epoch_id ||
      pair.candidate_key.memory_epoch_id != frame.memory_epoch_id ||
      target_set.count(pair.lost_key) == 0U ||
      candidate_set.count(pair.candidate_key) == 0U ||
      !pairs.emplace(PairKey{pair.lost_key, pair.candidate_key}, pair).second)
    {
      throw std::invalid_argument(
              "runtime reidentification pairs are invalid or duplicate");
    }

    const CandidateGuard guard{
      pair.expected_candidate_lidar_key,
      pair.expected_candidate_visual_key};
    const auto existing = candidate_guards.find(pair.candidate_key);
    if (existing == candidate_guards.end()) {
      candidate_guards.emplace(pair.candidate_key, guard);
    } else if (
      existing->second.first != guard.first ||
      existing->second.second != guard.second)
    {
      throw std::invalid_argument(
              "runtime reidentification candidate guards must be consistent");
    }
  }

  std::vector<GlobalObjectKey> candidates(candidate_set.begin(), candidate_set.end());
  std::vector<GlobalObjectKey> targets(target_set.begin(), target_set.end());
  std::vector<std::uint64_t> row_ids;
  std::vector<std::uint64_t> column_ids;
  row_ids.reserve(candidates.size());
  column_ids.reserve(targets.size());
  for (const auto & key : candidates) {
    row_ids.push_back(key.global_object_id);
  }
  for (const auto & key : targets) {
    column_ids.push_back(key.global_object_id);
  }

  OptionalCostMatrix costs(
    candidates.size(), std::vector<std::optional<double>>(targets.size()));
  for (std::size_t row = 0U; row < candidates.size(); ++row) {
    for (std::size_t column = 0U; column < targets.size(); ++column) {
      const auto & pair = pairs.at({targets[column], candidates[row]});
      if (pair_passes(pair, config_)) {
        costs[row][column] = 1.0 - combined_score(pair);
      }
    }
  }
  const double unmatched_cost = std::nextafter(
    1.0 - config_.minimum_combined_score,
    std::numeric_limits<double>::infinity());
  const auto assignment = hungarian_assignment(
    row_ids, column_ids, costs, unmatched_cost);
  std::map<GlobalObjectKey, GlobalObjectKey> assigned;
  for (const auto & match : assignment.matches) {
    assigned.emplace(
      GlobalObjectKey{frame.memory_epoch_id, match.column_id},
      GlobalObjectKey{frame.memory_epoch_id, match.row_id});
  }

  auto next_states = confirmation_states_;
  for (auto it = next_states.begin(); it != next_states.end();) {
    if (target_set.count(it->first.first) == 0U ||
      candidate_set.count(it->first.second) == 0U)
    {
      it = next_states.erase(it);
    } else {
      ++it;
    }
  }
  RuntimeReidentificationResult output;
  output.decisions.reserve(targets.size());
  for (const auto & target : targets) {
    RuntimeReidentificationDecision decision;
    decision.lost_key = target;
    const auto assigned_it = assigned.find(target);
    if (assigned_it == assigned.end()) {
      for (auto it = next_states.begin(); it != next_states.end();) {
        it = it->first.first == target ? next_states.erase(it) : std::next(it);
      }
      decision.decision = ReidentificationDecision::kRejectedScore;
      decision.reason = "no globally assigned reidentification candidate";
      output.decisions.push_back(std::move(decision));
      continue;
    }

    const auto candidate = assigned_it->second;
    const auto & pair = pairs.at({target, candidate});
    const double score = combined_score(pair);
    double runner_up = config_.minimum_combined_score;
    for (const auto & other_target : targets) {
      if (other_target != target) {
        const auto & other = pairs.at({other_target, candidate});
        if (pair_passes(other, config_)) {
          runner_up = std::max(runner_up, combined_score(other));
        }
      }
    }
    for (const auto & other_candidate : candidates) {
      if (other_candidate != candidate) {
        const auto & other = pairs.at({target, other_candidate});
        if (pair_passes(other, config_)) {
          runner_up = std::max(runner_up, combined_score(other));
        }
      }
    }
    decision.candidate_key = candidate;
    decision.expected_candidate_lidar_key = pair.expected_candidate_lidar_key;
    decision.expected_candidate_visual_key = pair.expected_candidate_visual_key;
    decision.combined_score = score;
    if (score - runner_up < config_.ambiguity_margin) {
      for (auto it = next_states.begin(); it != next_states.end();) {
        it = it->first.first == target ? next_states.erase(it) : std::next(it);
      }
      decision.decision = ReidentificationDecision::kRejectedScore;
      decision.reason = "row or column reidentification margin is ambiguous";
      output.decisions.push_back(std::move(decision));
      continue;
    }

    for (auto it = next_states.begin(); it != next_states.end();) {
      if (it->first.first == target && it->first.second != candidate) {
        it = next_states.erase(it);
      } else {
        ++it;
      }
    }
    auto & state = next_states[{target, candidate}];
    if (state.last_frame == 0U || frame.frame_index != state.last_frame + 1U) {
      state.consecutive_hits = 1U;
      state.confirmed = false;
    } else if (!state.confirmed &&
      state.consecutive_hits != std::numeric_limits<std::uint32_t>::max())
    {
      ++state.consecutive_hits;
    }
    state.last_frame = frame.frame_index;
    if (state.consecutive_hits >= config_.confirmation_frames) {
      state.confirmed = true;
      decision.decision = ReidentificationDecision::kConfirmed;
      decision.reason = "reidentification assignment confirmed";
    } else {
      decision.decision = ReidentificationDecision::kTentative;
      decision.reason = "reidentification assignment awaits confirmation";
    }
    decision.consecutive_hits = state.consecutive_hits;
    output.decisions.push_back(std::move(decision));
  }

  confirmation_states_ = std::move(next_states);
  last_frame_index_ = frame.frame_index;
  return output;
}

void RuntimeReidentificationCoordinator::reset() noexcept
{
  last_frame_index_ = 0U;
  confirmation_states_.clear();
}

std::size_t RuntimeReidentificationCoordinator::confirmation_state_count()
  const noexcept
{
  return confirmation_states_.size();
}

}  // namespace track_robot_semantic_memory
