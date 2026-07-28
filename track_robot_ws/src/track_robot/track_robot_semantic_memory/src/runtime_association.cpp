#include "track_robot_semantic_memory/runtime_association.hpp"

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

bool valid_lidar(const LidarAssociationKey & key) noexcept
{
  return key.source_epoch_id != 0U && key.tracklet_id >= 0;
}

using PairKey = std::pair<std::uint64_t, std::int64_t>;

}  // namespace

std::vector<RuntimePairCandidate> shortlist_visual_pairs(
  const std::vector<RuntimePairCandidate> & pairs,
  std::uint64_t visual_candidate_id,
  std::size_t maximum_lidar_candidates)
{
  if (visual_candidate_id == 0U || maximum_lidar_candidates == 0U ||
    maximum_lidar_candidates > 256U)
  {
    throw std::invalid_argument("visual shortlist bounds are invalid");
  }
  std::set<LidarAssociationKey, bool (*)(
      const LidarAssociationKey &, const LidarAssociationKey &)> unique(
    [](const auto & left, const auto & right) {
      return std::tie(left.source_epoch_id, left.tracklet_id) <
             std::tie(right.source_epoch_id, right.tracklet_id);
    });
  std::vector<RuntimePairCandidate> output;
  for (const auto & pair : pairs) {
    if (pair.visual_candidate_id != visual_candidate_id) {
      continue;
    }
    if (!valid_lidar(pair.lidar) || !std::isfinite(pair.score) ||
      pair.score < 0.0 || pair.score > 1.0 ||
      !unique.insert(pair.lidar).second)
    {
      throw std::invalid_argument("visual shortlist pair is invalid or duplicate");
    }
    if (pair.gates_passed) {
      output.push_back(pair);
    }
  }
  std::sort(
    output.begin(), output.end(),
    [](const auto & left, const auto & right) {
      if (left.score != right.score) {
        return left.score > right.score;
      }
      return std::tie(left.lidar.source_epoch_id, left.lidar.tracklet_id) <
             std::tie(right.lidar.source_epoch_id, right.lidar.tracklet_id);
    });
  if (output.size() > maximum_lidar_candidates) {
    output.resize(maximum_lidar_candidates);
  }
  return output;
}

void RuntimeAssociationConfig::validate() const
{
  ConfirmationConfig checked = confirmation;
  (void)AssociationConfirmation(checked);
  if (!std::isfinite(match_threshold) || match_threshold < 0.0 ||
    match_threshold > 1.0 || maximum_visuals == 0U || maximum_visuals > 64U ||
    maximum_lidars == 0U || maximum_lidars > 256U || maximum_pairs == 0U ||
    maximum_pairs > 1024U)
  {
    throw std::invalid_argument("runtime association config is invalid");
  }
}

RuntimeAssociationCoordinator::RuntimeAssociationCoordinator(
  RuntimeAssociationConfig config)
: config_(std::move(config)), confirmation_(config_.confirmation)
{
  config_.validate();
}

RuntimeAssociationResult RuntimeAssociationCoordinator::process(
  const RuntimeAssociationFrame & frame)
{
  if (frame.frame_index == 0U || frame.visuals.size() > config_.maximum_visuals ||
    frame.lidars.size() > config_.maximum_lidars ||
    frame.pairs.size() > config_.maximum_pairs)
  {
    throw std::invalid_argument("runtime association frame exceeds its bounds");
  }

  std::map<std::uint64_t, RuntimeVisualCandidate> visuals;
  std::set<VisualAssociationKey> stable_visual_keys;
  for (const auto & visual : frame.visuals) {
    if (visual.visual_candidate_id == 0U ||
      (visual.stable_key.has_value() && !visual.stable_key->valid()) ||
      !visuals.emplace(visual.visual_candidate_id, visual).second ||
      (visual.stable_key.has_value() &&
      !stable_visual_keys.insert(*visual.stable_key).second))
    {
      throw std::invalid_argument("runtime visual identities must be valid and unique");
    }
  }
  std::map<std::int64_t, LidarAssociationKey> lidars;
  std::optional<std::uint64_t> lidar_epoch;
  for (const auto & lidar : frame.lidars) {
    if (!valid_lidar(lidar) ||
      (lidar_epoch.has_value() && *lidar_epoch != lidar.source_epoch_id) ||
      !lidars.emplace(lidar.tracklet_id, lidar).second)
    {
      throw std::invalid_argument("runtime LiDAR identities must be valid and unique");
    }
    lidar_epoch = lidar.source_epoch_id;
  }

  std::map<PairKey, RuntimePairCandidate> pairs;
  for (const auto & pair : frame.pairs) {
    if (visuals.count(pair.visual_candidate_id) == 0U ||
      lidars.count(pair.lidar.tracklet_id) == 0U ||
      lidars.at(pair.lidar.tracklet_id) != pair.lidar ||
      !std::isfinite(pair.score) || pair.score < 0.0 || pair.score > 1.0 ||
      !pairs.emplace(PairKey{pair.visual_candidate_id, pair.lidar.tracklet_id}, pair).second)
    {
      throw std::invalid_argument("runtime association pairs are invalid or duplicate");
    }
  }

  std::vector<std::uint64_t> row_ids;
  std::vector<std::uint64_t> column_ids;
  for (const auto & item : visuals) {
    row_ids.push_back(item.first);
  }
  for (const auto & item : lidars) {
    column_ids.push_back(static_cast<std::uint64_t>(item.first));
  }
  OptionalCostMatrix costs(
    row_ids.size(), std::vector<std::optional<double>>(column_ids.size()));
  for (std::size_t row = 0U; row < row_ids.size(); ++row) {
    for (std::size_t column = 0U; column < column_ids.size(); ++column) {
      const auto found = pairs.find(
        {row_ids[row], static_cast<std::int64_t>(column_ids[column])});
      if (found != pairs.end() && found->second.gates_passed &&
        found->second.score >= config_.match_threshold)
      {
        costs[row][column] = 1.0 - found->second.score;
      }
    }
  }
  const double unmatched_cost = std::nextafter(
    1.0 - config_.match_threshold,
    std::numeric_limits<double>::infinity());
  const auto assignment = hungarian_assignment(
    row_ids, column_ids, costs, unmatched_cost);
  std::map<std::uint64_t, LidarAssociationKey> assigned;
  for (const auto & match : assignment.matches) {
    assigned.emplace(
      match.row_id, lidars.at(static_cast<std::int64_t>(match.column_id)));
  }

  auto next_confirmation = confirmation_;
  RuntimeAssociationResult output;
  output.decisions.reserve(visuals.size());
  for (const auto & item : visuals) {
    const auto visual_id = item.first;
    RuntimeAssociationDecision decision;
    decision.visual_candidate_id = visual_id;
    const auto assigned_it = assigned.find(visual_id);
    if (assigned_it == assigned.end()) {
      if (!item.second.stable_key.has_value()) {
        decision.decision = ConfirmationDecision::kTentative;
        decision.reason = "stable visual identity is unavailable";
      } else {
        const auto result = next_confirmation.update(ConfirmationInput{
            *item.second.stable_key, std::nullopt, 0.0,
            config_.match_threshold, false, false, frame.frame_index});
        decision.decision = result.decision;
        decision.attached_lidar = result.attached_lidar;
        decision.reason = result.reason;
      }
      output.decisions.push_back(std::move(decision));
      continue;
    }

    decision.assigned_lidar = assigned_it->second;
    const auto & assigned_pair = pairs.at(
      {visual_id, assigned_it->second.tracklet_id});
    decision.best_score = assigned_pair.score;
    decision.second_score = config_.match_threshold;
    for (const auto & pair : frame.pairs) {
      if (pair.visual_candidate_id == visual_id && pair.gates_passed &&
        pair.lidar != assigned_it->second)
      {
        decision.second_score = std::max(decision.second_score, pair.score);
      }
    }
    bool split_merge = false;
    for (const auto & pair : frame.pairs) {
      if (pair.visual_candidate_id != visual_id && pair.gates_passed &&
        pair.lidar == assigned_it->second &&
        decision.best_score - pair.score < config_.confirmation.ambiguity_margin)
      {
        split_merge = true;
      }
    }
    decision.margin = decision.best_score - decision.second_score;
    if (!item.second.stable_key.has_value()) {
      decision.decision = ConfirmationDecision::kTentative;
      decision.reason = "stable visual identity is unavailable";
    } else {
      const auto result = next_confirmation.update(ConfirmationInput{
          *item.second.stable_key, assigned_it->second,
          decision.best_score, decision.second_score, true, split_merge,
          frame.frame_index});
      decision.decision = result.decision;
      decision.attached_lidar = result.attached_lidar;
      decision.reason = result.reason;
    }
    output.decisions.push_back(std::move(decision));
  }
  confirmation_ = std::move(next_confirmation);
  return output;
}

void RuntimeAssociationCoordinator::reset() noexcept
{
  confirmation_.reset();
}

}  // namespace track_robot_semantic_memory
