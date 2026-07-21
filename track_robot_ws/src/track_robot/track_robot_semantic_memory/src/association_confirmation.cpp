#include "track_robot_semantic_memory/association_confirmation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace track_robot_semantic_memory
{

AssociationConfirmation::AssociationConfirmation(ConfirmationConfig config)
: config_(std::move(config))
{
  if (config_.confirmation_frames == 0U || config_.detach_after_misses == 0U ||
    !std::isfinite(config_.ambiguity_margin) || config_.ambiguity_margin < 0.0 ||
    !std::isfinite(config_.previous_association_hysteresis) ||
    config_.previous_association_hysteresis < 0.0 ||
    config_.maximum_states == 0U || config_.maximum_states > 64U)
  {
    throw std::invalid_argument("association confirmation config is invalid");
  }
}

ConfirmationResult AssociationConfirmation::update(
  const ConfirmationInput & input)
{
  const bool valid_lidar = !input.assigned_lidar.has_value() ||
    (input.assigned_lidar->source_epoch_id != 0U &&
    input.assigned_lidar->tracklet_id >= 0);
  if (!input.visual_key.valid() || input.frame_index == 0U ||
    !std::isfinite(input.best_score) || input.best_score < 0.0 ||
    input.best_score > 1.0 || !std::isfinite(input.second_score) ||
    input.second_score < 0.0 || input.second_score > 1.0 || !valid_lidar)
  {
    throw std::invalid_argument("association confirmation input is invalid");
  }
  auto found_state = states_.find(input.visual_key);
  if (found_state == states_.end()) {
    if (states_.size() >= config_.maximum_states) {
      const auto oldest = std::min_element(
        states_.begin(), states_.end(),
        [](const auto & left, const auto & right) {
          return std::tie(left.second.last_frame, left.first) <
                 std::tie(right.second.last_frame, right.first);
        });
      states_.erase(oldest);
    }
    found_state = states_.emplace(input.visual_key, CandidateState{}).first;
  }
  auto & state = found_state->second;
  if (state.seen && input.frame_index <= state.last_frame) {
    throw std::invalid_argument(
            "association frame indexes must increase per visual candidate");
  }
  const bool consecutive_frame = !state.seen ||
    input.frame_index == state.last_frame + 1U;
  state.seen = true;
  state.last_frame = input.frame_index;

  if (!state.attached.has_value() &&
    state.cooldown_until_frame != 0U &&
    input.frame_index <= state.cooldown_until_frame)
  {
    state.tentative.reset();
    state.consecutive_hits = 0U;
    return ConfirmationResult{
      ConfirmationDecision::kCooldown, std::nullopt, 0U,
      "reattachment cooldown is active"};
  }

  if (!input.gates_passed || !input.assigned_lidar.has_value()) {
    state.tentative.reset();
    state.consecutive_hits = 0U;
    if (state.attached.has_value()) {
      ++state.misses;
      if (state.misses < config_.detach_after_misses) {
        return ConfirmationResult{
          ConfirmationDecision::kMatched, state.attached, 0U,
          "attachment retained through a bounded evidence miss"};
      }
      state.attached.reset();
      state.misses = 0U;
      const auto remaining = std::numeric_limits<std::uint64_t>::max() -
        input.frame_index;
      state.cooldown_until_frame = input.frame_index +
        std::min(config_.cooldown_frames, remaining);
    }
    return ConfirmationResult{
      ConfirmationDecision::kUnmatched, std::nullopt, 0U,
      "no gate-valid assignment"};
  }

  const bool same_as_attachment = state.attached.has_value() &&
    *state.attached == *input.assigned_lidar;
  const double effective_margin = input.best_score - input.second_score +
    (same_as_attachment ? config_.previous_association_hysteresis : 0.0);
  if (input.split_merge_hypothesis || effective_margin < config_.ambiguity_margin) {
    state.tentative.reset();
    state.consecutive_hits = 0U;
    return ConfirmationResult{
      ConfirmationDecision::kAmbiguous, state.attached, 0U,
      input.split_merge_hypothesis ?
      "split/merge hypothesis requires later evidence" :
      "top-two score margin is insufficient"};
  }

  if (same_as_attachment) {
    state.misses = 0U;
    return ConfirmationResult{
      ConfirmationDecision::kMatched, state.attached,
      config_.confirmation_frames, "existing attachment retained"};
  }

  if (state.attached.has_value()) {
    if (!consecutive_frame || !state.tentative.has_value() ||
      *state.tentative != *input.assigned_lidar)
    {
      state.tentative = input.assigned_lidar;
      state.consecutive_hits = 1U;
    } else {
      ++state.consecutive_hits;
    }
    if (state.consecutive_hits >= config_.confirmation_frames) {
      state.attached = state.tentative;
      state.tentative.reset();
      state.misses = 0U;
      return ConfirmationResult{
        ConfirmationDecision::kMatched, state.attached,
        state.consecutive_hits,
        "challenger replaced attachment after multi-frame confirmation"};
    }
    return ConfirmationResult{
      ConfirmationDecision::kTentative, state.attached,
      state.consecutive_hits,
      "challenger cannot replace an existing attachment immediately"};
  }

  if (!consecutive_frame || !state.tentative.has_value() ||
    *state.tentative != *input.assigned_lidar)
  {
    state.tentative = input.assigned_lidar;
    state.consecutive_hits = 1U;
  } else {
    ++state.consecutive_hits;
  }
  if (state.consecutive_hits < config_.confirmation_frames) {
    return ConfirmationResult{
      ConfirmationDecision::kTentative, std::nullopt,
      state.consecutive_hits, "new attachment is awaiting confirmation"};
  }

  state.attached = state.tentative;
  state.tentative.reset();
  state.misses = 0U;
  return ConfirmationResult{
    ConfirmationDecision::kMatched, state.attached,
    state.consecutive_hits, "new attachment confirmed"};
}

void AssociationConfirmation::reset() noexcept
{
  states_.clear();
}

}  // namespace track_robot_semantic_memory
