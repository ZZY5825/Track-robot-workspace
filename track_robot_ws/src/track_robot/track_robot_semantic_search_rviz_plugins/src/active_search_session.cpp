#include "track_robot_semantic_search_rviz_plugins/active_search_session.hpp"

namespace track_robot_semantic_search_rviz_plugins
{

std::optional<std::uint64_t> ActiveSearchSession::begin()
{
  if (active()) {
    return std::nullopt;
  }

  ++generation_;
  adopted_query_id_ = 0U;
  authorization_requested_ = false;
  state_ = ActiveSearchState::GOAL_PENDING;
  return generation_;
}

bool ActiveSearchSession::request_stop()
{
  if (!active() || state_ == ActiveSearchState::CANCEL_PENDING) {
    return false;
  }

  state_ = ActiveSearchState::CANCEL_PENDING;
  return true;
}

bool ActiveSearchSession::on_goal_response(const std::uint64_t generation, const bool accepted)
{
  if (generation != generation_ ||
    (state_ != ActiveSearchState::GOAL_PENDING && state_ != ActiveSearchState::CANCEL_PENDING))
  {
    return false;
  }

  if (state_ == ActiveSearchState::CANCEL_PENDING) {
    if (!accepted) {
      state_ = ActiveSearchState::IDLE;
      adopted_query_id_ = 0U;
      authorization_requested_ = false;
    }
    return true;
  }

  state_ = accepted ? ActiveSearchState::SEARCHING : ActiveSearchState::IDLE;
  return true;
}

ActiveSearchFeedbackDecision ActiveSearchSession::on_feedback(
  const std::uint64_t generation,
  const std::uint64_t query_id,
  const std::string & reason)
{
  ActiveSearchFeedbackDecision decision;
  if (generation != generation_ || query_id == 0U ||
    (state_ != ActiveSearchState::SEARCHING &&
    state_ != ActiveSearchState::AUTHORIZATION_PENDING &&
    state_ != ActiveSearchState::AUTHORIZED))
  {
    return decision;
  }

  decision.accepted = true;
  decision.query_id = query_id;
  if (adopted_query_id_ == 0U) {
    adopted_query_id_ = query_id;
    decision.adopt_query = true;
  }

  if (reason == "WAITING_FOR_AUTHORIZATION" && !authorization_requested_) {
    authorization_requested_ = true;
    state_ = ActiveSearchState::AUTHORIZATION_PENDING;
    decision.authorize_rotation = true;
  }
  return decision;
}

bool ActiveSearchSession::on_authorization_result(const std::uint64_t generation, const bool accepted)
{
  if (generation != generation_ || state_ != ActiveSearchState::AUTHORIZATION_PENDING) {
    return false;
  }

  state_ = accepted ? ActiveSearchState::AUTHORIZED : ActiveSearchState::SEARCHING;
  return true;
}

bool ActiveSearchSession::finish(const std::uint64_t generation)
{
  if (generation != generation_ || !active()) {
    return false;
  }

  state_ = ActiveSearchState::IDLE;
  adopted_query_id_ = 0U;
  authorization_requested_ = false;
  return true;
}

ActiveSearchState ActiveSearchSession::state() const
{
  return state_;
}

std::uint64_t ActiveSearchSession::generation() const
{
  return generation_;
}

bool ActiveSearchSession::active() const
{
  return state_ != ActiveSearchState::IDLE;
}

}  // namespace track_robot_semantic_search_rviz_plugins
