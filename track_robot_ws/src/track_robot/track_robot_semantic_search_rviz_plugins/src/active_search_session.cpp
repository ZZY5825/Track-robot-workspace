#include "track_robot_semantic_search_rviz_plugins/active_search_session.hpp"

namespace track_robot_semantic_search_rviz_plugins
{

std::string active_search_feedback_status(const std::string & reason)
{
  if (reason == "PASSIVE_OBSERVATION") {
    return "passive observation";
  }
  if (reason == "WAITING_FOR_AUTHORIZATION") {
    return "starting bounded rotation";
  }
  if (reason.find("ROTATING") == 0U) {
    return "rotating in place";
  }
  if (reason.find("SETTLING") == 0U) {
    return "settling";
  }
  if (reason.find("OBSERVING") == 0U) {
    return "observing";
  }
  if (reason.empty()) {
    return "searching";
  }
  return reason.substr(0U, 128U);
}

std::optional<std::uint64_t> ActiveSearchSession::begin()
{
  if (active()) {
    return std::nullopt;
  }

  ++generation_;
  adopted_query_id_ = 0U;
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
    state_ != ActiveSearchState::SEARCHING)
  {
    return decision;
  }
  if (adopted_query_id_ != 0U && adopted_query_id_ != query_id) {
    return decision;
  }

  decision.accepted = true;
  decision.query_id = query_id;
  if (adopted_query_id_ == 0U) {
    adopted_query_id_ = query_id;
    decision.adopt_query = true;
  }

  (void)reason;
  return decision;
}

bool ActiveSearchSession::finish(const std::uint64_t generation)
{
  if (generation != generation_ || !active()) {
    return false;
  }

  state_ = ActiveSearchState::IDLE;
  adopted_query_id_ = 0U;
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

bool ActiveSearchSession::manual_query_allowed() const
{
  return !active();
}

}  // namespace track_robot_semantic_search_rviz_plugins
