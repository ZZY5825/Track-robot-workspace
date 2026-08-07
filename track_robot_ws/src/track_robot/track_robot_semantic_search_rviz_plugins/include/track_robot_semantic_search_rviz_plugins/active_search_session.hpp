#ifndef TRACK_ROBOT_SEMANTIC_SEARCH_RVIZ_PLUGINS__ACTIVE_SEARCH_SESSION_HPP_
#define TRACK_ROBOT_SEMANTIC_SEARCH_RVIZ_PLUGINS__ACTIVE_SEARCH_SESSION_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace track_robot_semantic_search_rviz_plugins
{

enum class ActiveSearchState
{
  IDLE,
  GOAL_PENDING,
  SEARCHING,
  CANCEL_PENDING,
};

struct ActiveSearchFeedbackDecision
{
  bool accepted{false};
  bool adopt_query{false};
  std::uint64_t query_id{0U};
};

std::string active_search_feedback_status(const std::string & reason);

class ActiveSearchSession
{
public:
  std::optional<std::uint64_t> begin();
  bool request_stop();
  bool on_goal_response(std::uint64_t generation, bool accepted);
  ActiveSearchFeedbackDecision on_feedback(
    std::uint64_t generation,
    std::uint64_t query_id,
    const std::string & reason);
  bool finish(std::uint64_t generation);
  [[nodiscard]] ActiveSearchState state() const;
  [[nodiscard]] std::uint64_t generation() const;
  [[nodiscard]] bool active() const;
  [[nodiscard]] bool manual_query_allowed() const;

private:
  ActiveSearchState state_{ActiveSearchState::IDLE};
  std::uint64_t generation_{0U};
  std::uint64_t adopted_query_id_{0U};
};

}  // namespace track_robot_semantic_search_rviz_plugins

#endif  // TRACK_ROBOT_SEMANTIC_SEARCH_RVIZ_PLUGINS__ACTIVE_SEARCH_SESSION_HPP_
