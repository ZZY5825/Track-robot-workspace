#include "track_robot_semantic_memory/lifecycle_policy.hpp"

#include <stdexcept>

namespace track_robot_semantic_memory
{

void LifecyclePolicyConfig::validate() const
{
  if (confirmation_hits == 0U) {
    throw std::invalid_argument("confirmation_hits must be positive");
  }
  if (stale_after_ns < 0 || lost_after_ns <= stale_after_ns ||
    archive_after_ns <= lost_after_ns)
  {
    throw std::invalid_argument("lifecycle time bounds must be strictly ordered");
  }
}

LifecyclePolicy::LifecyclePolicy(LifecyclePolicyConfig config)
: config_(config)
{
  config_.validate();
}

LifecycleState LifecyclePolicy::evaluate(
  LifecycleState current,
  std::uint32_t compatible_hit_count,
  std::int64_t time_since_support_ns) const
{
  if (time_since_support_ns < 0) {
    throw std::invalid_argument("time since support must be non-negative");
  }
  if (current == LifecycleState::kArchived) {
    return current;
  }
  if (time_since_support_ns == 0 &&
    (current == LifecycleState::kStale || current == LifecycleState::kLost))
  {
    return compatible_hit_count >= config_.confirmation_hits ?
      LifecycleState::kConfirmed : LifecycleState::kTentative;
  }
  if (time_since_support_ns > config_.archive_after_ns) {
    return LifecycleState::kArchived;
  }
  if (time_since_support_ns > config_.lost_after_ns) {
    return LifecycleState::kLost;
  }
  if (time_since_support_ns > config_.stale_after_ns) {
    return LifecycleState::kStale;
  }
  if (current == LifecycleState::kTentative &&
    compatible_hit_count >= config_.confirmation_hits)
  {
    return LifecycleState::kConfirmed;
  }
  return current;
}

EvidenceFreshness LifecyclePolicy::freshness(
  std::int64_t time_since_support_ns) const
{
  if (time_since_support_ns < 0) {
    throw std::invalid_argument("time since support must be non-negative");
  }
  if (time_since_support_ns == 0) {
    return EvidenceFreshness::kObserved;
  }
  if (time_since_support_ns <= config_.lost_after_ns) {
    return EvidenceFreshness::kPredicted;
  }
  return EvidenceFreshness::kUnsupported;
}

}  // namespace track_robot_semantic_memory
