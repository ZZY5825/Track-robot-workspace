#pragma once

#include <cstdint>

#include "track_robot_semantic_memory/types.hpp"

namespace track_robot_semantic_memory
{

struct LifecyclePolicyConfig
{
  std::uint32_t confirmation_hits{3U};
  std::int64_t stale_after_ns{500000000};
  std::int64_t lost_after_ns{2000000000};
  std::int64_t archive_after_ns{10000000000};

  void validate() const;
};

class LifecyclePolicy
{
public:
  explicit LifecyclePolicy(LifecyclePolicyConfig config);

  [[nodiscard]] LifecycleState evaluate(
    LifecycleState current,
    std::uint32_t compatible_hit_count,
    std::int64_t time_since_support_ns) const;
  [[nodiscard]] EvidenceFreshness freshness(
    std::int64_t time_since_support_ns) const;

private:
  LifecyclePolicyConfig config_;
};

}  // namespace track_robot_semantic_memory
