#pragma once

#include <cstdint>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "track_robot_semantic_memory/association_confirmation.hpp"
#include "track_robot_semantic_memory/id_types.hpp"
#include "track_robot_semantic_memory/types.hpp"

namespace track_robot_semantic_memory
{

struct ReidentificationConfig
{
  double maximum_spatial_distance_m{3.0};
  std::int64_t maximum_age_ns{5'000'000'000LL};
  double minimum_appearance_similarity{0.75};
  double minimum_combined_score{0.70};
  std::uint32_t confirmation_frames{3U};
  double ambiguity_margin{0.05};
  std::size_t maximum_candidates{64U};
  std::size_t maximum_lost_targets{256U};
  std::size_t maximum_pairs{1024U};

  void validate() const;
};

struct ReidentificationEvidence
{
  GlobalObjectKey object_key;
  LifecycleState lifecycle{LifecycleState::kLost};
  std::uint64_t candidate_id{0U};
  std::uint64_t frame_index{0U};
  bool domain_compatible{false};
  std::int64_t age_ns{0};
  double spatial_distance_m{0.0};
  double appearance_similarity{0.0};
  double geometry_similarity{0.0};
  double semantic_similarity{0.0};
};

enum class ReidentificationDecision : std::uint8_t
{
  kRejectedGate = 0U,
  kRejectedScore = 1U,
  kTentative = 2U,
  kConfirmed = 3U,
  kArchivedBlocked = 4U,
};

struct ReidentificationResult
{
  ReidentificationDecision decision{ReidentificationDecision::kRejectedGate};
  GlobalObjectKey object_key;
  std::uint64_t candidate_id{0U};
  std::uint32_t consecutive_hits{0U};
  double combined_score{0.0};
  bool event_emitted{false};
  std::string reason;
};

class ReidentificationTracker
{
public:
  explicit ReidentificationTracker(ReidentificationConfig config);

  ReidentificationResult update(const ReidentificationEvidence & evidence);
  void reset() noexcept;

private:
  struct State
  {
    bool seen{false};
    std::uint64_t last_frame{0U};
    std::uint64_t candidate_id{0U};
    std::uint32_t consecutive_hits{0U};
    bool confirmed{false};
  };

  ReidentificationConfig config_;
  std::map<GlobalObjectKey, State> states_;
};

struct RuntimeReidentificationPair
{
  GlobalObjectKey lost_key;
  GlobalObjectKey candidate_key;
  ProducerObjectKey expected_candidate_lidar_key;
  std::optional<VisualAssociationKey> expected_candidate_visual_key;
  LifecycleState lost_lifecycle{LifecycleState::kLost};
  bool domain_compatible{false};
  std::int64_t age_ns{0};
  double spatial_distance_m{0.0};
  double appearance_similarity{0.0};
  double geometry_similarity{0.0};
  double semantic_similarity{0.0};
};

struct RuntimeReidentificationFrame
{
  std::uint64_t frame_index{0U};
  std::uint64_t memory_epoch_id{0U};
  std::vector<GlobalObjectKey> candidates;
  std::vector<GlobalObjectKey> lost_targets;
  std::vector<RuntimeReidentificationPair> pairs;
};

struct RuntimeReidentificationDecision
{
  GlobalObjectKey lost_key;
  GlobalObjectKey candidate_key;
  ProducerObjectKey expected_candidate_lidar_key;
  std::optional<VisualAssociationKey> expected_candidate_visual_key;
  ReidentificationDecision decision{ReidentificationDecision::kRejectedGate};
  std::uint32_t consecutive_hits{0U};
  double combined_score{0.0};
  std::string reason;

  friend bool operator==(
    const RuntimeReidentificationDecision & left,
    const RuntimeReidentificationDecision & right) noexcept
  {
    return left.lost_key == right.lost_key &&
           left.candidate_key == right.candidate_key &&
           left.expected_candidate_lidar_key == right.expected_candidate_lidar_key &&
           left.expected_candidate_visual_key == right.expected_candidate_visual_key &&
           left.decision == right.decision &&
           left.consecutive_hits == right.consecutive_hits &&
           left.combined_score == right.combined_score &&
           left.reason == right.reason;
  }
};

struct RuntimeReidentificationResult
{
  std::vector<RuntimeReidentificationDecision> decisions;
};

class RuntimeReidentificationCoordinator
{
public:
  explicit RuntimeReidentificationCoordinator(ReidentificationConfig config);

  RuntimeReidentificationResult process(
    const RuntimeReidentificationFrame & frame);
  void reset() noexcept;
  [[nodiscard]] std::size_t confirmation_state_count() const noexcept;

private:
  struct ConfirmationState
  {
    std::uint64_t last_frame{0U};
    std::uint32_t consecutive_hits{0U};
    bool confirmed{false};
  };

  ReidentificationConfig config_;
  std::uint64_t last_frame_index_{0U};
  std::map<std::pair<GlobalObjectKey, GlobalObjectKey>, ConfirmationState>
    confirmation_states_;
};

}  // namespace track_robot_semantic_memory
