#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "track_robot_semantic_memory/association_confirmation.hpp"

namespace track_robot_semantic_memory
{

struct RuntimeAssociationConfig
{
  double match_threshold{0.5};
  ConfirmationConfig confirmation;
  std::size_t maximum_visuals{64U};
  std::size_t maximum_lidars{256U};
  std::size_t maximum_pairs{1024U};

  void validate() const;
};

struct RuntimeVisualCandidate
{
  std::uint64_t visual_candidate_id{0U};
  std::optional<VisualAssociationKey> stable_key;
};

struct RuntimePairCandidate
{
  std::uint64_t visual_candidate_id{0U};
  LidarAssociationKey lidar;
  double score{0.0};
  bool gates_passed{false};

  friend bool operator==(
    const RuntimePairCandidate & left,
    const RuntimePairCandidate & right) noexcept
  {
    return left.visual_candidate_id == right.visual_candidate_id &&
           left.lidar == right.lidar &&
           left.score == right.score &&
           left.gates_passed == right.gates_passed;
  }
};

std::vector<RuntimePairCandidate> shortlist_visual_pairs(
  const std::vector<RuntimePairCandidate> & pairs,
  std::uint64_t visual_candidate_id,
  std::size_t maximum_lidar_candidates);

struct RuntimeAssociationFrame
{
  std::uint64_t frame_index{0U};
  std::vector<RuntimeVisualCandidate> visuals;
  std::vector<LidarAssociationKey> lidars;
  std::vector<RuntimePairCandidate> pairs;
};

struct RuntimeAssociationDecision
{
  std::uint64_t visual_candidate_id{0U};
  std::optional<LidarAssociationKey> assigned_lidar;
  std::optional<LidarAssociationKey> attached_lidar;
  ConfirmationDecision decision{ConfirmationDecision::kUnmatched};
  double best_score{0.0};
  double second_score{0.0};
  double margin{0.0};
  std::string reason;

  friend bool operator==(
    const RuntimeAssociationDecision & left,
    const RuntimeAssociationDecision & right) noexcept
  {
    return left.visual_candidate_id == right.visual_candidate_id &&
           left.assigned_lidar == right.assigned_lidar &&
           left.attached_lidar == right.attached_lidar &&
           left.decision == right.decision &&
           left.best_score == right.best_score &&
           left.second_score == right.second_score &&
           left.margin == right.margin && left.reason == right.reason;
  }
};

struct RuntimeAssociationResult
{
  std::vector<RuntimeAssociationDecision> decisions;
};

class RuntimeAssociationCoordinator
{
public:
  explicit RuntimeAssociationCoordinator(RuntimeAssociationConfig config);

  RuntimeAssociationResult process(const RuntimeAssociationFrame & frame);
  void reset() noexcept;

private:
  RuntimeAssociationConfig config_;
  AssociationConfirmation confirmation_;
};

}  // namespace track_robot_semantic_memory
