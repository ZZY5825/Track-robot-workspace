#pragma once

#include <cstdint>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <tuple>

namespace track_robot_semantic_memory
{

enum class VisualAssociationKind : std::uint8_t
{
  kCameraTrack = 0U,
  kUpstreamProposal = 1U,
};

struct VisualAssociationKey
{
  VisualAssociationKind kind{VisualAssociationKind::kCameraTrack};
  std::uint64_t producer_epoch_id{0U};
  std::uint64_t local_id{0U};

  [[nodiscard]] bool valid() const noexcept
  {
    return producer_epoch_id != 0U;
  }

  friend bool operator==(
    const VisualAssociationKey & left,
    const VisualAssociationKey & right) noexcept
  {
    return left.kind == right.kind &&
           left.producer_epoch_id == right.producer_epoch_id &&
           left.local_id == right.local_id;
  }

  friend bool operator!=(
    const VisualAssociationKey & left,
    const VisualAssociationKey & right) noexcept
  {
    return !(left == right);
  }

  friend bool operator<(
    const VisualAssociationKey & left,
    const VisualAssociationKey & right) noexcept
  {
    return std::tie(left.kind, left.producer_epoch_id, left.local_id) <
           std::tie(right.kind, right.producer_epoch_id, right.local_id);
  }
};

struct LidarAssociationKey
{
  std::uint64_t source_epoch_id{0U};
  std::int64_t tracklet_id{-1};

  friend bool operator==(
    const LidarAssociationKey & left,
    const LidarAssociationKey & right) noexcept
  {
    return left.source_epoch_id == right.source_epoch_id &&
           left.tracklet_id == right.tracklet_id;
  }

  friend bool operator!=(
    const LidarAssociationKey & left,
    const LidarAssociationKey & right) noexcept
  {
    return !(left == right);
  }
};

struct ConfirmationConfig
{
  std::uint32_t confirmation_frames{3U};
  std::uint32_t detach_after_misses{2U};
  double ambiguity_margin{0.1};
  double previous_association_hysteresis{0.05};
  std::uint64_t cooldown_frames{2U};
  std::size_t maximum_states{64U};
};

struct ConfirmationInput
{
  VisualAssociationKey visual_key;
  std::optional<LidarAssociationKey> assigned_lidar;
  double best_score{0.0};
  double second_score{0.0};
  bool gates_passed{false};
  bool split_merge_hypothesis{false};
  std::uint64_t frame_index{0U};
};

enum class ConfirmationDecision : std::uint8_t
{
  kUnmatched = 0U,
  kTentative = 1U,
  kMatched = 2U,
  kAmbiguous = 3U,
  kCooldown = 4U,
};

struct ConfirmationResult
{
  ConfirmationDecision decision{ConfirmationDecision::kUnmatched};
  std::optional<LidarAssociationKey> attached_lidar;
  std::uint32_t consecutive_hits{0U};
  std::string reason;
};

class AssociationConfirmation
{
public:
  explicit AssociationConfirmation(ConfirmationConfig config);

  [[nodiscard]] ConfirmationResult update(
    const ConfirmationInput & input);

  void reset() noexcept;

private:
  struct CandidateState
  {
    bool seen{false};
    std::uint64_t last_frame{0U};
    std::optional<LidarAssociationKey> attached;
    std::optional<LidarAssociationKey> tentative;
    std::uint32_t consecutive_hits{0U};
    std::uint32_t misses{0U};
    std::uint64_t cooldown_until_frame{0U};
  };

  ConfirmationConfig config_;
  std::map<VisualAssociationKey, CandidateState> states_;
};

}  // namespace track_robot_semantic_memory
