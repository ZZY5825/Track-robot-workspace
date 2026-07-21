#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <optional>

#include "track_robot_semantic_memory/id_types.hpp"
#include "track_robot_semantic_memory/types.hpp"

namespace track_robot_semantic_memory
{

class MemoryDomainKey
{
public:
  MemoryDomainKey(
    MemoryMode mode,
    std::uint64_t localization_epoch_id,
    std::string canonical_frame_id)
  : mode_(mode),
    localization_epoch_id_(localization_epoch_id),
    canonical_frame_id_(std::move(canonical_frame_id))
  {
    if (canonical_frame_id_.empty() || canonical_frame_id_.size() > 128U) {
      throw std::invalid_argument(
              "canonical_frame_id must contain 1 to 128 characters");
    }
  }

  [[nodiscard]] MemoryMode mode() const noexcept {return mode_;}
  [[nodiscard]] std::uint64_t localization_epoch_id() const noexcept
  {
    return localization_epoch_id_;
  }
  [[nodiscard]] const std::string & canonical_frame_id() const noexcept
  {
    return canonical_frame_id_;
  }

  friend bool operator==(
    const MemoryDomainKey & left, const MemoryDomainKey & right) noexcept
  {
    return left.mode_ == right.mode_ &&
           left.localization_epoch_id_ == right.localization_epoch_id_ &&
           left.canonical_frame_id_ == right.canonical_frame_id_;
  }

  friend bool operator!=(
    const MemoryDomainKey & left, const MemoryDomainKey & right) noexcept
  {
    return !(left == right);
  }

private:
  MemoryMode mode_;
  std::uint64_t localization_epoch_id_;
  std::string canonical_frame_id_;
};

struct DomainTransition
{
  std::uint64_t memory_epoch_id{0U};
  bool changed{false};
};

class MemoryDomainTracker
{
public:
  explicit MemoryDomainTracker(std::uint64_t initial_memory_epoch_id);

  DomainTransition update(const MemoryDomainKey & domain);
  DomainTransition advance_epoch();
  [[nodiscard]] bool accepts(const GlobalObjectKey & key) const noexcept;
  [[nodiscard]] std::uint64_t memory_epoch_id() const noexcept;
  [[nodiscard]] const std::optional<MemoryDomainKey> & domain() const noexcept;

private:
  static std::uint64_t next_epoch(std::uint64_t current) noexcept;

  std::uint64_t memory_epoch_id_;
  std::optional<MemoryDomainKey> domain_;
};

}  // namespace track_robot_semantic_memory
