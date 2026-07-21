#include "track_robot_semantic_memory/memory_domain.hpp"

#include <limits>
#include <stdexcept>

namespace track_robot_semantic_memory
{

MemoryDomainTracker::MemoryDomainTracker(std::uint64_t initial_memory_epoch_id)
: memory_epoch_id_(initial_memory_epoch_id)
{
  if (initial_memory_epoch_id == 0U) {
    throw std::invalid_argument("initial memory epoch must be non-zero");
  }
}

DomainTransition MemoryDomainTracker::update(const MemoryDomainKey & domain)
{
  if (!domain_.has_value()) {
    domain_ = domain;
    return {memory_epoch_id_, true};
  }
  if (*domain_ == domain) {
    return {memory_epoch_id_, false};
  }
  memory_epoch_id_ = next_epoch(memory_epoch_id_);
  domain_ = domain;
  return {memory_epoch_id_, true};
}

DomainTransition MemoryDomainTracker::advance_epoch()
{
  memory_epoch_id_ = next_epoch(memory_epoch_id_);
  return {memory_epoch_id_, true};
}

bool MemoryDomainTracker::accepts(const GlobalObjectKey & key) const noexcept
{
  return domain_.has_value() && key.valid() && key.memory_epoch_id == memory_epoch_id_;
}

std::uint64_t MemoryDomainTracker::memory_epoch_id() const noexcept
{
  return memory_epoch_id_;
}

const std::optional<MemoryDomainKey> & MemoryDomainTracker::domain() const noexcept
{
  return domain_;
}

std::uint64_t MemoryDomainTracker::next_epoch(std::uint64_t current) noexcept
{
  return current == std::numeric_limits<std::uint64_t>::max() ? 1U : current + 1U;
}

}  // namespace track_robot_semantic_memory
