#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace track_robot_semantic_memory
{

using OptionalCostMatrix = std::vector<std::vector<std::optional<double>>>;

struct AssignmentMatch
{
  std::uint64_t row_id{0U};
  std::uint64_t column_id{0U};
  double cost{0.0};

  friend bool operator==(
    const AssignmentMatch & left, const AssignmentMatch & right) noexcept
  {
    return left.row_id == right.row_id &&
           left.column_id == right.column_id && left.cost == right.cost;
  }
};

struct GlobalAssignment
{
  std::vector<AssignmentMatch> matches;
  std::vector<std::uint64_t> unmatched_rows;
  std::vector<std::uint64_t> unmatched_columns;
  double total_cost{0.0};
};

[[nodiscard]] GlobalAssignment hungarian_assignment(
  const std::vector<std::uint64_t> & row_ids,
  const std::vector<std::uint64_t> & column_ids,
  const OptionalCostMatrix & costs,
  double unmatched_cost);

}  // namespace track_robot_semantic_memory
