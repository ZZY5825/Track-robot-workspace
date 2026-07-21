#include "track_robot_semantic_memory/hungarian_assignment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <utility>

namespace track_robot_semantic_memory
{
namespace
{

std::vector<std::size_t> sorted_indices(
  const std::vector<std::uint64_t> & ids)
{
  std::vector<std::size_t> indices(ids.size());
  std::iota(indices.begin(), indices.end(), 0U);
  std::stable_sort(
    indices.begin(), indices.end(),
    [&ids](std::size_t left, std::size_t right) {
      return ids[left] < ids[right];
    });
  return indices;
}

void validate_inputs(
  const std::vector<std::uint64_t> & row_ids,
  const std::vector<std::uint64_t> & column_ids,
  const OptionalCostMatrix & costs,
  double unmatched_cost)
{
  if (!std::isfinite(unmatched_cost) || unmatched_cost < 0.0 ||
    unmatched_cost > 1.0e12)
  {
    throw std::invalid_argument("unmatched cost must be finite and nonnegative");
  }
  if (std::set<std::uint64_t>(row_ids.begin(), row_ids.end()).size() !=
    row_ids.size() ||
    std::set<std::uint64_t>(column_ids.begin(), column_ids.end()).size() !=
    column_ids.size())
  {
    throw std::invalid_argument("assignment IDs must be unique");
  }
  if (costs.size() != row_ids.size()) {
    throw std::invalid_argument("cost matrix row count does not match row IDs");
  }
  for (const auto & row : costs) {
    if (row.size() != column_ids.size()) {
      throw std::invalid_argument(
              "cost matrix column count does not match column IDs");
    }
    for (const auto & cost : row) {
      if (cost.has_value() && (!std::isfinite(*cost) || *cost < 0.0)) {
        throw std::invalid_argument(
                "available assignment costs must be finite and nonnegative");
      }
    }
  }
}

}  // namespace

GlobalAssignment hungarian_assignment(
  const std::vector<std::uint64_t> & row_ids,
  const std::vector<std::uint64_t> & column_ids,
  const OptionalCostMatrix & costs,
  double unmatched_cost)
{
  validate_inputs(row_ids, column_ids, costs, unmatched_cost);
  GlobalAssignment result;
  if (row_ids.empty()) {
    result.unmatched_columns = column_ids;
    std::sort(result.unmatched_columns.begin(), result.unmatched_columns.end());
    return result;
  }

  const auto row_order = sorted_indices(row_ids);
  const auto column_order = sorted_indices(column_ids);
  const std::size_t row_count = row_ids.size();
  const std::size_t real_column_count = column_ids.size();
  const std::size_t column_count = real_column_count + row_count;
  const double forbidden_cost = std::max(1.0e15, unmatched_cost * 1.0e3 + 1.0);

  std::vector<std::vector<double>> matrix(
    row_count + 1U, std::vector<double>(column_count + 1U, unmatched_cost));
  for (std::size_t canonical_row = 0U; canonical_row < row_count; ++canonical_row) {
    const std::size_t input_row = row_order[canonical_row];
    for (std::size_t canonical_column = 0U;
      canonical_column < real_column_count; ++canonical_column)
    {
      const auto & cost = costs[input_row][column_order[canonical_column]];
      matrix[canonical_row + 1U][canonical_column + 1U] =
        cost.has_value() ? *cost : forbidden_cost;
    }
  }

  std::vector<double> row_potential(row_count + 1U, 0.0);
  std::vector<double> column_potential(column_count + 1U, 0.0);
  std::vector<std::size_t> assigned_row(column_count + 1U, 0U);
  std::vector<std::size_t> predecessor(column_count + 1U, 0U);

  for (std::size_t row = 1U; row <= row_count; ++row) {
    assigned_row[0] = row;
    std::size_t current_column = 0U;
    std::vector<double> minimum_reduced_cost(
      column_count + 1U, std::numeric_limits<double>::infinity());
    std::vector<bool> used(column_count + 1U, false);
    do {
      used[current_column] = true;
      const std::size_t current_row = assigned_row[current_column];
      double delta = std::numeric_limits<double>::infinity();
      std::size_t next_column = 0U;
      for (std::size_t column = 1U; column <= column_count; ++column) {
        if (used[column]) {
          continue;
        }
        const double reduced = matrix[current_row][column] -
          row_potential[current_row] - column_potential[column];
        if (reduced < minimum_reduced_cost[column]) {
          minimum_reduced_cost[column] = reduced;
          predecessor[column] = current_column;
        }
        if (minimum_reduced_cost[column] < delta ||
          (minimum_reduced_cost[column] == delta && column < next_column))
        {
          delta = minimum_reduced_cost[column];
          next_column = column;
        }
      }
      if (!std::isfinite(delta)) {
        throw std::logic_error("assignment unexpectedly became infeasible");
      }
      for (std::size_t column = 0U; column <= column_count; ++column) {
        if (used[column]) {
          row_potential[assigned_row[column]] += delta;
          column_potential[column] -= delta;
        } else {
          minimum_reduced_cost[column] -= delta;
        }
      }
      current_column = next_column;
    } while (assigned_row[current_column] != 0U);

    do {
      const std::size_t previous_column = predecessor[current_column];
      assigned_row[current_column] = assigned_row[previous_column];
      current_column = previous_column;
    } while (current_column != 0U);
  }

  std::vector<std::size_t> column_for_row(row_count + 1U, 0U);
  for (std::size_t column = 1U; column <= column_count; ++column) {
    if (assigned_row[column] != 0U) {
      column_for_row[assigned_row[column]] = column;
    }
  }

  std::set<std::uint64_t> matched_columns;
  for (std::size_t canonical_row = 1U; canonical_row <= row_count; ++canonical_row) {
    const std::uint64_t row_id = row_ids[row_order[canonical_row - 1U]];
    const std::size_t column = column_for_row[canonical_row];
    if (column >= 1U && column <= real_column_count) {
      const std::size_t input_column = column_order[column - 1U];
      const auto & cost = costs[row_order[canonical_row - 1U]][input_column];
      if (cost.has_value() && *cost < unmatched_cost) {
        const std::uint64_t column_id = column_ids[input_column];
        result.matches.push_back(AssignmentMatch{row_id, column_id, *cost});
        matched_columns.insert(column_id);
        result.total_cost += *cost;
        continue;
      }
    }
    result.unmatched_rows.push_back(row_id);
    result.total_cost += unmatched_cost;
  }

  for (const auto column_id : column_ids) {
    if (matched_columns.count(column_id) == 0U) {
      result.unmatched_columns.push_back(column_id);
    }
  }
  std::sort(result.matches.begin(), result.matches.end(),
    [](const AssignmentMatch & left, const AssignmentMatch & right) {
      return left.row_id < right.row_id;
    });
  std::sort(result.unmatched_rows.begin(), result.unmatched_rows.end());
  std::sort(result.unmatched_columns.begin(), result.unmatched_columns.end());
  return result;
}

}  // namespace track_robot_semantic_memory
