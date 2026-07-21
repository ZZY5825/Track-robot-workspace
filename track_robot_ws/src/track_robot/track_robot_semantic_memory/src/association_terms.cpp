#include "track_robot_semantic_memory/association_terms.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

namespace track_robot_semantic_memory
{
namespace
{

constexpr std::size_t kMaximumMetadataLength = 128U;
constexpr std::size_t kMaximumDescriptorDimension = 1024U;

double not_a_number() noexcept
{
  return std::numeric_limits<double>::quiet_NaN();
}

bool valid_term_parameters(
  double raw_value, double minimum, double maximum, double weight) noexcept
{
  return std::isfinite(raw_value) && std::isfinite(minimum) &&
         std::isfinite(maximum) && std::isfinite(weight) &&
         maximum > minimum && weight >= 0.0;
}

bool descriptor_shape_valid(
  const AppearanceDescriptor & descriptor,
  double normalization_tolerance,
  std::string * reason)
{
  if (descriptor.encoder_id.empty() ||
    descriptor.encoder_id.size() > kMaximumMetadataLength ||
    descriptor.checkpoint_id.empty() ||
    descriptor.checkpoint_id.size() > kMaximumMetadataLength)
  {
    *reason = "descriptor model identity is empty or unbounded";
    return false;
  }
  if (descriptor.dimension == 0U ||
    descriptor.dimension > kMaximumDescriptorDimension ||
    descriptor.values.size() != descriptor.dimension)
  {
    *reason = "descriptor dimension does not match its bounded values";
    return false;
  }
  if (!descriptor.l2_normalized) {
    *reason = "descriptor is not declared L2-normalized";
    return false;
  }
  if (!std::isfinite(normalization_tolerance) || normalization_tolerance < 0.0) {
    *reason = "normalization tolerance is invalid";
    return false;
  }

  double squared_norm = 0.0;
  for (const double value : descriptor.values) {
    if (!std::isfinite(value)) {
      *reason = "descriptor contains a non-finite value";
      return false;
    }
    squared_norm += value * value;
  }
  if (!std::isfinite(squared_norm) ||
    std::abs(std::sqrt(squared_norm) - 1.0) > normalization_tolerance)
  {
    *reason = "descriptor values fail L2-normalization verification";
    return false;
  }
  return true;
}

}  // namespace

AssociationTermResult invalid_association_term(
  std::string name, bool hard_gate, std::string reason)
{
  const double nan = not_a_number();
  return AssociationTermResult{
    std::move(name), false, hard_gate, false, nan, nan, 0.0, nan,
    std::move(reason)};
}

AssociationTermResult boolean_gate(
  std::string name, bool passed, std::string failure_reason)
{
  return AssociationTermResult{
    std::move(name), true, true, passed, passed ? 1.0 : 0.0,
    passed ? 1.0 : 0.0, 0.0, 0.0,
    passed ? std::string{} : std::move(failure_reason)};
}

AssociationTermResult maximum_gate(
  std::string name, double raw_value, double maximum)
{
  if (!std::isfinite(raw_value) || !std::isfinite(maximum) || maximum < 0.0) {
    return invalid_association_term(
      std::move(name), true, "gate value or threshold is non-finite/invalid");
  }
  const bool passed = raw_value >= 0.0 && raw_value <= maximum;
  return AssociationTermResult{
    std::move(name), true, true, passed, raw_value,
    maximum > 0.0 ? std::clamp(raw_value / maximum, 0.0, 1.0) : 0.0,
    0.0, 0.0, passed ? std::string{} : "maximum gate exceeded"};
}

AssociationTermResult range_gate(
  std::string name, double raw_value, double minimum, double maximum)
{
  if (!std::isfinite(raw_value) || !std::isfinite(minimum) ||
    !std::isfinite(maximum) || maximum < minimum)
  {
    return invalid_association_term(
      std::move(name), true, "gate value or range is non-finite/invalid");
  }
  const bool passed = raw_value >= minimum && raw_value <= maximum;
  return AssociationTermResult{
    std::move(name), true, true, passed, raw_value,
    maximum > minimum ? std::clamp(
      (raw_value - minimum) / (maximum - minimum), 0.0, 1.0) : 1.0,
    0.0, 0.0, passed ? std::string{} : "value outside allowed range"};
}

AssociationTermResult higher_is_better_term(
  std::string name, double raw_value, double minimum, double maximum,
  double weight)
{
  if (!valid_term_parameters(raw_value, minimum, maximum, weight)) {
    return invalid_association_term(
      std::move(name), false, "soft term value, range, or weight is invalid");
  }
  const double normalized = std::clamp(
    (raw_value - minimum) / (maximum - minimum), 0.0, 1.0);
  return AssociationTermResult{
    std::move(name), true, false, true, raw_value, normalized, weight,
    normalized * weight, {}};
}

AssociationTermResult lower_is_better_term(
  std::string name, double raw_value, double minimum, double maximum,
  double weight)
{
  if (!valid_term_parameters(raw_value, minimum, maximum, weight)) {
    return invalid_association_term(
      std::move(name), false, "soft term value, range, or weight is invalid");
  }
  const double normalized = 1.0 - std::clamp(
    (raw_value - minimum) / (maximum - minimum), 0.0, 1.0);
  return AssociationTermResult{
    std::move(name), true, false, true, raw_value, normalized, weight,
    normalized * weight, {}};
}

AssociationTermResult descriptor_compatibility_gate(
  const AppearanceDescriptor & first,
  const AppearanceDescriptor & second,
  double normalization_tolerance)
{
  std::string reason;
  if (!descriptor_shape_valid(first, normalization_tolerance, &reason) ||
    !descriptor_shape_valid(second, normalization_tolerance, &reason))
  {
    return boolean_gate("descriptor_compatibility", false, reason);
  }
  const bool compatible = first.encoder_id == second.encoder_id &&
    first.checkpoint_id == second.checkpoint_id &&
    first.version == second.version && first.dimension == second.dimension;
  return boolean_gate(
    "descriptor_compatibility", compatible,
    "encoder, checkpoint, version, or dimension mismatch");
}

AssociationTermResult descriptor_cosine_term(
  const AppearanceDescriptor & first,
  const AppearanceDescriptor & second,
  double weight,
  double normalization_tolerance)
{
  const auto gate = descriptor_compatibility_gate(
    first, second, normalization_tolerance);
  if (!gate.gate_passed) {
    return invalid_association_term("visual_cosine", false, gate.reason);
  }
  const double cosine = std::inner_product(
    first.values.begin(), first.values.end(), second.values.begin(), 0.0);
  return higher_is_better_term("visual_cosine", cosine, -1.0, 1.0, weight);
}

}  // namespace track_robot_semantic_memory
