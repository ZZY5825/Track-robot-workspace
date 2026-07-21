#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace track_robot_semantic_memory
{

struct AssociationTermResult
{
  std::string name;
  bool valid{false};
  bool hard_gate{false};
  bool gate_passed{false};
  double raw_value{0.0};
  double normalized_value{0.0};
  double weight{0.0};
  double contribution{0.0};
  std::string reason;
};

struct AppearanceDescriptor
{
  std::string encoder_id;
  std::string checkpoint_id;
  std::uint32_t version{0U};
  std::uint16_t dimension{0U};
  bool l2_normalized{false};
  std::vector<double> values;
};

[[nodiscard]] AssociationTermResult invalid_association_term(
  std::string name, bool hard_gate, std::string reason);

[[nodiscard]] AssociationTermResult boolean_gate(
  std::string name, bool passed, std::string failure_reason = {});

[[nodiscard]] AssociationTermResult maximum_gate(
  std::string name, double raw_value, double maximum);

[[nodiscard]] AssociationTermResult range_gate(
  std::string name, double raw_value, double minimum, double maximum);

[[nodiscard]] AssociationTermResult higher_is_better_term(
  std::string name, double raw_value, double minimum, double maximum,
  double weight);

[[nodiscard]] AssociationTermResult lower_is_better_term(
  std::string name, double raw_value, double minimum, double maximum,
  double weight);

[[nodiscard]] AssociationTermResult descriptor_compatibility_gate(
  const AppearanceDescriptor & first,
  const AppearanceDescriptor & second,
  double normalization_tolerance);

[[nodiscard]] AssociationTermResult descriptor_cosine_term(
  const AppearanceDescriptor & first,
  const AppearanceDescriptor & second,
  double weight,
  double normalization_tolerance);

}  // namespace track_robot_semantic_memory
