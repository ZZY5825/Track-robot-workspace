#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "track_robot_semantic_memory/association_terms.hpp"

namespace track_robot_semantic_memory
{

struct AppearanceMemoryConfig
{
  std::size_t maximum_prototypes{4U};
  double minimum_quality{0.5};
  double new_prototype_similarity_threshold{0.8};
  double normalization_tolerance{1e-4};

  void validate() const;
};

struct AppearanceObservation
{
  AppearanceDescriptor descriptor;
  double quality{0.0};
  bool confirmed{false};
  bool ambiguous{false};
  bool prediction_only{false};
};

struct AppearancePrototype
{
  AppearanceDescriptor descriptor;
  double accumulated_quality_weight{0.0};
  double best_quality{0.0};
  std::uint32_t update_count{0U};
};

struct BestAppearanceView
{
  AppearanceDescriptor descriptor;
  double quality{0.0};
};

enum class AppearanceUpdateDecision : std::uint8_t
{
  kRejected = 0U,
  kPrototypeCreated = 1U,
  kPrototypeUpdated = 2U,
};

struct AppearanceUpdateResult
{
  AppearanceUpdateDecision decision{AppearanceUpdateDecision::kRejected};
  std::size_t prototype_index{0U};
  double nearest_similarity{0.0};
  std::string reason;
};

struct AppearanceMergeResult
{
  std::size_t accepted{0U};
  std::size_t rejected{0U};
};

class AppearanceMemory
{
public:
  explicit AppearanceMemory(AppearanceMemoryConfig config);

  AppearanceUpdateResult update(const AppearanceObservation & observation);
  AppearanceMergeResult merge_from(const AppearanceMemory & source);

  [[nodiscard]] const std::vector<AppearancePrototype> & prototypes() const noexcept;
  [[nodiscard]] const std::optional<BestAppearanceView> & best_view() const noexcept;
  [[nodiscard]] std::string summary_id() const;
  void clear() noexcept;

private:
  AppearanceMemoryConfig config_;
  std::vector<AppearancePrototype> prototypes_;
  std::optional<BestAppearanceView> best_view_;
};

}  // namespace track_robot_semantic_memory
