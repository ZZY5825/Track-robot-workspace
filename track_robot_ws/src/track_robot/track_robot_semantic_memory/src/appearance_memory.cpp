#include "track_robot_semantic_memory/appearance_memory.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace track_robot_semantic_memory
{
namespace
{

double cosine(
  const AppearanceDescriptor & first,
  const AppearanceDescriptor & second) noexcept
{
  return std::inner_product(
    first.values.begin(), first.values.end(), second.values.begin(), 0.0);
}

bool incrementable(std::uint32_t value) noexcept
{
  return value != std::numeric_limits<std::uint32_t>::max();
}

class Fnv1a64
{
public:
  void byte(std::uint8_t value) noexcept
  {
    value_ ^= value;
    value_ *= 1099511628211ULL;
  }

  template<typename UIntT>
  void integer(UIntT value) noexcept
  {
    for (std::size_t index = sizeof(UIntT); index > 0U; --index) {
      byte(static_cast<std::uint8_t>(
          (value >> ((index - 1U) * 8U)) & static_cast<UIntT>(0xffU)));
    }
  }

  void text(const std::string & value) noexcept
  {
    integer<std::uint32_t>(static_cast<std::uint32_t>(value.size()));
    for (const unsigned char character : value) {
      byte(character);
    }
  }

  void floating(double value) noexcept
  {
    std::uint64_t bits = 0U;
    static_assert(sizeof(bits) == sizeof(value), "unexpected double width");
    std::memcpy(&bits, &value, sizeof(bits));
    integer(bits);
  }

  [[nodiscard]] std::uint64_t value() const noexcept {return value_;}

private:
  std::uint64_t value_{14695981039346656037ULL};
};

}  // namespace

void AppearanceMemoryConfig::validate() const
{
  if (maximum_prototypes == 0U || maximum_prototypes > 4U ||
    !std::isfinite(minimum_quality) || minimum_quality < 0.0 ||
    minimum_quality > 1.0 ||
    !std::isfinite(new_prototype_similarity_threshold) ||
    new_prototype_similarity_threshold < -1.0 ||
    new_prototype_similarity_threshold > 1.0 ||
    !std::isfinite(normalization_tolerance) || normalization_tolerance < 0.0 ||
    normalization_tolerance >= 1.0)
  {
    throw std::invalid_argument("appearance memory config is invalid");
  }
}

AppearanceMemory::AppearanceMemory(AppearanceMemoryConfig config)
: config_(std::move(config))
{
  config_.validate();
  prototypes_.reserve(config_.maximum_prototypes);
}

AppearanceUpdateResult AppearanceMemory::update(
  const AppearanceObservation & observation)
{
  if (!observation.confirmed || observation.ambiguous ||
    observation.prediction_only)
  {
    return {AppearanceUpdateDecision::kRejected, 0U, 0.0,
      "appearance evidence is not confirmed and observable"};
  }
  if (!std::isfinite(observation.quality) ||
    observation.quality < config_.minimum_quality || observation.quality > 1.0)
  {
    return {AppearanceUpdateDecision::kRejected, 0U, 0.0,
      "appearance quality is below threshold or invalid"};
  }
  const auto self_gate = descriptor_compatibility_gate(
    observation.descriptor, observation.descriptor,
    config_.normalization_tolerance);
  if (!self_gate.gate_passed) {
    return {AppearanceUpdateDecision::kRejected, 0U, 0.0, self_gate.reason};
  }
  if (!prototypes_.empty()) {
    const auto compatibility = descriptor_compatibility_gate(
      prototypes_.front().descriptor, observation.descriptor,
      config_.normalization_tolerance);
    if (!compatibility.gate_passed) {
      return {AppearanceUpdateDecision::kRejected, 0U, 0.0,
        compatibility.reason};
    }
  }

  if (prototypes_.empty()) {
    prototypes_.push_back(AppearancePrototype{
      observation.descriptor, observation.quality, observation.quality, 1U});
    best_view_ = BestAppearanceView{observation.descriptor, observation.quality};
    return {AppearanceUpdateDecision::kPrototypeCreated, 0U, 1.0,
      "first compatible prototype created"};
  }

  std::size_t nearest_index = 0U;
  double nearest_similarity = -std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < prototypes_.size(); ++index) {
    const double similarity = cosine(
      prototypes_[index].descriptor, observation.descriptor);
    if (similarity > nearest_similarity) {
      nearest_similarity = similarity;
      nearest_index = index;
    }
  }
  if (nearest_similarity < config_.new_prototype_similarity_threshold &&
    prototypes_.size() < config_.maximum_prototypes)
  {
    prototypes_.push_back(AppearancePrototype{
      observation.descriptor, observation.quality, observation.quality, 1U});
    if (!best_view_.has_value() || observation.quality > best_view_->quality) {
      best_view_ = BestAppearanceView{observation.descriptor, observation.quality};
    }
    return {AppearanceUpdateDecision::kPrototypeCreated,
      prototypes_.size() - 1U, nearest_similarity,
      "diverse high-quality view created a bounded prototype"};
  }

  auto & prototype = prototypes_[nearest_index];
  const double previous_weight = prototype.accumulated_quality_weight;
  const double total_weight = previous_weight + observation.quality;
  std::vector<double> blended_values(prototype.descriptor.values.size(), 0.0);
  double squared_norm = 0.0;
  for (std::size_t index = 0U; index < prototype.descriptor.values.size(); ++index) {
    const double blended = (
      previous_weight * prototype.descriptor.values[index] +
      observation.quality * observation.descriptor.values[index]) / total_weight;
    blended_values[index] = blended;
    squared_norm += blended * blended;
  }
  const double norm = std::sqrt(squared_norm);
  if (!std::isfinite(norm) || norm <= 1e-12) {
    return {AppearanceUpdateDecision::kRejected, nearest_index,
      nearest_similarity, "appearance EMA would produce a zero/non-finite norm"};
  }
  for (auto & value : blended_values) {
    value /= norm;
  }
  prototype.descriptor.values = std::move(blended_values);
  prototype.accumulated_quality_weight = std::min(total_weight, 1000.0);
  prototype.best_quality = std::max(prototype.best_quality, observation.quality);
  if (incrementable(prototype.update_count)) {
    ++prototype.update_count;
  }
  if (!best_view_.has_value() || observation.quality > best_view_->quality) {
    best_view_ = BestAppearanceView{observation.descriptor, observation.quality};
  }
  return {AppearanceUpdateDecision::kPrototypeUpdated, nearest_index,
    nearest_similarity, "nearest compatible prototype updated by normalized EMA"};
}

AppearanceMergeResult AppearanceMemory::merge_from(
  const AppearanceMemory & source)
{
  AppearanceMergeResult result;
  for (const auto & prototype : source.prototypes()) {
    const auto update_result = update(AppearanceObservation{
        prototype.descriptor, prototype.best_quality, true, false, false});
    if (update_result.decision == AppearanceUpdateDecision::kRejected) {
      ++result.rejected;
    } else {
      ++result.accepted;
    }
  }
  return result;
}

const std::vector<AppearancePrototype> & AppearanceMemory::prototypes() const noexcept
{
  return prototypes_;
}

const std::optional<BestAppearanceView> & AppearanceMemory::best_view() const noexcept
{
  return best_view_;
}

std::string AppearanceMemory::summary_id() const
{
  if (prototypes_.empty()) {
    return {};
  }
  Fnv1a64 hash;
  hash.integer<std::uint32_t>(static_cast<std::uint32_t>(prototypes_.size()));
  const auto & descriptor = prototypes_.front().descriptor;
  hash.text(descriptor.encoder_id);
  hash.text(descriptor.checkpoint_id);
  hash.integer(descriptor.version);
  hash.integer(descriptor.dimension);
  for (const auto & prototype : prototypes_) {
    hash.integer<std::uint32_t>(
      static_cast<std::uint32_t>(prototype.descriptor.values.size()));
    for (const double value : prototype.descriptor.values) {
      hash.floating(value);
    }
    hash.floating(prototype.accumulated_quality_weight);
    hash.floating(prototype.best_quality);
    hash.integer(prototype.update_count);
  }
  std::ostringstream output;
  output << "appearance-v1-" << std::hex << std::setfill('0') <<
    std::setw(16) << std::nouppercase << hash.value();
  return output.str();
}

void AppearanceMemory::clear() noexcept
{
  prototypes_.clear();
  best_view_.reset();
}

}  // namespace track_robot_semantic_memory
