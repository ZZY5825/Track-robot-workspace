#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <random>

namespace track_robot_lidar_tracking {

class SourceEpoch {
 public:
  explicit SourceEpoch(const uint64_t configured_seed)
  : value_(configured_seed == 0U ? liveSeed() : configured_seed) {}

  uint64_t value() const noexcept { return value_; }

  uint64_t advance() noexcept {
    if (value_ == std::numeric_limits<uint64_t>::max()) {
      value_ = 1U;
    } else {
      ++value_;
      if (value_ == 0U) {
        value_ = 1U;
      }
    }
    return value_;
  }

 private:
  static uint64_t liveSeed() {
    std::random_device random;
    const auto clock_value = static_cast<uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
    uint64_t value = clock_value ^
      (static_cast<uint64_t>(random()) << 32U) ^
      static_cast<uint64_t>(random());
    return value == 0U ? 1U : value;
  }

  uint64_t value_;
};

}  // namespace track_robot_lidar_tracking
