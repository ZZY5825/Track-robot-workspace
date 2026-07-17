#pragma once

#include <cstdint>

namespace track_robot_semantic_memory
{

enum class MemoryMode : std::uint8_t
{
  kObservationOnly = 0U,
  kLocalSession = 1U,
  kWorld = 2U,
};

enum class LifecycleState : std::uint8_t
{
  kTentative = 0U,
  kConfirmed = 1U,
  kStale = 2U,
  kLost = 3U,
  kArchived = 4U,
};

enum class SupportState : std::uint8_t
{
  kNone = 0U,
  kCameraLidar = 1U,
  kCameraOnly = 2U,
  kLidarOnly = 3U,
  kPredictionOnly = 4U,
};

enum class EvidenceFreshness : std::uint8_t
{
  kObserved = 0U,
  kPredicted = 1U,
  kUnsupported = 2U,
};

enum class VisibilityState : std::uint8_t
{
  kVisible = 0U,
  kOccluded = 1U,
  kOutsideFieldOfView = 2U,
  kUnknown = 3U,
};

enum class MotionState : std::uint8_t
{
  kStatic = 0U,
  kDynamic = 1U,
  kUncertain = 2U,
  kUnknown = kUncertain,
  kTemporarilyMoving = 3U,
};

enum class ReidentificationState : std::uint8_t
{
  kNotRequired = 0U,
  kPending = 1U,
  kConfirmed = 2U,
  kRejected = 3U,
};

}  // namespace track_robot_semantic_memory
