#pragma once

#include <string>

namespace track_robot_semantic_memory
{

enum class MemoryEventType;

[[nodiscard]] std::string normalized_event_name(MemoryEventType type);

[[nodiscard]] std::string run_normalized_replay(
  const std::string & serialized_input);

}  // namespace track_robot_semantic_memory
