#pragma once

namespace track_robot_safety
{

enum class CommandFreshness
{
  FRESH,
  WAITING_FOR_FIRST_COMMAND,
  STALE,
};

inline CommandFreshness classifyCommandFreshness(
  const bool waiting_for_first_command,
  const bool have_command,
  const double command_age_sec,
  const double command_timeout_sec,
  const bool stale_zero_command_may_idle = false)
{
  if (waiting_for_first_command) {
    return CommandFreshness::WAITING_FOR_FIRST_COMMAND;
  }
  if (!have_command) {
    return CommandFreshness::STALE;
  }
  if (command_age_sec <= command_timeout_sec) {
    return CommandFreshness::FRESH;
  }
  return stale_zero_command_may_idle ?
    CommandFreshness::WAITING_FOR_FIRST_COMMAND : CommandFreshness::STALE;
}

}  // namespace track_robot_safety
