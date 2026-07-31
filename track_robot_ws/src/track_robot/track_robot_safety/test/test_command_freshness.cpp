#include <limits>

#include "gtest/gtest.h"
#include "track_robot_safety/command_freshness.hpp"

namespace
{

using track_robot_safety::CommandFreshness;
using track_robot_safety::classifyCommandFreshness;

TEST(CommandFreshness, Nav2MayWaitSafelyForItsFirstCommand)
{
  EXPECT_EQ(
    classifyCommandFreshness(true, false, std::numeric_limits<double>::infinity(), 0.15),
    CommandFreshness::WAITING_FOR_FIRST_COMMAND);
}

TEST(CommandFreshness, PreArmLifecycleCommandDoesNotDefeatBootstrapWait)
{
  EXPECT_EQ(
    classifyCommandFreshness(true, true, 67.0, 0.15),
    CommandFreshness::WAITING_FOR_FIRST_COMMAND);
}

TEST(CommandFreshness, DefaultModeStillRejectsMissingCommand)
{
  EXPECT_EQ(
    classifyCommandFreshness(false, false, std::numeric_limits<double>::infinity(), 0.15),
    CommandFreshness::STALE);
}

TEST(CommandFreshness, PreviouslyReceivedCommandStillFailsClosedWhenStale)
{
  EXPECT_EQ(
    classifyCommandFreshness(false, true, 0.16, 0.15),
    CommandFreshness::STALE);
}

TEST(CommandFreshness, FreshCommandRemainsFresh)
{
  EXPECT_EQ(
    classifyCommandFreshness(false, true, 0.14, 0.15),
    CommandFreshness::FRESH);
}

TEST(CommandFreshness, StaleZeroCommandMayRemainSafelyIdle)
{
  EXPECT_EQ(
    classifyCommandFreshness(false, true, 67.0, 0.15, true),
    CommandFreshness::WAITING_FOR_FIRST_COMMAND);
}

TEST(CommandFreshness, StaleNonzeroCommandStillFailsClosed)
{
  EXPECT_EQ(
    classifyCommandFreshness(false, true, 0.16, 0.15, false),
    CommandFreshness::STALE);
}

}  // namespace
