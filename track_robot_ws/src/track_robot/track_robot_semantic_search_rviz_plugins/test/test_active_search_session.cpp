#include <gtest/gtest.h>

#include "track_robot_semantic_search_rviz_plugins/active_search_session.hpp"

namespace plugin = track_robot_semantic_search_rviz_plugins;

TEST(ActiveSearchSession, BeginsOnceAndRejectsDuplicateStart)
{
  plugin::ActiveSearchSession session;
  const auto generation = session.begin();
  ASSERT_TRUE(generation.has_value());
  EXPECT_EQ(session.state(), plugin::ActiveSearchState::GOAL_PENDING);
  EXPECT_FALSE(session.begin().has_value());
}

TEST(ActiveSearchSession, AuthorizesOnlyOnceAfterWaitingFeedback)
{
  plugin::ActiveSearchSession session;
  const auto generation = *session.begin();
  ASSERT_TRUE(session.on_goal_response(generation, true));

  const auto observing = session.on_feedback(
    generation, 44U, "PASSIVE_OBSERVATION");
  EXPECT_TRUE(observing.adopt_query);
  EXPECT_FALSE(observing.authorize_rotation);

  const auto waiting = session.on_feedback(
    generation, 44U, "WAITING_FOR_AUTHORIZATION");
  EXPECT_TRUE(waiting.authorize_rotation);
  const auto repeated = session.on_feedback(
    generation, 44U, "WAITING_FOR_AUTHORIZATION");
  EXPECT_FALSE(repeated.authorize_rotation);
}

TEST(ActiveSearchSession, StopBeforeGoalResponseRemainsCancelled)
{
  plugin::ActiveSearchSession session;
  const auto generation = *session.begin();
  ASSERT_TRUE(session.request_stop());
  EXPECT_EQ(session.state(), plugin::ActiveSearchState::CANCEL_PENDING);
  ASSERT_TRUE(session.on_goal_response(generation, true));
  EXPECT_EQ(session.state(), plugin::ActiveSearchState::CANCEL_PENDING);
}

TEST(ActiveSearchSession, StopBeforeRejectedGoalResponseResetsToIdle)
{
  plugin::ActiveSearchSession session;
  const auto generation = *session.begin();
  ASSERT_TRUE(session.request_stop());

  ASSERT_TRUE(session.on_goal_response(generation, false));
  EXPECT_EQ(session.state(), plugin::ActiveSearchState::IDLE);
  EXPECT_FALSE(session.active());
}

TEST(ActiveSearchSession, RejectsFeedbackBeforeGoalAcceptance)
{
  plugin::ActiveSearchSession session;
  const auto generation = *session.begin();

  const auto decision = session.on_feedback(
    generation, 44U, "WAITING_FOR_AUTHORIZATION");
  EXPECT_FALSE(decision.accepted);
  EXPECT_FALSE(decision.adopt_query);
  EXPECT_FALSE(decision.authorize_rotation);
  EXPECT_EQ(session.state(), plugin::ActiveSearchState::GOAL_PENDING);
}

TEST(ActiveSearchSession, IgnoresStaleCallbacksAndResetsOnFinish)
{
  plugin::ActiveSearchSession session;
  const auto first = *session.begin();
  ASSERT_TRUE(session.finish(first));
  const auto second = *session.begin();
  EXPECT_FALSE(session.on_goal_response(first, true));
  EXPECT_FALSE(session.finish(first));
  EXPECT_TRUE(session.finish(second));
  EXPECT_EQ(session.state(), plugin::ActiveSearchState::IDLE);
}

TEST(ActiveSearchSession, RejectsZeroQueryIdAndTransitionsOnAuthorizationResult)
{
  plugin::ActiveSearchSession session;
  const auto generation = *session.begin();
  ASSERT_TRUE(session.on_goal_response(generation, true));

  const auto invalid = session.on_feedback(
    generation, 0U, "WAITING_FOR_AUTHORIZATION");
  EXPECT_FALSE(invalid.accepted);
  EXPECT_FALSE(invalid.adopt_query);
  EXPECT_FALSE(invalid.authorize_rotation);

  const auto waiting = session.on_feedback(
    generation, 7U, "WAITING_FOR_AUTHORIZATION");
  ASSERT_TRUE(waiting.authorize_rotation);
  EXPECT_EQ(session.state(), plugin::ActiveSearchState::AUTHORIZATION_PENDING);
  EXPECT_TRUE(session.on_authorization_result(generation, true));
  EXPECT_EQ(session.state(), plugin::ActiveSearchState::AUTHORIZED);
}
