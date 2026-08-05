#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <QString>

#include "track_robot_semantic_search_rviz_plugins/query_session.hpp"

namespace plugin = track_robot_semantic_search_rviz_plugins;

TEST(QuerySession, NormalizesNewQueryAndBuildsCanonicalPayload)
{
  plugin::QuerySession session;

  const auto command = session.new_query(
    QString::fromUtf8("  blue   chair  "), 100U);
  const auto payload = nlohmann::json::parse(command.payload);

  EXPECT_EQ(command.query_id, 100U);
  EXPECT_EQ(command.query_version, 1U);
  EXPECT_EQ(command.normalized_text, "blue chair");
  EXPECT_EQ(payload.size(), 3U);
  EXPECT_EQ(payload.at("query_id"), 100U);
  EXPECT_EQ(payload.at("query_text"), "blue chair");
  EXPECT_EQ(payload.at("query_version"), 1U);
}

TEST(QuerySession, UsesNfkcAndStrictlyIncreasingProcessIds)
{
  plugin::QuerySession session;

  const auto first = session.new_query(
    QString::fromUtf8("ｂｌｕｅ　ｃｈａｉｒ"), 200U);
  const auto repeated = session.new_query("person", 200U);
  const auto backwards = session.new_query("backpack", 150U);

  EXPECT_EQ(first.normalized_text, "blue chair");
  EXPECT_EQ(first.query_id, 200U);
  EXPECT_EQ(repeated.query_id, 201U);
  EXPECT_EQ(backwards.query_id, 202U);
}

TEST(QuerySession, RevisionKeepsIdAndAdvancesVersion)
{
  plugin::QuerySession session;
  const auto original = session.new_query("blue chair", 55U);

  const auto revised = session.revise("dark blue chair");

  EXPECT_EQ(revised.query_id, original.query_id);
  EXPECT_EQ(revised.query_version, 2U);
  EXPECT_EQ(revised.normalized_text, "dark blue chair");
}

TEST(QuerySession, RejectsEmptyOversizedAndRevisionWithoutCurrentQuery)
{
  plugin::QuerySession session;

  EXPECT_THROW((void)session.revise("chair"), std::logic_error);
  EXPECT_THROW((void)session.new_query("   ", 1U), std::invalid_argument);
  EXPECT_THROW(
    (void)session.new_query(QString(513, QChar('x')), 1U),
    std::invalid_argument);
  EXPECT_THROW(
    (void)session.new_query("chair", 0U),
    std::invalid_argument);
}

TEST(QuerySession, CorrelatesOnlyMatchingDiagnostics)
{
  plugin::QuerySession session;
  const auto command = session.new_query("blue chair", 91U);

  EXPECT_FALSE(session.correlate_diagnostic(
    R"({"state":"query_accepted","query_id":90,"query_version":1})")
    .has_value());
  EXPECT_FALSE(session.correlate_diagnostic("not-json").has_value());

  const auto matched = session.correlate_diagnostic(
    R"({"state":"query_accepted","reason":"ready","query_id":91,)"
    R"("query_version":1,"model_ready":true})");

  ASSERT_TRUE(matched.has_value());
  EXPECT_EQ(matched->state, "query_accepted");
  EXPECT_EQ(matched->reason, "ready");
  EXPECT_TRUE(matched->model_ready);
  EXPECT_EQ(matched->query_id, command.query_id);
}

TEST(QuerySession, AdoptsExternallyAllocatedQueryForCorrelation)
{
  plugin::QuerySession session;

  const auto adopted = session.adopt_query("  green   bottle  ", 901U, 1U);

  EXPECT_EQ(adopted.normalized_text, "green bottle");
  EXPECT_EQ(adopted.query_id, 901U);
  EXPECT_EQ(adopted.query_version, 1U);
  ASSERT_TRUE(session.correlate_diagnostic(
    R"({"state":"query_accepted","query_id":901,"query_version":1})")
    .has_value());
}

TEST(QuerySession, RejectsInvalidExternalQueryIdentity)
{
  plugin::QuerySession session;

  EXPECT_THROW((void)session.adopt_query("bottle", 0U, 1U),
    std::invalid_argument);
  EXPECT_THROW((void)session.adopt_query("bottle", 1U, 0U),
    std::invalid_argument);
  EXPECT_THROW((void)session.adopt_query("   ", 1U, 1U),
    std::invalid_argument);
}

TEST(QuerySession, RejectsIdentifierAndVersionOverflow)
{
  plugin::QuerySession session;

  const auto largest = session.new_query(
    "chair", std::numeric_limits<std::uint64_t>::max());
  EXPECT_EQ(largest.query_id, std::numeric_limits<std::uint64_t>::max());
  EXPECT_THROW(
    (void)session.new_query("person", 1U),
    std::overflow_error);
}
