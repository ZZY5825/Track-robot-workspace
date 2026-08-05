#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <QString>

namespace track_robot_semantic_search_rviz_plugins
{

struct QueryCommand
{
  std::uint64_t query_id{0U};
  std::uint64_t query_version{0U};
  std::string normalized_text;
  std::string payload;
};

struct DiagnosticMatch
{
  std::uint64_t query_id{0U};
  std::uint64_t query_version{0U};
  std::string state;
  std::string reason;
  bool model_ready{false};
  bool model_ready_reported{false};
};

class QuerySession
{
public:
  static std::string normalize_query(const QString & text);

  QueryCommand new_query(
    const QString & text,
    std::uint64_t timestamp_seed);
  QueryCommand adopt_query(
    const QString & text,
    std::uint64_t query_id,
    std::uint64_t query_version);
  QueryCommand revise(const QString & text);

  [[nodiscard]] std::optional<QueryCommand> current() const;
  [[nodiscard]] std::optional<DiagnosticMatch> correlate_diagnostic(
    const std::string & payload) const;

private:
  static std::string payload_for(
    const std::string & normalized_text,
    std::uint64_t query_id,
    std::uint64_t query_version);

  std::uint64_t last_query_id_{0U};
  std::optional<QueryCommand> current_;
};

}  // namespace track_robot_semantic_search_rviz_plugins
