#include "track_robot_semantic_search_rviz_plugins/query_session.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>
#include <QVector>

namespace track_robot_semantic_search_rviz_plugins
{
namespace
{

constexpr int kMaximumQueryCharacters = 512;
constexpr std::size_t kMaximumReasonCharacters = 256U;

std::string bounded_reason(std::string reason)
{
  if (reason.size() > kMaximumReasonCharacters) {
    reason.resize(kMaximumReasonCharacters);
  }
  return reason;
}

}  // namespace

QueryCommand QuerySession::new_query(
  const QString & text,
  std::uint64_t timestamp_seed)
{
  if (timestamp_seed == 0U) {
    throw std::invalid_argument("query timestamp seed must be positive");
  }
  std::uint64_t query_id = timestamp_seed;
  if (query_id <= last_query_id_) {
    if (last_query_id_ == std::numeric_limits<std::uint64_t>::max()) {
      throw std::overflow_error("query ID space is exhausted");
    }
    query_id = last_query_id_ + 1U;
  }
  const auto normalized = normalize_query(text);
  current_ = QueryCommand{
    query_id,
    1U,
    normalized,
    payload_for(normalized, query_id, 1U)};
  last_query_id_ = query_id;
  return *current_;
}

QueryCommand QuerySession::adopt_query(
  const QString & text,
  std::uint64_t query_id,
  std::uint64_t query_version)
{
  if (query_id == 0U || query_version == 0U) {
    throw std::invalid_argument(
            "external query ID and version must be positive");
  }
  const auto normalized = normalize_query(text);
  current_ = QueryCommand{
    query_id,
    query_version,
    normalized,
    payload_for(normalized, query_id, query_version)};
  last_query_id_ = std::max(last_query_id_, query_id);
  return *current_;
}

QueryCommand QuerySession::revise(const QString & text)
{
  if (!current_.has_value()) {
    throw std::logic_error("create a new query before revising it");
  }
  if (current_->query_version ==
    std::numeric_limits<std::uint64_t>::max())
  {
    throw std::overflow_error("query version space is exhausted");
  }
  const auto normalized = normalize_query(text);
  const auto version = current_->query_version + 1U;
  current_ = QueryCommand{
    current_->query_id,
    version,
    normalized,
    payload_for(normalized, current_->query_id, version)};
  return *current_;
}

std::optional<QueryCommand> QuerySession::current() const
{
  return current_;
}

std::optional<DiagnosticMatch> QuerySession::correlate_diagnostic(
  const std::string & payload) const
{
  if (!current_.has_value()) {
    return std::nullopt;
  }
  try {
    const auto value = nlohmann::json::parse(payload);
    if (!value.is_object() ||
      !value.contains("query_id") ||
      !value.at("query_id").is_number_unsigned() ||
      !value.contains("query_version") ||
      !value.at("query_version").is_number_unsigned() ||
      !value.contains("state") ||
      !value.at("state").is_string())
    {
      return std::nullopt;
    }
    const auto query_id = value.at("query_id").get<std::uint64_t>();
    const auto query_version =
      value.at("query_version").get<std::uint64_t>();
    if (query_id != current_->query_id ||
      query_version != current_->query_version)
    {
      return std::nullopt;
    }
    std::string reason;
    if (value.contains("reason") && value.at("reason").is_string()) {
      reason = bounded_reason(value.at("reason").get<std::string>());
    }
    const bool has_model_ready =
      value.contains("model_ready") && value.at("model_ready").is_boolean();
    return DiagnosticMatch{
      query_id,
      query_version,
      value.at("state").get<std::string>(),
      reason,
      has_model_ready ? value.at("model_ready").get<bool>() : false,
      has_model_ready};
  } catch (const nlohmann::json::exception &) {
    return std::nullopt;
  }
}

std::string QuerySession::normalize_query(const QString & text)
{
  const auto normalized =
    text.normalized(QString::NormalizationForm_KC).simplified();
  if (normalized.isEmpty()) {
    throw std::invalid_argument("query text must not be empty");
  }
  if (normalized.toUcs4().size() > kMaximumQueryCharacters) {
    throw std::invalid_argument(
            "query text exceeds 512 normalized characters");
  }
  return normalized.toUtf8().toStdString();
}

std::string QuerySession::payload_for(
  const std::string & normalized_text,
  std::uint64_t query_id,
  std::uint64_t query_version)
{
  return nlohmann::json{
    {"query_id", query_id},
    {"query_text", normalized_text},
    {"query_version", query_version},
  }.dump();
}

}  // namespace track_robot_semantic_search_rviz_plugins
