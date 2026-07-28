#include "track_robot_semantic_search_rviz_plugins/semantic_search_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include <QFormLayout>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QString>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/display_context.hpp"

namespace track_robot_semantic_search_rviz_plugins
{
namespace
{

constexpr const char * kQueryTopic = "/semantic_search/query";
constexpr const char * kDiagnosticTopic =
  "/semantic_search/perception_diagnostics";
constexpr const char * kRegionsTopic = "/semantic_search/regions";
constexpr const char * kActiveObjectsTopic =
  "/semantic_memory/active_objects";
constexpr const char * kBestCandidateTopic =
  "/semantic_memory/best_candidate";

QString compact_reason(const std::string & reason)
{
  return reason.empty() ? QStringLiteral("no reason supplied") :
         QString::fromUtf8(reason.c_str());
}

}  // namespace

SemanticSearchPanel::SemanticSearchPanel(QWidget * parent)
: rviz_common::Panel(parent),
  query_topic_(kQueryTopic),
  diagnostic_topic_(kDiagnosticTopic),
  regions_topic_(kRegionsTopic),
  active_objects_topic_(kActiveObjectsTopic),
  best_candidate_topic_(kBestCandidateTopic)
{
  auto * safety = new QLabel(
    tr("PASSIVE OBSERVATION ONLY — this panel cannot move the robot."),
    this);
  safety->setWordWrap(true);
  safety->setStyleSheet(
    "QLabel { color: #ffcc66; font-weight: bold; padding: 6px; "
    "background: #3b2f16; }");

  query_input_ = new QLineEdit(this);
  query_input_->setPlaceholderText(
    tr("English object description, e.g. blue chair"));
  new_button_ = new QPushButton(tr("New Query"), this);
  revise_button_ = new QPushButton(tr("Revise"), this);

  auto * buttons = new QHBoxLayout();
  buttons->addWidget(new_button_);
  buttons->addWidget(revise_button_);

  query_status_ = new QLabel(tr("none"), this);
  model_status_ = new QLabel(tr("waiting for diagnostics"), this);
  acknowledgement_status_ = new QLabel(tr("not submitted"), this);
  region_status_ = new QLabel(tr("waiting"), this);
  object_status_ = new QLabel(tr("waiting"), this);
  best_status_ = new QLabel(tr("unavailable"), this);
  for (auto * label : {
      query_status_, model_status_, acknowledgement_status_,
      region_status_, object_status_, best_status_})
  {
    label->setWordWrap(true);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  }

  auto * form = new QFormLayout();
  form->addRow(tr("Current query"), query_status_);
  form->addRow(tr("Acknowledgement"), acknowledgement_status_);
  form->addRow(tr("Model"), model_status_);
  form->addRow(tr("Image candidates"), region_status_);
  form->addRow(tr("3D objects"), object_status_);
  form->addRow(tr("Best candidate"), best_status_);

  auto * layout = new QVBoxLayout();
  layout->addWidget(safety);
  layout->addWidget(query_input_);
  layout->addLayout(buttons);
  layout->addLayout(form);
  layout->addStretch();
  setLayout(layout);

  connect(
    new_button_, &QPushButton::clicked,
    this, &SemanticSearchPanel::submit_new_query);
  connect(
    revise_button_, &QPushButton::clicked,
    this, &SemanticSearchPanel::submit_revision);
  connect(
    query_input_, &QLineEdit::returnPressed,
    this, &SemanticSearchPanel::submit_new_query);
}

void SemanticSearchPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    acknowledgement_status_->setText(
      tr("RViz ROS node is unavailable"));
    return;
  }
  node_ = abstraction->get_raw_node();
  if (!node_) {
    acknowledgement_status_->setText(
      tr("RViz ROS node is unavailable"));
    return;
  }

  const auto reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  const auto snapshot_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  query_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    query_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  diagnostic_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    diagnostic_topic_,
    reliable,
    std::bind(
      &SemanticSearchPanel::on_diagnostic, this, std::placeholders::_1));
  region_subscription_ = node_->create_subscription<
    track_robot_interfaces::msg::SemanticRegionArray>(
    regions_topic_,
    reliable,
    std::bind(
      &SemanticSearchPanel::on_regions, this, std::placeholders::_1));
  object_subscription_ = node_->create_subscription<
    track_robot_interfaces::msg::SemanticObjectArray>(
    active_objects_topic_,
    snapshot_qos,
    std::bind(
      &SemanticSearchPanel::on_active_objects, this, std::placeholders::_1));
  best_subscription_ = node_->create_subscription<
    track_robot_interfaces::msg::SemanticObjectArray>(
    best_candidate_topic_,
    snapshot_qos,
    std::bind(
      &SemanticSearchPanel::on_best_candidate, this, std::placeholders::_1));
}

void SemanticSearchPanel::submit_new_query()
{
  publish_query(false);
}

void SemanticSearchPanel::submit_revision()
{
  publish_query(true);
}

void SemanticSearchPanel::publish_query(bool revision)
{
  if (!query_publisher_) {
    acknowledgement_status_->setText(tr("query publisher is unavailable"));
    return;
  }
  try {
    QueryCommand command;
    if (revision) {
      command = session_.revise(query_input_->text());
    } else {
      const auto now = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      const auto seed = now > 0 ?
        static_cast<std::uint64_t>(now) : std::uint64_t{1U};
      command = session_.new_query(query_input_->text(), seed);
    }
    std_msgs::msg::String message;
    message.data = command.payload;
    query_publisher_->publish(message);
    query_status_->setText(
      tr("%1  [id=%2 version=%3]")
      .arg(QString::fromUtf8(command.normalized_text.c_str()))
      .arg(command.query_id)
      .arg(command.query_version));
    if (query_publisher_->get_subscription_count() == 0U) {
      acknowledgement_status_->setText(
        tr("sent; no query subscriber is currently visible"));
    } else {
      acknowledgement_status_->setText(tr("sent; waiting for acknowledgement"));
    }
  } catch (const std::exception & error) {
    acknowledgement_status_->setText(
      tr("not sent: %1").arg(QString::fromUtf8(error.what())));
  }
}

void SemanticSearchPanel::on_diagnostic(
  const std_msgs::msg::String::SharedPtr message)
{
  const auto matched = session_.correlate_diagnostic(message->data);
  if (!matched.has_value()) {
    return;
  }
  const auto acknowledgement = QStringLiteral("%1: %2")
    .arg(QString::fromUtf8(matched->state.c_str()))
    .arg(compact_reason(matched->reason));
  queue_label(acknowledgement_status_, acknowledgement);
  const auto model = matched->model_ready_reported ?
    (matched->model_ready ? tr("READY") : tr("UNAVAILABLE")) :
    tr("not reported");
  queue_label(model_status_, model);
}

void SemanticSearchPanel::on_regions(
  const track_robot_interfaces::msg::SemanticRegionArray::SharedPtr message)
{
  const auto current = session_.current();
  if (!current.has_value() ||
    message->query_id != current->query_id ||
    message->query_version != current->query_version)
  {
    return;
  }
  queue_label(
    region_status_,
    tr("%1 correlated candidates").arg(message->regions.size()));
}

void SemanticSearchPanel::on_active_objects(
  const track_robot_interfaces::msg::SemanticObjectArray::SharedPtr message)
{
  queue_label(
    object_status_,
    tr("%1 active objects  [epoch=%2]")
    .arg(message->objects.size())
    .arg(message->memory_epoch_id));
}

void SemanticSearchPanel::on_best_candidate(
  const track_robot_interfaces::msg::SemanticObjectArray::SharedPtr message)
{
  if (message->objects.empty()) {
    queue_label(
      best_status_,
      tr("none (not calibrated or no qualifying match)"));
    return;
  }
  if (message->objects.size() != 1U) {
    queue_label(best_status_, tr("invalid winner snapshot"));
    return;
  }
  const auto & candidate = message->objects.front();
  QString text = tr("object %1").arg(candidate.global_object_id);
  if (candidate.active_query_id != 0U &&
    candidate.active_query_version != 0U &&
    std::isfinite(candidate.task_relevance))
  {
    text += tr("  relevance=%1").arg(
      static_cast<double>(candidate.task_relevance), 0, 'f', 3);
  }
  queue_label(best_status_, text);
}

void SemanticSearchPanel::queue_label(
  QLabel * label,
  const QString & value)
{
  QMetaObject::invokeMethod(
    this,
    [label, value]() {
      label->setText(value);
    },
    Qt::QueuedConnection);
}

void SemanticSearchPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString value;
  if (config.mapGetString("query_topic", &value)) {
    query_topic_ = value.toStdString();
  }
  if (config.mapGetString("diagnostic_topic", &value)) {
    diagnostic_topic_ = value.toStdString();
  }
  if (config.mapGetString("regions_topic", &value)) {
    regions_topic_ = value.toStdString();
  }
  if (config.mapGetString("active_objects_topic", &value)) {
    active_objects_topic_ = value.toStdString();
  }
  if (config.mapGetString("best_candidate_topic", &value)) {
    best_candidate_topic_ = value.toStdString();
  }
}

void SemanticSearchPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("query_topic", QString::fromStdString(query_topic_));
  config.mapSetValue(
    "diagnostic_topic", QString::fromStdString(diagnostic_topic_));
  config.mapSetValue("regions_topic", QString::fromStdString(regions_topic_));
  config.mapSetValue(
    "active_objects_topic", QString::fromStdString(active_objects_topic_));
  config.mapSetValue(
    "best_candidate_topic", QString::fromStdString(best_candidate_topic_));
}

}  // namespace track_robot_semantic_search_rviz_plugins

PLUGINLIB_EXPORT_CLASS(
  track_robot_semantic_search_rviz_plugins::SemanticSearchPanel,
  rviz_common::Panel)
