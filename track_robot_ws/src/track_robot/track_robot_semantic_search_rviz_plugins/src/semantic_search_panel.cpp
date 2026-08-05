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
constexpr const char * kSelectedTargetTopic =
  "/semantic_search/phase4a/selected_target";
constexpr const char * kDiagnosticRankingTopic =
  "/semantic_memory/diagnostic_ranking";
constexpr const char * kAuthorizeService =
  "/semantic_navigation/authorize_approach";
constexpr const char * kCancelDisarmService =
  "/semantic_navigation/cancel_and_disarm";
constexpr const char * kSearchAction =
  "/semantic_search/search_for_object";
constexpr const char * kAuthorizeRotationService =
  "/semantic_search/active_search/authorize_rotation";
constexpr const char * kCancelSearchService =
  "/semantic_search/active_search/cancel";
constexpr std::int32_t kSearchTimeoutSec = 60;
constexpr float kMaximumSearchAngleRad = 1.5708F;
constexpr int kMaximumResultReasonCharacters = 256;

QString compact_reason(const std::string & reason)
{
  return reason.empty() ? QStringLiteral("no reason supplied") :
         QString::fromUtf8(reason.c_str());
}

QString support_name(std::uint8_t support)
{
  using Object = track_robot_interfaces::msg::SemanticObject;
  switch (support) {
    case Object::SUPPORT_CAMERA_LIDAR:
      return QStringLiteral("camera+lidar");
    case Object::SUPPORT_CAMERA_ONLY:
      return QStringLiteral("camera-only");
    case Object::SUPPORT_LIDAR_ONLY:
      return QStringLiteral("lidar-only");
    case Object::SUPPORT_PREDICTION_ONLY:
      return QStringLiteral("prediction-only");
    default:
      return QStringLiteral("none");
  }
}

QString active_search_result_text(const SearchForObject::Result & result)
{
  QString status;
  switch (result.status) {
    case SearchForObject::Result::CONFIRMED:
      status = QStringLiteral("confirmed");
      break;
    case SearchForObject::Result::NOT_FOUND:
      status = QStringLiteral("not found");
      break;
    case SearchForObject::Result::UNCERTAIN:
      status = QStringLiteral("uncertain");
      break;
    case SearchForObject::Result::CANCELLED:
      status = QStringLiteral("cancelled");
      break;
    case SearchForObject::Result::TIMEOUT:
      status = QStringLiteral("timed out");
      break;
    case SearchForObject::Result::SAFETY_REJECTED:
      status = QStringLiteral("safety rejected");
      break;
    case SearchForObject::Result::SENSOR_UNAVAILABLE:
      status = QStringLiteral("sensor unavailable");
      break;
    case SearchForObject::Result::MODEL_UNAVAILABLE:
      status = QStringLiteral("model unavailable");
      break;
    case SearchForObject::Result::LOCALIZATION_UNAVAILABLE:
      status = QStringLiteral("localization unavailable");
      break;
    case SearchForObject::Result::SEARCH_SPACE_EXHAUSTED:
      status = QStringLiteral("search space exhausted");
      break;
    case SearchForObject::Result::INTERNAL_FAULT:
      status = QStringLiteral("internal fault");
      break;
    default:
      status = QStringLiteral("unknown result (%1)").arg(result.status);
      break;
  }
  if (result.selected_object_valid) {
    status += QStringLiteral("; object %1").arg(
      result.selected_global_object_id);
  }
  const auto evidence = QString::fromStdString(result.evidence_summary);
  if (!evidence.isEmpty()) {
    status += QStringLiteral(": %1").arg(
      evidence.left(kMaximumResultReasonCharacters));
  }
  return status;
}

}  // namespace

SemanticSearchPanel::SemanticSearchPanel(QWidget * parent)
: rviz_common::Panel(parent),
  query_topic_(kQueryTopic),
  diagnostic_topic_(kDiagnosticTopic),
  regions_topic_(kRegionsTopic),
  active_objects_topic_(kActiveObjectsTopic),
  best_candidate_topic_(kBestCandidateTopic),
  selected_target_topic_(kSelectedTargetTopic),
  diagnostic_ranking_topic_(kDiagnosticRankingTopic),
  authorize_service_(kAuthorizeService),
  cancel_disarm_service_(kCancelDisarmService),
  search_action_(kSearchAction),
  authorize_rotation_service_(kAuthorizeRotationService),
  cancel_search_service_(kCancelSearchService)
{
  auto * safety = new QLabel(
    tr("SUPERVISED SEMANTIC APPROACH — motion starts only after an exact "
      "target is selected and Start Approach is clicked."),
    this);
  safety->setWordWrap(true);
  safety->setStyleSheet(
    "QLabel { color: #ffcc66; font-weight: bold; padding: 6px; "
    "background: #3b2f16; }");
  auto * calibration_warning = new QLabel(
    tr("RC override and E-stop remain authoritative. Cancel & Disarm "
      "requests an immediate Nav2 cancel and safety disarm."), this);
  calibration_warning->setWordWrap(true);
  calibration_warning->setStyleSheet(
    "QLabel { color: #ffffff; font-weight: bold; padding: 7px; "
    "background: #8a2d3c; }");

  query_input_ = new QLineEdit(this);
  query_input_->setPlaceholderText(
    tr("English object description, e.g. blue chair"));
  new_button_ = new QPushButton(tr("New Query"), this);
  revise_button_ = new QPushButton(tr("Revise"), this);
  finding_button_ = new QPushButton(tr("Start Finding"), this);
  start_approach_button_ = new QPushButton(tr("Start Approach"), this);
  start_approach_button_->setEnabled(false);
  cancel_disarm_button_ = new QPushButton(tr("Cancel & Disarm"), this);

  auto * buttons = new QHBoxLayout();
  buttons->addWidget(new_button_);
  buttons->addWidget(revise_button_);

  auto * motion_buttons = new QHBoxLayout();
  motion_buttons->addWidget(start_approach_button_);
  motion_buttons->addWidget(cancel_disarm_button_);

  query_status_ = new QLabel(tr("none"), this);
  model_status_ = new QLabel(tr("waiting for diagnostics"), this);
  acknowledgement_status_ = new QLabel(tr("not submitted"), this);
  region_status_ = new QLabel(tr("waiting"), this);
  object_status_ = new QLabel(tr("waiting"), this);
  best_status_ = new QLabel(tr("unavailable"), this);
  diagnostic_ranking_status_ = new QLabel(tr("waiting"), this);
  finding_status_ = new QLabel(tr("idle"), this);
  motion_status_ = new QLabel(
    tr("selected target is not ready"), this);
  diagnostic_ranking_status_->setStyleSheet(
    "QLabel { color: #d8b4fe; font-weight: bold; }");
  for (auto * label : {
      query_status_, model_status_, acknowledgement_status_,
      region_status_, object_status_, diagnostic_ranking_status_,
      best_status_, finding_status_, motion_status_})
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
  form->addRow(tr("Diagnostic ranking"), diagnostic_ranking_status_);
  form->addRow(tr("Best candidate"), best_status_);
  form->addRow(tr("Active search"), finding_status_);
  form->addRow(tr("Motion authorization"), motion_status_);

  auto * layout = new QVBoxLayout();
  layout->addWidget(safety);
  layout->addWidget(calibration_warning);
  layout->addWidget(query_input_);
  layout->addLayout(buttons);
  layout->addWidget(finding_button_);
  layout->addLayout(motion_buttons);
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
  connect(
    finding_button_, &QPushButton::clicked,
    this, &SemanticSearchPanel::toggle_finding);
  connect(
    start_approach_button_, &QPushButton::clicked,
    this, &SemanticSearchPanel::start_approach);
  connect(
    cancel_disarm_button_, &QPushButton::clicked,
    this, &SemanticSearchPanel::cancel_and_disarm);
}

SemanticSearchPanel::~SemanticSearchPanel()
{
  const auto lifetime = callback_lifetime_;
  if (lifetime) {
    std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
    lifetime->alive = false;
  }

  {
    std::lock_guard<std::mutex> lock(finding_mutex_);
    search_goal_handle_.reset();
  }
  search_client_.reset();
  authorize_rotation_client_.reset();
  cancel_search_client_.reset();
  callback_lifetime_.reset();
}

bool SemanticSearchPanel::TargetReference::same_identity(
  const TargetReference & other) const
{
  return memory_epoch_id == other.memory_epoch_id &&
         global_object_id == other.global_object_id &&
         localization_epoch_id == other.localization_epoch_id &&
         query_id == other.query_id &&
         query_version == other.query_version;
}

bool SemanticSearchPanel::TargetReference::complete() const
{
  return memory_epoch_id != 0U && global_object_id != 0U &&
         localization_epoch_id != 0U && query_id != 0U &&
         query_version != 0U && snapshot_sequence != 0U;
}

std::optional<SemanticSearchPanel::TargetReference>
SemanticSearchPanel::reference_from(
  const track_robot_interfaces::msg::SemanticObjectArray & message)
{
  if (message.objects.size() != 1U) {
    return std::nullopt;
  }
  const auto & object = message.objects.front();
  TargetReference reference;
  reference.memory_epoch_id = message.memory_epoch_id;
  reference.global_object_id = object.global_object_id;
  reference.localization_epoch_id = object.localization_epoch_id;
  reference.query_id = object.active_query_id;
  reference.query_version = object.active_query_version;
  reference.snapshot_sequence = message.snapshot_sequence;
  return reference.complete() ?
         std::optional<TargetReference>(reference) : std::nullopt;
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
  selected_target_subscription_ = node_->create_subscription<
    track_robot_interfaces::msg::SemanticObjectArray>(
    selected_target_topic_,
    snapshot_qos,
    std::bind(
      &SemanticSearchPanel::on_selected_target, this, std::placeholders::_1));
  diagnostic_ranking_subscription_ = node_->create_subscription<
    track_robot_interfaces::msg::SemanticObjectArray>(
    diagnostic_ranking_topic_,
    snapshot_qos,
    std::bind(
      &SemanticSearchPanel::on_diagnostic_ranking, this,
      std::placeholders::_1));
  authorize_client_ = node_->create_client<
    track_robot_interfaces::srv::AuthorizeSemanticApproach>(
    authorize_service_);
  cancel_disarm_client_ = node_->create_client<std_srvs::srv::Trigger>(
    cancel_disarm_service_);
  search_client_ = rclcpp_action::create_client<SearchForObject>(
    node_, search_action_);
  authorize_rotation_client_ = node_->create_client<std_srvs::srv::Trigger>(
    authorize_rotation_service_);
  cancel_search_client_ = node_->create_client<std_srvs::srv::Trigger>(
    cancel_search_service_);
}

void SemanticSearchPanel::submit_new_query()
{
  publish_query(false);
}

void SemanticSearchPanel::submit_revision()
{
  publish_query(true);
}

void SemanticSearchPanel::toggle_finding()
{
  bool active = false;
  {
    std::lock_guard<std::mutex> lock(finding_mutex_);
    active = finding_session_.active();
  }
  if (active) {
    stop_finding();
  } else {
    start_finding();
  }
}

void SemanticSearchPanel::start_finding()
{
  if (!search_client_ || !search_client_->action_server_is_ready()) {
    render_finding_state(tr("active-search action server is unavailable"));
    return;
  }

  const auto normalized = query_input_->text().normalized(
    QString::NormalizationForm_KC).simplified();
  if (normalized.isEmpty()) {
    render_finding_state(tr("query text must not be empty"));
    return;
  }
  const auto normalized_text = normalized.toStdString();

  std::optional<std::uint64_t> generation;
  {
    std::lock_guard<std::mutex> lock(finding_mutex_);
    generation = finding_session_.begin();
  }
  if (!generation.has_value()) {
    return;
  }

  finding_button_->setEnabled(true);
  finding_button_->setText(tr("Stop Finding"));
  finding_status_->setText(tr("starting active search"));

  SearchForObject::Goal goal;
  goal.query_text = normalized_text;
  goal.timeout.sec = kSearchTimeoutSec;
  goal.timeout.nanosec = 0U;
  goal.allow_rotation = true;
  goal.maximum_rotation_angle = kMaximumSearchAngleRad;
  goal.client_request_id =
    "rviz-phase5a-" + std::to_string(*generation);

  rclcpp_action::Client<SearchForObject>::SendGoalOptions options;
  const auto weak_lifetime =
    std::weak_ptr<CallbackLifetime>(callback_lifetime_);
  options.goal_response_callback =
    [this, weak_lifetime, generation = *generation](
      std::shared_future<SearchGoalHandle::SharedPtr> future)
    {
      const auto lifetime = weak_lifetime.lock();
      if (!lifetime) {
        return;
      }
      std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      try {
        const auto handle = future.get();
        bool cancel_after_acceptance = false;
        {
          std::lock_guard<std::mutex> lock(finding_mutex_);
          if (finding_session_.generation() != generation ||
            !finding_session_.on_goal_response(generation, handle != nullptr))
          {
            return;
          }
          if (handle) {
            search_goal_handle_ = handle;
            cancel_after_acceptance =
              finding_session_.state() == ActiveSearchState::CANCEL_PENDING;
          }
        }
        if (!handle) {
          QMetaObject::invokeMethod(
            this,
            [this, weak_lifetime, generation]() {
              const auto lifetime = weak_lifetime.lock();
              if (!lifetime) {
                return;
              }
              std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
              if (!lifetime->alive) {
                return;
              }
              std::lock_guard<std::mutex> lock(finding_mutex_);
              if (finding_session_.generation() != generation ||
                finding_session_.active())
              {
                return;
              }
              search_goal_handle_.reset();
              finding_button_->setText(tr("Start Finding"));
              finding_button_->setEnabled(true);
              finding_status_->setText(tr("active-search goal was rejected"));
            },
            Qt::QueuedConnection);
          return;
        }
        if (cancel_after_acceptance && search_client_) {
          search_client_->async_cancel_goal(handle);
        } else {
          QMetaObject::invokeMethod(
            this,
            [this, weak_lifetime, generation]() {
              const auto lifetime = weak_lifetime.lock();
              if (!lifetime) {
                return;
              }
              std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
              if (!lifetime->alive) {
                return;
              }
              std::lock_guard<std::mutex> lock(finding_mutex_);
              if (finding_session_.generation() != generation ||
                !finding_session_.active())
              {
                return;
              }
              finding_button_->setEnabled(true);
              finding_status_->setText(tr("searching"));
            },
            Qt::QueuedConnection);
        }
      } catch (const std::exception & error) {
        finish_finding(
          generation,
          tr("active-search goal request failed: %1").arg(error.what()));
      }
    };
  options.feedback_callback =
    [this, weak_lifetime, generation = *generation, normalized_text](
      SearchGoalHandle::SharedPtr,
      const std::shared_ptr<const SearchForObject::Feedback> feedback)
    {
      const auto lifetime = weak_lifetime.lock();
      if (!lifetime) {
        return;
      }
      std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      ActiveSearchFeedbackDecision decision;
      {
        std::lock_guard<std::mutex> lock(finding_mutex_);
        if (finding_session_.generation() != generation) {
          return;
        }
        decision = finding_session_.on_feedback(
          generation, feedback->query_id, feedback->current_reason);
        if (!decision.accepted) {
          return;
        }
        if (decision.adopt_query) {
          try {
            std::lock_guard<std::mutex> session_lock(session_mutex_);
            if (finding_session_.generation() != generation ||
              !finding_session_.active())
            {
              return;
            }
            const auto query = session_.adopt_query(
              QString::fromStdString(normalized_text), decision.query_id, 1U);
            const auto status = tr("%1  [id=%2 version=%3]")
              .arg(QString::fromStdString(query.normalized_text))
              .arg(query.query_id)
              .arg(query.query_version);
            QMetaObject::invokeMethod(
              this,
              [this, weak_lifetime, generation, status]() {
                const auto lifetime = weak_lifetime.lock();
                if (!lifetime) {
                  return;
                }
                std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
                if (!lifetime->alive) {
                  return;
                }
                std::lock_guard<std::mutex> lock(finding_mutex_);
                if (finding_session_.generation() == generation &&
                  finding_session_.active())
                {
                  query_status_->setText(status);
                }
              },
              Qt::QueuedConnection);
          } catch (const std::exception & error) {
            const auto status =
              tr("active-search query adoption failed: %1").arg(error.what());
            QMetaObject::invokeMethod(
              this,
              [this, weak_lifetime, generation, status]() {
                const auto lifetime = weak_lifetime.lock();
                if (!lifetime) {
                  return;
                }
                std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
                if (!lifetime->alive) {
                  return;
                }
                std::lock_guard<std::mutex> lock(finding_mutex_);
                if (finding_session_.generation() == generation &&
                  finding_session_.active())
                {
                  finding_status_->setText(status);
                }
              },
              Qt::QueuedConnection);
            return;
          }
        }
      }
      if (decision.authorize_rotation &&
        feedback->current_reason == "WAITING_FOR_AUTHORIZATION")
      {
        authorize_rotation(generation);
      }
    };
  options.result_callback =
    [this, weak_lifetime, generation = *generation](
      const SearchGoalHandle::WrappedResult & result)
    {
      const auto lifetime = weak_lifetime.lock();
      if (!lifetime) {
        return;
      }
      std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      if (result.result) {
        finish_finding(generation, active_search_result_text(*result.result));
      } else {
        finish_finding(generation, tr("active-search result is unavailable"));
      }
    };

  try {
    search_client_->async_send_goal(goal, options);
  } catch (const std::exception & error) {
    finish_finding(
      *generation,
      tr("active-search goal submission failed: %1").arg(error.what()));
  }
}

void SemanticSearchPanel::stop_finding()
{
  const auto weak_lifetime =
    std::weak_ptr<CallbackLifetime>(callback_lifetime_);
  SearchGoalHandle::SharedPtr handle;
  std::uint64_t generation = 0U;
  {
    std::lock_guard<std::mutex> lock(finding_mutex_);
    if (!finding_session_.request_stop()) {
      return;
    }
    handle = search_goal_handle_;
    generation = finding_session_.generation();
  }
  finding_button_->setEnabled(false);
  finding_button_->setText(tr("Stop Finding"));
  finding_status_->setText(tr("cancelling search"));

  if (handle && search_client_) {
    try {
      search_client_->async_cancel_goal(handle);
    } catch (const std::exception & error) {
      const auto status =
        tr("action cancellation request failed: %1").arg(error.what());
      QMetaObject::invokeMethod(
        this,
        [this, weak_lifetime, generation, status]() {
          const auto lifetime = weak_lifetime.lock();
          if (!lifetime) {
            return;
          }
          std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
          if (!lifetime->alive) {
            return;
          }
          std::lock_guard<std::mutex> lock(finding_mutex_);
          if (finding_session_.generation() == generation &&
            finding_session_.active())
          {
            finding_status_->setText(status);
          }
        },
        Qt::QueuedConnection);
    }
  }
  if (!cancel_search_client_ || !cancel_search_client_->service_is_ready()) {
    return;
  }
  cancel_search_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this, weak_lifetime, generation](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
    {
      const auto lifetime = weak_lifetime.lock();
      if (!lifetime) {
        return;
      }
      std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      try {
        const auto response = future.get();
        if (!response->success) {
          const auto status = tr("active-search cancellation service failed: %1").arg(
            QString::fromStdString(response->message));
          QMetaObject::invokeMethod(
            this,
            [this, weak_lifetime, generation, status]() {
              const auto lifetime = weak_lifetime.lock();
              if (!lifetime) {
                return;
              }
              std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
              if (!lifetime->alive) {
                return;
              }
              std::lock_guard<std::mutex> lock(finding_mutex_);
              if (finding_session_.generation() == generation &&
                finding_session_.active())
              {
                finding_status_->setText(status);
              }
            },
            Qt::QueuedConnection);
        }
      } catch (const std::exception & error) {
        const auto status =
          tr("active-search cancellation service failed: %1").arg(error.what());
        QMetaObject::invokeMethod(
          this,
          [this, weak_lifetime, generation, status]() {
            const auto lifetime = weak_lifetime.lock();
            if (!lifetime) {
              return;
            }
            std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
            if (!lifetime->alive) {
              return;
            }
            std::lock_guard<std::mutex> lock(finding_mutex_);
            if (finding_session_.generation() == generation &&
              finding_session_.active())
            {
              finding_status_->setText(status);
            }
          },
          Qt::QueuedConnection);
      }
    });
}

void SemanticSearchPanel::authorize_rotation(const std::uint64_t generation)
{
  const auto weak_lifetime =
    std::weak_ptr<CallbackLifetime>(callback_lifetime_);
  if (!authorize_rotation_client_ ||
    !authorize_rotation_client_->service_is_ready())
  {
    QMetaObject::invokeMethod(
      this,
      [this, weak_lifetime, generation]() {
        const auto lifetime = weak_lifetime.lock();
        if (!lifetime) {
          return;
        }
        std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
        if (!lifetime->alive) {
          return;
        }
        bool stop = false;
        {
          std::lock_guard<std::mutex> lock(finding_mutex_);
          if (finding_session_.generation() != generation ||
            finding_session_.state() != ActiveSearchState::AUTHORIZATION_PENDING)
          {
            return;
          }
          stop = finding_session_.on_authorization_result(generation, false);
        }
        if (stop) {
          stop_finding();
          finding_status_->setText(
            tr("active-search authorization service is unavailable"));
        }
      },
      Qt::QueuedConnection);
    return;
  }

  try {
    authorize_rotation_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this, weak_lifetime, generation](
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
      {
        const auto lifetime = weak_lifetime.lock();
        if (!lifetime) {
          return;
        }
        std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
        if (!lifetime->alive) {
          return;
        }
        try {
          const auto response = future.get();
          bool stop = false;
          {
            std::lock_guard<std::mutex> lock(finding_mutex_);
            if (finding_session_.generation() != generation ||
              !finding_session_.on_authorization_result(
                generation, response->success))
            {
              return;
            }
            stop = !response->success;
          }
          if (stop) {
            QMetaObject::invokeMethod(
              this,
              [this, weak_lifetime, generation,
                reason = QString::fromStdString(response->message)]() {
                const auto lifetime = weak_lifetime.lock();
                if (!lifetime) {
                  return;
                }
                std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
                if (!lifetime->alive) {
                  return;
                }
                {
                  std::lock_guard<std::mutex> lock(finding_mutex_);
                  if (finding_session_.generation() != generation ||
                    !finding_session_.active())
                  {
                    return;
                  }
                }
                stop_finding();
                finding_status_->setText(
                  tr("active-search authorization failed: %1").arg(reason));
              },
              Qt::QueuedConnection);
            return;
          }
          QMetaObject::invokeMethod(
            this,
            [this, weak_lifetime, generation]() {
              const auto lifetime = weak_lifetime.lock();
              if (!lifetime) {
                return;
              }
              std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
              if (!lifetime->alive) {
                return;
              }
              std::lock_guard<std::mutex> lock(finding_mutex_);
              if (finding_session_.generation() != generation ||
                !finding_session_.active())
              {
                return;
              }
              finding_status_->setText(tr("rotation authorized"));
            },
            Qt::QueuedConnection);
        } catch (const std::exception & error) {
          QMetaObject::invokeMethod(
            this,
            [this, weak_lifetime, generation,
              reason = QString::fromUtf8(error.what())]() {
              const auto lifetime = weak_lifetime.lock();
              if (!lifetime) {
                return;
              }
              std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
              if (!lifetime->alive) {
                return;
              }
              bool stop = false;
              {
                std::lock_guard<std::mutex> lock(finding_mutex_);
                if (finding_session_.generation() != generation ||
                  !finding_session_.on_authorization_result(generation, false))
                {
                  return;
                }
                stop = true;
              }
              if (stop) {
                {
                  std::lock_guard<std::mutex> lock(finding_mutex_);
                  if (finding_session_.generation() != generation ||
                    !finding_session_.active())
                  {
                    return;
                  }
                }
                stop_finding();
                finding_status_->setText(
                  tr("active-search authorization failed: %1").arg(reason));
              }
            },
            Qt::QueuedConnection);
        }
      });
  } catch (const std::exception & error) {
    QMetaObject::invokeMethod(
      this,
      [this, weak_lifetime, generation,
        reason = QString::fromUtf8(error.what())]() {
        const auto lifetime = weak_lifetime.lock();
        if (!lifetime) {
          return;
        }
        std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
        if (!lifetime->alive) {
          return;
        }
        bool stop = false;
        {
          std::lock_guard<std::mutex> lock(finding_mutex_);
          if (finding_session_.generation() != generation ||
            !finding_session_.on_authorization_result(generation, false))
          {
            return;
          }
          stop = true;
        }
        if (stop) {
          {
            std::lock_guard<std::mutex> lock(finding_mutex_);
            if (finding_session_.generation() != generation ||
              !finding_session_.active())
            {
              return;
            }
          }
          stop_finding();
          finding_status_->setText(
            tr("active-search authorization failed: %1").arg(reason));
        }
      },
      Qt::QueuedConnection);
  }
}

void SemanticSearchPanel::render_finding_state(const QString & status)
{
  const auto weak_lifetime =
    std::weak_ptr<CallbackLifetime>(callback_lifetime_);
  QMetaObject::invokeMethod(
    this,
    [this, weak_lifetime, status]() {
      const auto lifetime = weak_lifetime.lock();
      if (!lifetime) {
        return;
      }
      std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
      if (lifetime->alive) {
        finding_status_->setText(status);
      }
    },
    Qt::QueuedConnection);
}

void SemanticSearchPanel::finish_finding(
  const std::uint64_t generation,
  const QString & status)
{
  const auto weak_lifetime =
    std::weak_ptr<CallbackLifetime>(callback_lifetime_);
  QMetaObject::invokeMethod(
    this,
    [this, weak_lifetime, generation, status]() {
      const auto lifetime = weak_lifetime.lock();
      if (!lifetime) {
        return;
      }
      std::lock_guard<std::mutex> callback_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      std::lock_guard<std::mutex> lock(finding_mutex_);
      if (!finding_session_.finish(generation)) {
        return;
      }
      search_goal_handle_.reset();
      finding_button_->setText(tr("Start Finding"));
      finding_button_->setEnabled(true);
      finding_status_->setText(status);
    },
    Qt::QueuedConnection);
}

void SemanticSearchPanel::start_approach()
{
  std::optional<TargetReference> reference;
  {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    if (selected_reference_.has_value()) {
      reference = selected_reference_;
      approach_request_active_ = true;
    }
  }
  if (!reference.has_value()) {
    motion_status_->setText(
      tr("selected target is not ready"));
    start_approach_button_->setEnabled(false);
    return;
  }
  if (!authorize_client_ || !authorize_client_->service_is_ready()) {
    motion_status_->setText(tr("authorization service is unavailable"));
    return;
  }
  auto request = std::make_shared<
    track_robot_interfaces::srv::AuthorizeSemanticApproach::Request>();
  request->memory_epoch_id = reference->memory_epoch_id;
  request->global_object_id = reference->global_object_id;
  request->localization_epoch_id = reference->localization_epoch_id;
  request->query_id = reference->query_id;
  request->query_version = reference->query_version;
  request->snapshot_sequence = reference->snapshot_sequence;
  start_approach_button_->setEnabled(false);
  motion_status_->setText(tr("starting approach"));
  authorize_client_->async_send_request(
    request,
    [this](
      rclcpp::Client<
        track_robot_interfaces::srv::AuthorizeSemanticApproach>::
      SharedFuture future)
    {
      try {
        const auto response = future.get();
        if (!response->accepted) {
          {
            std::lock_guard<std::mutex> lock(reference_mutex_);
            approach_request_active_ = false;
          }
          refresh_authorization_state();
        }
        queue_label(
          motion_status_,
          response->accepted ?
          tr("approach enabled (supervised)") :
          tr("rejected: %1").arg(QString::fromStdString(response->reason)));
      } catch (const std::exception & error) {
        {
          std::lock_guard<std::mutex> lock(reference_mutex_);
          approach_request_active_ = false;
        }
        refresh_authorization_state();
        queue_label(
          motion_status_,
          tr("authorization call failed: %1").arg(error.what()));
      }
    });
}

void SemanticSearchPanel::cancel_and_disarm()
{
  {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    approach_request_active_ = false;
  }
  if (!cancel_disarm_client_ ||
    !cancel_disarm_client_->service_is_ready())
  {
    motion_status_->setText(tr("cancel/disarm service is unavailable"));
    return;
  }
  motion_status_->setText(tr("cancel and disarm pending"));
  cancel_disarm_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      try {
        const auto response = future.get();
        refresh_authorization_state();
        queue_label(
          motion_status_,
          tr("%1: %2")
          .arg(response->success ? tr("accepted") : tr("failed"))
          .arg(QString::fromStdString(response->message)));
      } catch (const std::exception & error) {
        refresh_authorization_state();
        queue_label(
          motion_status_,
          tr("cancel/disarm call failed: %1").arg(error.what()));
      }
    });
}

void SemanticSearchPanel::publish_query(bool revision)
{
  if (!query_publisher_) {
    acknowledgement_status_->setText(tr("query publisher is unavailable"));
    return;
  }
  try {
    QueryCommand command;
    {
      std::lock_guard<std::mutex> session_lock(session_mutex_);
      if (revision) {
        command = session_.revise(query_input_->text());
      } else {
        const auto now = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        const auto seed = now > 0 ?
          static_cast<std::uint64_t>(now) : std::uint64_t{1U};
        command = session_.new_query(query_input_->text(), seed);
      }
    }
    std_msgs::msg::String message;
    message.data = command.payload;
    query_publisher_->publish(message);
    {
      std::lock_guard<std::mutex> lock(reference_mutex_);
      best_reference_.reset();
      selected_reference_.reset();
      approach_request_active_ = false;
    }
    refresh_authorization_state();
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
  std::optional<DiagnosticMatch> matched;
  {
    std::lock_guard<std::mutex> session_lock(session_mutex_);
    matched = session_.correlate_diagnostic(message->data);
  }
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
  std::optional<QueryCommand> current;
  {
    std::lock_guard<std::mutex> session_lock(session_mutex_);
    current = session_.current();
  }
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
  {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    best_reference_ = reference_from(*message);
  }
  refresh_authorization_state();
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

void SemanticSearchPanel::on_selected_target(
  const track_robot_interfaces::msg::SemanticObjectArray::SharedPtr message)
{
  {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    selected_reference_ = reference_from(*message);
  }
  refresh_authorization_state();
}

void SemanticSearchPanel::refresh_authorization_state()
{
  bool ready = false;
  bool request_active = false;
  std::uint64_t object_id = 0U;
  {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    ready = selected_reference_.has_value();
    if (ready) {
      object_id = selected_reference_->global_object_id;
    }
    request_active = approach_request_active_;
  }
  QMetaObject::invokeMethod(
    this,
    [this, ready, request_active, object_id]() {
      if (request_active) {
        start_approach_button_->setEnabled(false);
        return;
      }
      start_approach_button_->setEnabled(ready);
      if (ready) {
        motion_status_->setText(
          tr("ready for operator authorization: object %1").arg(object_id));
      } else {
        motion_status_->setText(
          tr("selected target is not ready"));
      }
    },
    Qt::QueuedConnection);
}

void SemanticSearchPanel::on_diagnostic_ranking(
  const track_robot_interfaces::msg::SemanticObjectArray::SharedPtr message)
{
  if (message->objects.empty()) {
    queue_label(
      diagnostic_ranking_status_,
      tr("none — valid abstention  [epoch=%1]").arg(message->memory_epoch_id));
    return;
  }
  const auto & candidate = message->objects.front();
  queue_label(
    diagnostic_ranking_status_,
    tr("#1 object=%1  query=%2/v%3  support=%4  relevance=%5  count=%6")
    .arg(candidate.global_object_id)
    .arg(candidate.active_query_id)
    .arg(candidate.active_query_version)
    .arg(support_name(candidate.support_state))
    .arg(static_cast<double>(candidate.task_relevance), 0, 'f', 3)
    .arg(message->objects.size()));
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
  if (config.mapGetString("selected_target_topic", &value)) {
    selected_target_topic_ = value.toStdString();
  }
  if (config.mapGetString("diagnostic_ranking_topic", &value)) {
    diagnostic_ranking_topic_ = value.toStdString();
  }
  if (config.mapGetString("authorize_service", &value)) {
    authorize_service_ = value.toStdString();
  }
  if (config.mapGetString("cancel_disarm_service", &value)) {
    cancel_disarm_service_ = value.toStdString();
  }
  if (config.mapGetString("search_action", &value)) {
    search_action_ = value.toStdString();
  }
  if (config.mapGetString("authorize_rotation_service", &value)) {
    authorize_rotation_service_ = value.toStdString();
  }
  if (config.mapGetString("cancel_search_service", &value)) {
    cancel_search_service_ = value.toStdString();
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
  config.mapSetValue(
    "selected_target_topic", QString::fromStdString(selected_target_topic_));
  config.mapSetValue(
    "diagnostic_ranking_topic",
    QString::fromStdString(diagnostic_ranking_topic_));
  config.mapSetValue(
    "authorize_service", QString::fromStdString(authorize_service_));
  config.mapSetValue(
    "cancel_disarm_service", QString::fromStdString(cancel_disarm_service_));
  config.mapSetValue("search_action", QString::fromStdString(search_action_));
  config.mapSetValue(
    "authorize_rotation_service",
    QString::fromStdString(authorize_rotation_service_));
  config.mapSetValue(
    "cancel_search_service", QString::fromStdString(cancel_search_service_));
}

}  // namespace track_robot_semantic_search_rviz_plugins

PLUGINLIB_EXPORT_CLASS(
  track_robot_semantic_search_rviz_plugins::SemanticSearchPanel,
  rviz_common::Panel)
