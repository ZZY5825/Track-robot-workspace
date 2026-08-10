#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "track_robot_interfaces/action/search_for_object.hpp"
#include "track_robot_interfaces/msg/semantic_object_array.hpp"
#include "track_robot_interfaces/msg/semantic_region_array.hpp"
#include "track_robot_interfaces/srv/authorize_semantic_approach.hpp"
#include "track_robot_semantic_search_rviz_plugins/active_search_session.hpp"
#include "track_robot_semantic_search_rviz_plugins/query_session.hpp"

namespace track_robot_semantic_search_rviz_plugins
{

using SearchForObject = track_robot_interfaces::action::SearchForObject;
using SearchGoalHandle = rclcpp_action::ClientGoalHandle<SearchForObject>;

class SemanticSearchPanel final : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SemanticSearchPanel(QWidget * parent = nullptr);
  ~SemanticSearchPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void submit_new_query();
  void submit_revision();
  void toggle_finding();
  void start_approach();
  void cancel_and_disarm();

private:
  struct CallbackLifetime
  {
    std::mutex mutex;
    bool alive{true};
  };

  struct TargetReference
  {
    std::uint64_t memory_epoch_id{0U};
    std::uint64_t global_object_id{0U};
    std::uint64_t localization_epoch_id{0U};
    std::uint64_t query_id{0U};
    std::uint64_t query_version{0U};
    std::uint64_t snapshot_sequence{0U};

    bool same_identity(const TargetReference & other) const;
    bool complete() const;
  };

  static std::optional<TargetReference> reference_from(
    const track_robot_interfaces::msg::SemanticObjectArray & message);
  void publish_query(bool revision);
  void on_diagnostic(const std_msgs::msg::String::SharedPtr message);
  void on_regions(
    const track_robot_interfaces::msg::SemanticRegionArray::SharedPtr message);
  void on_active_objects(
    const track_robot_interfaces::msg::SemanticObjectArray::SharedPtr message);
  void on_best_candidate(
    const track_robot_interfaces::msg::SemanticObjectArray::SharedPtr message);
  void on_selected_target(
    const track_robot_interfaces::msg::SemanticObjectArray::SharedPtr message);
  void on_diagnostic_ranking(
    const track_robot_interfaces::msg::SemanticObjectArray::SharedPtr message);
  void on_navigation_diagnostics(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message);
  void start_finding();
  void stop_finding();
  void send_action_cancel(
    std::uint64_t generation,
    const SearchGoalHandle::SharedPtr & handle);
  void send_explicit_cancel(std::uint64_t generation);
  void render_finding_state(const QString & status);
  void render_finding_state(std::uint64_t generation, const QString & status);
  void finish_finding(std::uint64_t generation, const QString & status);
  void set_query_controls_enabled(bool enabled);
  void refresh_authorization_state();
  void queue_label(QLabel * label, const QString & value);

  QuerySession session_;
  std::mutex session_mutex_;
  std::shared_ptr<CallbackLifetime> callback_lifetime_{
    std::make_shared<CallbackLifetime>()};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr query_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    diagnostic_subscription_;
  rclcpp::Subscription<
    track_robot_interfaces::msg::SemanticRegionArray>::SharedPtr
    region_subscription_;
  rclcpp::Subscription<
    track_robot_interfaces::msg::SemanticObjectArray>::SharedPtr
    object_subscription_;
  rclcpp::Subscription<
    track_robot_interfaces::msg::SemanticObjectArray>::SharedPtr
    best_subscription_;
  rclcpp::Subscription<
    track_robot_interfaces::msg::SemanticObjectArray>::SharedPtr
    selected_target_subscription_;
  rclcpp::Subscription<
    track_robot_interfaces::msg::SemanticObjectArray>::SharedPtr
    diagnostic_ranking_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    navigation_diagnostics_subscription_;
  rclcpp::Client<
    track_robot_interfaces::srv::AuthorizeSemanticApproach>::SharedPtr
    authorize_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_disarm_client_;
  rclcpp_action::Client<SearchForObject>::SharedPtr search_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_search_client_;
  SearchGoalHandle::SharedPtr search_goal_handle_;
  ActiveSearchSession finding_session_;
  std::mutex finding_mutex_;

  QLineEdit * query_input_{nullptr};
  QPushButton * new_button_{nullptr};
  QPushButton * revise_button_{nullptr};
  QPushButton * finding_button_{nullptr};
  QPushButton * start_approach_button_{nullptr};
  QPushButton * cancel_disarm_button_{nullptr};
  QLabel * query_status_{nullptr};
  QLabel * model_status_{nullptr};
  QLabel * acknowledgement_status_{nullptr};
  QLabel * region_status_{nullptr};
  QLabel * object_status_{nullptr};
  QLabel * best_status_{nullptr};
  QLabel * diagnostic_ranking_status_{nullptr};
  QLabel * finding_status_{nullptr};
  QLabel * motion_status_{nullptr};
  QLabel * recovery_status_{nullptr};

  std::mutex reference_mutex_;
  std::optional<TargetReference> best_reference_;
  std::optional<TargetReference> selected_reference_;
  bool approach_request_active_{false};

  std::string query_topic_;
  std::string diagnostic_topic_;
  std::string regions_topic_;
  std::string active_objects_topic_;
  std::string best_candidate_topic_;
  std::string selected_target_topic_;
  std::string diagnostic_ranking_topic_;
  std::string navigation_diagnostics_topic_;
  std::string authorize_service_;
  std::string cancel_disarm_service_;
  std::string search_action_;
  std::string cancel_search_service_;
};

}  // namespace track_robot_semantic_search_rviz_plugins
