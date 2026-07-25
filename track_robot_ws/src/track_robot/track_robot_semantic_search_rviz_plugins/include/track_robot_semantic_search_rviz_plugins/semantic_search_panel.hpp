#pragma once

#include <memory>
#include <string>

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"
#include "track_robot_interfaces/msg/semantic_object_array.hpp"
#include "track_robot_interfaces/msg/semantic_region_array.hpp"
#include "track_robot_semantic_search_rviz_plugins/query_session.hpp"

namespace track_robot_semantic_search_rviz_plugins
{

class SemanticSearchPanel final : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SemanticSearchPanel(QWidget * parent = nullptr);
  ~SemanticSearchPanel() override = default;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void submit_new_query();
  void submit_revision();

private:
  void publish_query(bool revision);
  void on_diagnostic(const std_msgs::msg::String::SharedPtr message);
  void on_regions(
    const track_robot_interfaces::msg::SemanticRegionArray::SharedPtr message);
  void on_active_objects(
    const track_robot_interfaces::msg::SemanticObjectArray::SharedPtr message);
  void on_best_candidate(
    const track_robot_interfaces::msg::SemanticObjectArray::SharedPtr message);
  void queue_label(QLabel * label, const QString & value);

  QuerySession session_;
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

  QLineEdit * query_input_{nullptr};
  QPushButton * new_button_{nullptr};
  QPushButton * revise_button_{nullptr};
  QLabel * query_status_{nullptr};
  QLabel * model_status_{nullptr};
  QLabel * acknowledgement_status_{nullptr};
  QLabel * region_status_{nullptr};
  QLabel * object_status_{nullptr};
  QLabel * best_status_{nullptr};

  std::string query_topic_;
  std::string diagnostic_topic_;
  std::string regions_topic_;
  std::string active_objects_topic_;
  std::string best_candidate_topic_;
};

}  // namespace track_robot_semantic_search_rviz_plugins
