#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace {

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

bool finitePositive(float x, float y, float z, double min_range, double max_range) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;
  const double range = std::sqrt(static_cast<double>(x) * x + static_cast<double>(y) * y +
                                 static_cast<double>(z) * z);
  return range >= min_range && range <= max_range;
}

size_t pointOffset(const sensor_msgs::msg::PointCloud2 &cloud, size_t point_index) {
  const size_t row = point_index / cloud.width;
  const size_t col = point_index % cloud.width;
  return row * cloud.row_step + col * cloud.point_step;
}

const sensor_msgs::msg::PointField *findFloat32Field(
    const sensor_msgs::msg::PointCloud2 &cloud, const std::string &name) {
  const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                                  [&name](const auto &f) { return f.name == name; });
  if (field == cloud.fields.end() ||
      field->datatype != sensor_msgs::msg::PointField::FLOAT32) {
    return nullptr;
  }
  return &(*field);
}

sensor_msgs::msg::PointCloud2 makeSubsetCloud(const sensor_msgs::msg::PointCloud2 &src,
                                              const std::vector<size_t> &indices) {
  sensor_msgs::msg::PointCloud2 dst;
  dst.header = src.header;
  dst.fields = src.fields;
  dst.is_bigendian = src.is_bigendian;
  dst.point_step = src.point_step;
  dst.height = 1;
  dst.width = static_cast<uint32_t>(indices.size());
  dst.row_step = dst.point_step * dst.width;
  dst.is_dense = false;
  dst.data.resize(static_cast<size_t>(dst.row_step) * dst.height);

  for (size_t out_idx = 0; out_idx < indices.size(); ++out_idx) {
    const size_t src_offset = pointOffset(src, indices[out_idx]);
    const size_t dst_offset = out_idx * dst.point_step;
    std::copy_n(src.data.begin() + static_cast<std::ptrdiff_t>(src_offset), src.point_step,
                dst.data.begin() + static_cast<std::ptrdiff_t>(dst_offset));
  }
  return dst;
}

sensor_msgs::msg::PointCloud2 makeMaskedDebugCloud(const sensor_msgs::msg::PointCloud2 &src,
                                                   const std::vector<size_t> &visible_indices) {
  sensor_msgs::msg::PointCloud2 dst = src;
  dst.is_dense = false;

  const auto *x_field = findFloat32Field(dst, "x");
  const auto *y_field = findFloat32Field(dst, "y");
  const auto *z_field = findFloat32Field(dst, "z");
  if (!x_field || !y_field || !z_field) {
    return makeSubsetCloud(src, visible_indices);
  }

  const size_t n_points = static_cast<size_t>(dst.height) * static_cast<size_t>(dst.width);
  std::vector<uint8_t> visible(n_points, 0);
  for (const auto idx : visible_indices) {
    if (idx < visible.size()) visible[idx] = 1;
  }

  const float quiet_nan = std::numeric_limits<float>::quiet_NaN();
  for (size_t i = 0; i < n_points; ++i) {
    if (visible[i]) continue;
    const size_t offset = pointOffset(dst, i);
    if (offset + dst.point_step > dst.data.size()) continue;
    std::memcpy(dst.data.data() + offset + x_field->offset, &quiet_nan, sizeof(float));
    std::memcpy(dst.data.data() + offset + y_field->offset, &quiet_nan, sizeof(float));
    std::memcpy(dst.data.data() + offset + z_field->offset, &quiet_nan, sizeof(float));
  }

  return dst;
}

}  // namespace

class RangeImageMosFilter : public rclcpp::Node {
 public:
  RangeImageMosFilter() : Node("range_image_mos_filter") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/rslidar_points");
    static_topic_ = declare_parameter<std::string>("static_topic", "/rslidar_points_static");
    dynamic_topic_ =
        declare_parameter<std::string>("dynamic_topic", "/rslidar_points_dynamic_debug");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    use_odom_ = declare_parameter<bool>("use_odom", true);
    filter_only_when_stationary_ =
        declare_parameter<bool>("filter_only_when_stationary", true);

    min_range_ = declare_parameter<double>("min_range", 0.8);
    max_range_ = declare_parameter<double>("max_range", 30.0);
    foreground_margin_ = declare_parameter<double>("foreground_margin", 0.45);
    background_match_tolerance_ = declare_parameter<double>("background_match_tolerance", 0.25);
    background_update_alpha_ = declare_parameter<double>("background_update_alpha", 0.05);
    min_background_observations_ = declare_parameter<int>("min_background_observations", 4);
    stationary_translation_threshold_ =
        declare_parameter<double>("stationary_translation_threshold", 0.03);
    stationary_yaw_threshold_deg_ = declare_parameter<double>("stationary_yaw_threshold_deg", 1.0);
    publish_dynamic_debug_ = declare_parameter<bool>("publish_dynamic_debug", true);

    rclcpp::SensorDataQoS qos;
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, qos, std::bind(&RangeImageMosFilter::cloudCallback, this,
                                     std::placeholders::_1));
    if (use_odom_) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          odom_topic_, 50,
          std::bind(&RangeImageMosFilter::odomCallback, this, std::placeholders::_1));
    }
    static_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(static_topic_, qos);
    dynamic_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(dynamic_topic_, qos);

    RCLCPP_INFO(get_logger(), "Range-image MOS filter: %s -> %s", input_topic_.c_str(),
                static_topic_.c_str());
  }

 private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto &p = msg->pose.pose.position;
    const double yaw = yawFromQuaternion(msg->pose.pose.orientation);

    if (!have_last_odom_) {
      last_odom_x_ = p.x;
      last_odom_y_ = p.y;
      last_odom_yaw_ = yaw;
      have_last_odom_ = true;
      stationary_ = true;
      return;
    }

    const double translation = std::hypot(p.x - last_odom_x_, p.y - last_odom_y_);
    const double yaw_delta = std::abs(normalizeAngle(yaw - last_odom_yaw_));
    const double yaw_threshold = stationary_yaw_threshold_deg_ * M_PI / 180.0;
    stationary_ = translation <= stationary_translation_threshold_ && yaw_delta <= yaw_threshold;

    last_odom_x_ = p.x;
    last_odom_y_ = p.y;
    last_odom_yaw_ = yaw;
  }

  void resizeBackground(size_t n) {
    if (background_range_.size() == n) return;
    background_range_.assign(n, std::numeric_limits<float>::infinity());
    background_observations_.assign(n, 0);
    RCLCPP_INFO(get_logger(), "Initialized background range image with %zu cells", n);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const size_t n_points = static_cast<size_t>(msg->height) * static_cast<size_t>(msg->width);
    resizeBackground(n_points);

    const bool can_filter = (!filter_only_when_stationary_ || stationary_) &&
                            (!use_odom_ || have_last_odom_ || !filter_only_when_stationary_);
    const bool can_update_background = (!use_odom_ || stationary_ || !have_last_odom_);

    std::vector<size_t> static_indices;
    std::vector<size_t> dynamic_indices;
    static_indices.reserve(n_points);
    dynamic_indices.reserve(n_points / 8);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (size_t i = 0; i < n_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (!finitePositive(x, y, z, min_range_, max_range_)) {
        continue;
      }

      const float range = std::sqrt(x * x + y * y + z * z);
      float &bg = background_range_[i];
      int &obs = background_observations_[i];

      const bool has_background = std::isfinite(bg) && obs >= min_background_observations_;
      const bool foreground_occluder = can_filter && has_background &&
                                      static_cast<double>(range) + foreground_margin_ < bg;

      if (foreground_occluder) {
        dynamic_indices.emplace_back(i);
        continue;
      }

      static_indices.emplace_back(i);

      if (can_update_background) {
        if (!std::isfinite(bg)) {
          bg = range;
          obs = 1;
        } else if (std::abs(static_cast<double>(range) - bg) <= background_match_tolerance_) {
          bg = static_cast<float>((1.0 - background_update_alpha_) * bg +
                                  background_update_alpha_ * range);
          obs = std::min(obs + 1, 1000000);
        } else if (range > bg + foreground_margin_) {
          // The old foreground object may have disappeared; adapt slowly toward the farther surface.
          bg = static_cast<float>((1.0 - background_update_alpha_) * bg +
                                  background_update_alpha_ * range);
        }
      }
    }

    static_pub_->publish(makeSubsetCloud(*msg, static_indices));
    if (publish_dynamic_debug_) {
      dynamic_pub_->publish(makeMaskedDebugCloud(*msg, dynamic_indices));
    }
  }

  std::string input_topic_;
  std::string static_topic_;
  std::string dynamic_topic_;
  std::string odom_topic_;
  bool use_odom_{true};
  bool filter_only_when_stationary_{true};
  bool publish_dynamic_debug_{true};

  double min_range_{0.8};
  double max_range_{30.0};
  double foreground_margin_{0.45};
  double background_match_tolerance_{0.25};
  double background_update_alpha_{0.05};
  int min_background_observations_{4};
  double stationary_translation_threshold_{0.03};
  double stationary_yaw_threshold_deg_{1.0};

  bool have_last_odom_{false};
  bool stationary_{true};
  double last_odom_x_{0.0};
  double last_odom_y_{0.0};
  double last_odom_yaw_{0.0};

  std::vector<float> background_range_;
  std::vector<int> background_observations_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RangeImageMosFilter>());
  rclcpp::shutdown();
  return 0;
}
