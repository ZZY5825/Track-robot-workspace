#pragma once

#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace tf
{

class Vector3
{
public:
  Vector3() = default;
  Vector3(double x, double y, double z) : x_(x), y_(y), z_(z) {}
  double x_ = 0.0;
  double y_ = 0.0;
  double z_ = 0.0;
};

class Quaternion
{
public:
  void setW(double w) { w_ = w; }
  void setX(double x) { x_ = x; }
  void setY(double y) { y_ = y; }
  void setZ(double z) { z_ = z; }
  double x_ = 0.0;
  double y_ = 0.0;
  double z_ = 0.0;
  double w_ = 1.0;
};

class Transform
{
public:
  void setOrigin(const Vector3 & origin) { origin_ = origin; }
  void setRotation(const Quaternion & rotation) { rotation_ = rotation; }

  Vector3 origin_;
  Quaternion rotation_;
};

class StampedTransform
{
public:
  StampedTransform(
    const Transform & transform,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & frame_id,
    const std::string & child_frame_id)
  {
    msg_.header.stamp = stamp;
    msg_.header.frame_id = frame_id;
    msg_.child_frame_id = child_frame_id;
    msg_.transform.translation.x = transform.origin_.x_;
    msg_.transform.translation.y = transform.origin_.y_;
    msg_.transform.translation.z = transform.origin_.z_;
    msg_.transform.rotation.x = transform.rotation_.x_;
    msg_.transform.rotation.y = transform.rotation_.y_;
    msg_.transform.rotation.z = transform.rotation_.z_;
    msg_.transform.rotation.w = transform.rotation_.w_;
  }

  geometry_msgs::msg::TransformStamped msg_;
};

}  // namespace tf
