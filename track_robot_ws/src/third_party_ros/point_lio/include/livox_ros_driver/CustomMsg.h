#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <std_msgs/msg/header.hpp>

namespace livox_ros_driver
{

struct CustomPoint
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  uint8_t reflectivity = 0;
  uint8_t tag = 0;
  uint8_t line = 0;
  uint32_t offset_time = 0;
};

struct CustomMsg
{
  using SharedPtr = std::shared_ptr<CustomMsg>;
  using ConstSharedPtr = std::shared_ptr<const CustomMsg>;
  using Ptr = SharedPtr;
  using ConstPtr = ConstSharedPtr;

  std_msgs::msg::Header header;
  uint32_t point_num = 0;
  std::vector<CustomPoint> points;
};

}  // namespace livox_ros_driver
