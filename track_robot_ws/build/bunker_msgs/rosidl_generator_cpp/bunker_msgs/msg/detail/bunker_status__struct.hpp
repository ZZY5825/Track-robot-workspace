// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from bunker_msgs:msg/BunkerStatus.idl
// generated code does not contain a copyright notice

#ifndef BUNKER_MSGS__MSG__DETAIL__BUNKER_STATUS__STRUCT_HPP_
#define BUNKER_MSGS__MSG__DETAIL__BUNKER_STATUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'actuator_states'
#include "bunker_msgs/msg/detail/bunker_actuator_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__bunker_msgs__msg__BunkerStatus __attribute__((deprecated))
#else
# define DEPRECATED__bunker_msgs__msg__BunkerStatus __declspec(deprecated)
#endif

namespace bunker_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BunkerStatus_
{
  using Type = BunkerStatus_<ContainerAllocator>;

  explicit BunkerStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity = 0.0;
      this->angular_velocity = 0.0;
      this->vehicle_state = 0;
      this->control_mode = 0;
      this->error_code = 0;
      this->battery_voltage = 0.0;
      this->actuator_states.fill(bunker_msgs::msg::BunkerActuatorState_<ContainerAllocator>{_init});
    }
  }

  explicit BunkerStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    actuator_states(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity = 0.0;
      this->angular_velocity = 0.0;
      this->vehicle_state = 0;
      this->control_mode = 0;
      this->error_code = 0;
      this->battery_voltage = 0.0;
      this->actuator_states.fill(bunker_msgs::msg::BunkerActuatorState_<ContainerAllocator>{_alloc, _init});
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _linear_velocity_type =
    double;
  _linear_velocity_type linear_velocity;
  using _angular_velocity_type =
    double;
  _angular_velocity_type angular_velocity;
  using _vehicle_state_type =
    uint8_t;
  _vehicle_state_type vehicle_state;
  using _control_mode_type =
    uint8_t;
  _control_mode_type control_mode;
  using _error_code_type =
    uint16_t;
  _error_code_type error_code;
  using _battery_voltage_type =
    double;
  _battery_voltage_type battery_voltage;
  using _actuator_states_type =
    std::array<bunker_msgs::msg::BunkerActuatorState_<ContainerAllocator>, 3>;
  _actuator_states_type actuator_states;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__linear_velocity(
    const double & _arg)
  {
    this->linear_velocity = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const double & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }
  Type & set__vehicle_state(
    const uint8_t & _arg)
  {
    this->vehicle_state = _arg;
    return *this;
  }
  Type & set__control_mode(
    const uint8_t & _arg)
  {
    this->control_mode = _arg;
    return *this;
  }
  Type & set__error_code(
    const uint16_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__battery_voltage(
    const double & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__actuator_states(
    const std::array<bunker_msgs::msg::BunkerActuatorState_<ContainerAllocator>, 3> & _arg)
  {
    this->actuator_states = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t MOTOR_ID_FRONT_RIGHT =
    0;
  static constexpr int8_t MOTOR_ID_FRONT_LEFT =
    1;
  static constexpr int8_t MOTOR_ID_REAR_RIGHT =
    2;
  static constexpr int8_t MOTOR_ID_REAR_LEFT =
    3;
  static constexpr int8_t LIGHT_ID_FRONT =
    0;
  static constexpr int8_t LIGHT_ID_REAR =
    1;

  // pointer types
  using RawPtr =
    bunker_msgs::msg::BunkerStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const bunker_msgs::msg::BunkerStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<bunker_msgs::msg::BunkerStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<bunker_msgs::msg::BunkerStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      bunker_msgs::msg::BunkerStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<bunker_msgs::msg::BunkerStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      bunker_msgs::msg::BunkerStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<bunker_msgs::msg::BunkerStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<bunker_msgs::msg::BunkerStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<bunker_msgs::msg::BunkerStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__bunker_msgs__msg__BunkerStatus
    std::shared_ptr<bunker_msgs::msg::BunkerStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__bunker_msgs__msg__BunkerStatus
    std::shared_ptr<bunker_msgs::msg::BunkerStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BunkerStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->linear_velocity != other.linear_velocity) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    if (this->vehicle_state != other.vehicle_state) {
      return false;
    }
    if (this->control_mode != other.control_mode) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->actuator_states != other.actuator_states) {
      return false;
    }
    return true;
  }
  bool operator!=(const BunkerStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BunkerStatus_

// alias to use template instance with default allocator
using BunkerStatus =
  bunker_msgs::msg::BunkerStatus_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t BunkerStatus_<ContainerAllocator>::MOTOR_ID_FRONT_RIGHT;
template<typename ContainerAllocator>
constexpr int8_t BunkerStatus_<ContainerAllocator>::MOTOR_ID_FRONT_LEFT;
template<typename ContainerAllocator>
constexpr int8_t BunkerStatus_<ContainerAllocator>::MOTOR_ID_REAR_RIGHT;
template<typename ContainerAllocator>
constexpr int8_t BunkerStatus_<ContainerAllocator>::MOTOR_ID_REAR_LEFT;
template<typename ContainerAllocator>
constexpr int8_t BunkerStatus_<ContainerAllocator>::LIGHT_ID_FRONT;
template<typename ContainerAllocator>
constexpr int8_t BunkerStatus_<ContainerAllocator>::LIGHT_ID_REAR;

}  // namespace msg

}  // namespace bunker_msgs

#endif  // BUNKER_MSGS__MSG__DETAIL__BUNKER_STATUS__STRUCT_HPP_
