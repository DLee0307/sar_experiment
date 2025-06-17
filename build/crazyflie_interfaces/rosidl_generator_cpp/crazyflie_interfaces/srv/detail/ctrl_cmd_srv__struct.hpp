// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crazyflie_interfaces:srv/CTRLCmdSrv.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__CTRL_CMD_SRV__STRUCT_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__CTRL_CMD_SRV__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'cmd_vals'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__crazyflie_interfaces__srv__CTRLCmdSrv_Request __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_interfaces__srv__CTRLCmdSrv_Request __declspec(deprecated)
#endif

namespace crazyflie_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CTRLCmdSrv_Request_
{
  using Type = CTRLCmdSrv_Request_<ContainerAllocator>;

  explicit CTRLCmdSrv_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : cmd_vals(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd_type = 0;
      this->cmd_flag = 0;
      this->cmd_rx = false;
    }
  }

  explicit CTRLCmdSrv_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : cmd_vals(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd_type = 0;
      this->cmd_flag = 0;
      this->cmd_rx = false;
    }
  }

  // field types and members
  using _cmd_type_type =
    uint16_t;
  _cmd_type_type cmd_type;
  using _cmd_vals_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _cmd_vals_type cmd_vals;
  using _cmd_flag_type =
    uint16_t;
  _cmd_flag_type cmd_flag;
  using _cmd_rx_type =
    bool;
  _cmd_rx_type cmd_rx;

  // setters for named parameter idiom
  Type & set__cmd_type(
    const uint16_t & _arg)
  {
    this->cmd_type = _arg;
    return *this;
  }
  Type & set__cmd_vals(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->cmd_vals = _arg;
    return *this;
  }
  Type & set__cmd_flag(
    const uint16_t & _arg)
  {
    this->cmd_flag = _arg;
    return *this;
  }
  Type & set__cmd_rx(
    const bool & _arg)
  {
    this->cmd_rx = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crazyflie_interfaces::srv::CTRLCmdSrv_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_interfaces::srv::CTRLCmdSrv_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::CTRLCmdSrv_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::CTRLCmdSrv_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_interfaces__srv__CTRLCmdSrv_Request
    std::shared_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_interfaces__srv__CTRLCmdSrv_Request
    std::shared_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CTRLCmdSrv_Request_ & other) const
  {
    if (this->cmd_type != other.cmd_type) {
      return false;
    }
    if (this->cmd_vals != other.cmd_vals) {
      return false;
    }
    if (this->cmd_flag != other.cmd_flag) {
      return false;
    }
    if (this->cmd_rx != other.cmd_rx) {
      return false;
    }
    return true;
  }
  bool operator!=(const CTRLCmdSrv_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CTRLCmdSrv_Request_

// alias to use template instance with default allocator
using CTRLCmdSrv_Request =
  crazyflie_interfaces::srv::CTRLCmdSrv_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace crazyflie_interfaces


#ifndef _WIN32
# define DEPRECATED__crazyflie_interfaces__srv__CTRLCmdSrv_Response __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_interfaces__srv__CTRLCmdSrv_Response __declspec(deprecated)
#endif

namespace crazyflie_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CTRLCmdSrv_Response_
{
  using Type = CTRLCmdSrv_Response_<ContainerAllocator>;

  explicit CTRLCmdSrv_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->srv_success = false;
    }
  }

  explicit CTRLCmdSrv_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->srv_success = false;
    }
  }

  // field types and members
  using _srv_success_type =
    bool;
  _srv_success_type srv_success;

  // setters for named parameter idiom
  Type & set__srv_success(
    const bool & _arg)
  {
    this->srv_success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crazyflie_interfaces::srv::CTRLCmdSrv_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_interfaces::srv::CTRLCmdSrv_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::CTRLCmdSrv_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::CTRLCmdSrv_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_interfaces__srv__CTRLCmdSrv_Response
    std::shared_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_interfaces__srv__CTRLCmdSrv_Response
    std::shared_ptr<crazyflie_interfaces::srv::CTRLCmdSrv_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CTRLCmdSrv_Response_ & other) const
  {
    if (this->srv_success != other.srv_success) {
      return false;
    }
    return true;
  }
  bool operator!=(const CTRLCmdSrv_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CTRLCmdSrv_Response_

// alias to use template instance with default allocator
using CTRLCmdSrv_Response =
  crazyflie_interfaces::srv::CTRLCmdSrv_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace crazyflie_interfaces

namespace crazyflie_interfaces
{

namespace srv
{

struct CTRLCmdSrv
{
  using Request = crazyflie_interfaces::srv::CTRLCmdSrv_Request;
  using Response = crazyflie_interfaces::srv::CTRLCmdSrv_Response;
};

}  // namespace srv

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__CTRL_CMD_SRV__STRUCT_HPP_
