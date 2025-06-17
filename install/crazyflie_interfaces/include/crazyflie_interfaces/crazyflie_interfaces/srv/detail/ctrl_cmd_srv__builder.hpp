// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:srv/CTRLCmdSrv.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__CTRL_CMD_SRV__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__CTRL_CMD_SRV__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/srv/detail/ctrl_cmd_srv__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace srv
{

namespace builder
{

class Init_CTRLCmdSrv_Request_cmd_rx
{
public:
  explicit Init_CTRLCmdSrv_Request_cmd_rx(::crazyflie_interfaces::srv::CTRLCmdSrv_Request & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::srv::CTRLCmdSrv_Request cmd_rx(::crazyflie_interfaces::srv::CTRLCmdSrv_Request::_cmd_rx_type arg)
  {
    msg_.cmd_rx = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::srv::CTRLCmdSrv_Request msg_;
};

class Init_CTRLCmdSrv_Request_cmd_flag
{
public:
  explicit Init_CTRLCmdSrv_Request_cmd_flag(::crazyflie_interfaces::srv::CTRLCmdSrv_Request & msg)
  : msg_(msg)
  {}
  Init_CTRLCmdSrv_Request_cmd_rx cmd_flag(::crazyflie_interfaces::srv::CTRLCmdSrv_Request::_cmd_flag_type arg)
  {
    msg_.cmd_flag = std::move(arg);
    return Init_CTRLCmdSrv_Request_cmd_rx(msg_);
  }

private:
  ::crazyflie_interfaces::srv::CTRLCmdSrv_Request msg_;
};

class Init_CTRLCmdSrv_Request_cmd_vals
{
public:
  explicit Init_CTRLCmdSrv_Request_cmd_vals(::crazyflie_interfaces::srv::CTRLCmdSrv_Request & msg)
  : msg_(msg)
  {}
  Init_CTRLCmdSrv_Request_cmd_flag cmd_vals(::crazyflie_interfaces::srv::CTRLCmdSrv_Request::_cmd_vals_type arg)
  {
    msg_.cmd_vals = std::move(arg);
    return Init_CTRLCmdSrv_Request_cmd_flag(msg_);
  }

private:
  ::crazyflie_interfaces::srv::CTRLCmdSrv_Request msg_;
};

class Init_CTRLCmdSrv_Request_cmd_type
{
public:
  Init_CTRLCmdSrv_Request_cmd_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CTRLCmdSrv_Request_cmd_vals cmd_type(::crazyflie_interfaces::srv::CTRLCmdSrv_Request::_cmd_type_type arg)
  {
    msg_.cmd_type = std::move(arg);
    return Init_CTRLCmdSrv_Request_cmd_vals(msg_);
  }

private:
  ::crazyflie_interfaces::srv::CTRLCmdSrv_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::srv::CTRLCmdSrv_Request>()
{
  return crazyflie_interfaces::srv::builder::Init_CTRLCmdSrv_Request_cmd_type();
}

}  // namespace crazyflie_interfaces


namespace crazyflie_interfaces
{

namespace srv
{

namespace builder
{

class Init_CTRLCmdSrv_Response_srv_success
{
public:
  Init_CTRLCmdSrv_Response_srv_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::crazyflie_interfaces::srv::CTRLCmdSrv_Response srv_success(::crazyflie_interfaces::srv::CTRLCmdSrv_Response::_srv_success_type arg)
  {
    msg_.srv_success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::srv::CTRLCmdSrv_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::srv::CTRLCmdSrv_Response>()
{
  return crazyflie_interfaces::srv::builder::Init_CTRLCmdSrv_Response_srv_success();
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__CTRL_CMD_SRV__BUILDER_HPP_
