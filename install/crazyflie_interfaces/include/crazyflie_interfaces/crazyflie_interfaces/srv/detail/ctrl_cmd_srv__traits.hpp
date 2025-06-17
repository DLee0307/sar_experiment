// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crazyflie_interfaces:srv/CTRLCmdSrv.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__CTRL_CMD_SRV__TRAITS_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__CTRL_CMD_SRV__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crazyflie_interfaces/srv/detail/ctrl_cmd_srv__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'cmd_vals'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace crazyflie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CTRLCmdSrv_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: cmd_type
  {
    out << "cmd_type: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_type, out);
    out << ", ";
  }

  // member: cmd_vals
  {
    out << "cmd_vals: ";
    to_flow_style_yaml(msg.cmd_vals, out);
    out << ", ";
  }

  // member: cmd_flag
  {
    out << "cmd_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_flag, out);
    out << ", ";
  }

  // member: cmd_rx
  {
    out << "cmd_rx: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_rx, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CTRLCmdSrv_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cmd_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_type: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_type, out);
    out << "\n";
  }

  // member: cmd_vals
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_vals:\n";
    to_block_style_yaml(msg.cmd_vals, out, indentation + 2);
  }

  // member: cmd_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_flag, out);
    out << "\n";
  }

  // member: cmd_rx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_rx: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_rx, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CTRLCmdSrv_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace crazyflie_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use crazyflie_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const crazyflie_interfaces::srv::CTRLCmdSrv_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::srv::CTRLCmdSrv_Request & msg)
{
  return crazyflie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::srv::CTRLCmdSrv_Request>()
{
  return "crazyflie_interfaces::srv::CTRLCmdSrv_Request";
}

template<>
inline const char * name<crazyflie_interfaces::srv::CTRLCmdSrv_Request>()
{
  return "crazyflie_interfaces/srv/CTRLCmdSrv_Request";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::CTRLCmdSrv_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::CTRLCmdSrv_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<crazyflie_interfaces::srv::CTRLCmdSrv_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace crazyflie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CTRLCmdSrv_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: srv_success
  {
    out << "srv_success: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CTRLCmdSrv_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: srv_success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "srv_success: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CTRLCmdSrv_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace crazyflie_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use crazyflie_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const crazyflie_interfaces::srv::CTRLCmdSrv_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::srv::CTRLCmdSrv_Response & msg)
{
  return crazyflie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::srv::CTRLCmdSrv_Response>()
{
  return "crazyflie_interfaces::srv::CTRLCmdSrv_Response";
}

template<>
inline const char * name<crazyflie_interfaces::srv::CTRLCmdSrv_Response>()
{
  return "crazyflie_interfaces/srv/CTRLCmdSrv_Response";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::CTRLCmdSrv_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::CTRLCmdSrv_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<crazyflie_interfaces::srv::CTRLCmdSrv_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<crazyflie_interfaces::srv::CTRLCmdSrv>()
{
  return "crazyflie_interfaces::srv::CTRLCmdSrv";
}

template<>
inline const char * name<crazyflie_interfaces::srv::CTRLCmdSrv>()
{
  return "crazyflie_interfaces/srv/CTRLCmdSrv";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::CTRLCmdSrv>
  : std::integral_constant<
    bool,
    has_fixed_size<crazyflie_interfaces::srv::CTRLCmdSrv_Request>::value &&
    has_fixed_size<crazyflie_interfaces::srv::CTRLCmdSrv_Response>::value
  >
{
};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::CTRLCmdSrv>
  : std::integral_constant<
    bool,
    has_bounded_size<crazyflie_interfaces::srv::CTRLCmdSrv_Request>::value &&
    has_bounded_size<crazyflie_interfaces::srv::CTRLCmdSrv_Response>::value
  >
{
};

template<>
struct is_service<crazyflie_interfaces::srv::CTRLCmdSrv>
  : std::true_type
{
};

template<>
struct is_service_request<crazyflie_interfaces::srv::CTRLCmdSrv_Request>
  : std::true_type
{
};

template<>
struct is_service_response<crazyflie_interfaces::srv::CTRLCmdSrv_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__CTRL_CMD_SRV__TRAITS_HPP_
