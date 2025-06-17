// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_interfaces:srv/CTRLCmdSrv.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__CTRL_CMD_SRV__STRUCT_H_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__CTRL_CMD_SRV__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'cmd_vals'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in srv/CTRLCmdSrv in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__srv__CTRLCmdSrv_Request
{
  uint16_t cmd_type;
  geometry_msgs__msg__Vector3 cmd_vals;
  uint16_t cmd_flag;
  bool cmd_rx;
} crazyflie_interfaces__srv__CTRLCmdSrv_Request;

// Struct for a sequence of crazyflie_interfaces__srv__CTRLCmdSrv_Request.
typedef struct crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence
{
  crazyflie_interfaces__srv__CTRLCmdSrv_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/CTRLCmdSrv in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__srv__CTRLCmdSrv_Response
{
  bool srv_success;
} crazyflie_interfaces__srv__CTRLCmdSrv_Response;

// Struct for a sequence of crazyflie_interfaces__srv__CTRLCmdSrv_Response.
typedef struct crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence
{
  crazyflie_interfaces__srv__CTRLCmdSrv_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__CTRL_CMD_SRV__STRUCT_H_
