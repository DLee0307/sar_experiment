// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from crazyflie_interfaces:srv/CTRLCmdSrv.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "crazyflie_interfaces/srv/detail/ctrl_cmd_srv__struct.h"
#include "crazyflie_interfaces/srv/detail/ctrl_cmd_srv__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool crazyflie_interfaces__srv__ctrl_cmd_srv__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[58];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("crazyflie_interfaces.srv._ctrl_cmd_srv.CTRLCmdSrv_Request", full_classname_dest, 57) == 0);
  }
  crazyflie_interfaces__srv__CTRLCmdSrv_Request * ros_message = _ros_message;
  {  // cmd_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "cmd_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->cmd_type = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // cmd_vals
    PyObject * field = PyObject_GetAttrString(_pymsg, "cmd_vals");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->cmd_vals)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // cmd_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "cmd_flag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->cmd_flag = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // cmd_rx
    PyObject * field = PyObject_GetAttrString(_pymsg, "cmd_rx");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->cmd_rx = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * crazyflie_interfaces__srv__ctrl_cmd_srv__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CTRLCmdSrv_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("crazyflie_interfaces.srv._ctrl_cmd_srv");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CTRLCmdSrv_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  crazyflie_interfaces__srv__CTRLCmdSrv_Request * ros_message = (crazyflie_interfaces__srv__CTRLCmdSrv_Request *)raw_ros_message;
  {  // cmd_type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->cmd_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cmd_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cmd_vals
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->cmd_vals);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "cmd_vals", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cmd_flag
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->cmd_flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cmd_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cmd_rx
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->cmd_rx ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cmd_rx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/ctrl_cmd_srv__struct.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/ctrl_cmd_srv__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool crazyflie_interfaces__srv__ctrl_cmd_srv__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[59];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("crazyflie_interfaces.srv._ctrl_cmd_srv.CTRLCmdSrv_Response", full_classname_dest, 58) == 0);
  }
  crazyflie_interfaces__srv__CTRLCmdSrv_Response * ros_message = _ros_message;
  {  // srv_success
    PyObject * field = PyObject_GetAttrString(_pymsg, "srv_success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->srv_success = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * crazyflie_interfaces__srv__ctrl_cmd_srv__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CTRLCmdSrv_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("crazyflie_interfaces.srv._ctrl_cmd_srv");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CTRLCmdSrv_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  crazyflie_interfaces__srv__CTRLCmdSrv_Response * ros_message = (crazyflie_interfaces__srv__CTRLCmdSrv_Response *)raw_ros_message;
  {  // srv_success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->srv_success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "srv_success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
