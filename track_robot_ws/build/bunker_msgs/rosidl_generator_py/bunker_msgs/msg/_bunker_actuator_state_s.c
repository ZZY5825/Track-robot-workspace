// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from bunker_msgs:msg/BunkerActuatorState.idl
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
#include "bunker_msgs/msg/detail/bunker_actuator_state__struct.h"
#include "bunker_msgs/msg/detail/bunker_actuator_state__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool bunker_msgs__msg__bunker_actuator_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("bunker_msgs.msg._bunker_actuator_state.BunkerActuatorState", full_classname_dest, 58) == 0);
  }
  bunker_msgs__msg__BunkerActuatorState * ros_message = _ros_message;
  {  // motor_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motor_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // rpm
    PyObject * field = PyObject_GetAttrString(_pymsg, "rpm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rpm = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // current
    PyObject * field = PyObject_GetAttrString(_pymsg, "current");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pulse_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "pulse_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->pulse_count = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // driver_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "driver_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->driver_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // driver_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "driver_temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->driver_temperature = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // motor_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_temperature");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motor_temperature = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // driver_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "driver_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->driver_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * bunker_msgs__msg__bunker_actuator_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of BunkerActuatorState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("bunker_msgs.msg._bunker_actuator_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "BunkerActuatorState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  bunker_msgs__msg__BunkerActuatorState * ros_message = (bunker_msgs__msg__BunkerActuatorState *)raw_ros_message;
  {  // motor_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->motor_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rpm
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->rpm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rpm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pulse_count
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->pulse_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pulse_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // driver_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->driver_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "driver_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // driver_temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->driver_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "driver_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_temperature
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->motor_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // driver_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->driver_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "driver_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
