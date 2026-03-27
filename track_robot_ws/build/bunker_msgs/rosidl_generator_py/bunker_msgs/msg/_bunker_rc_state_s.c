// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from bunker_msgs:msg/BunkerRCState.idl
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
#include "bunker_msgs/msg/detail/bunker_rc_state__struct.h"
#include "bunker_msgs/msg/detail/bunker_rc_state__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool bunker_msgs__msg__bunker_rc_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[47];
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
    assert(strncmp("bunker_msgs.msg._bunker_rc_state.BunkerRCState", full_classname_dest, 46) == 0);
  }
  bunker_msgs__msg__BunkerRCState * ros_message = _ros_message;
  {  // swa
    PyObject * field = PyObject_GetAttrString(_pymsg, "swa");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->swa = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // swb
    PyObject * field = PyObject_GetAttrString(_pymsg, "swb");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->swb = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // swc
    PyObject * field = PyObject_GetAttrString(_pymsg, "swc");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->swc = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // swd
    PyObject * field = PyObject_GetAttrString(_pymsg, "swd");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->swd = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // stick_right_v
    PyObject * field = PyObject_GetAttrString(_pymsg, "stick_right_v");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->stick_right_v = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // stick_right_h
    PyObject * field = PyObject_GetAttrString(_pymsg, "stick_right_h");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->stick_right_h = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // stick_left_v
    PyObject * field = PyObject_GetAttrString(_pymsg, "stick_left_v");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->stick_left_v = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // stick_left_h
    PyObject * field = PyObject_GetAttrString(_pymsg, "stick_left_h");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->stick_left_h = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // var_a
    PyObject * field = PyObject_GetAttrString(_pymsg, "var_a");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->var_a = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * bunker_msgs__msg__bunker_rc_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of BunkerRCState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("bunker_msgs.msg._bunker_rc_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "BunkerRCState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  bunker_msgs__msg__BunkerRCState * ros_message = (bunker_msgs__msg__BunkerRCState *)raw_ros_message;
  {  // swa
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->swa);
    {
      int rc = PyObject_SetAttrString(_pymessage, "swa", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // swb
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->swb);
    {
      int rc = PyObject_SetAttrString(_pymessage, "swb", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // swc
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->swc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "swc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // swd
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->swd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "swd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stick_right_v
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->stick_right_v);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stick_right_v", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stick_right_h
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->stick_right_h);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stick_right_h", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stick_left_v
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->stick_left_v);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stick_left_v", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stick_left_h
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->stick_left_h);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stick_left_h", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // var_a
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->var_a);
    {
      int rc = PyObject_SetAttrString(_pymessage, "var_a", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
