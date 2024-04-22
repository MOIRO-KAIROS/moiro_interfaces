// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from yolov8_msgs:msg/DetectionInfo.idl
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
#include "yolov8_msgs/msg/detail/detection_info__struct.h"
#include "yolov8_msgs/msg/detail/detection_info__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool yolov8_msgs__msg__detection_info__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
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
    assert(strncmp("yolov8_msgs.msg._detection_info.DetectionInfo", full_classname_dest, 45) == 0);
  }
  yolov8_msgs__msg__DetectionInfo * ros_message = _ros_message;
  {  // center_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "center_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->center_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // center_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "center_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->center_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // average_depth
    PyObject * field = PyObject_GetAttrString(_pymsg, "average_depth");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->average_depth = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * yolov8_msgs__msg__detection_info__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DetectionInfo */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("yolov8_msgs.msg._detection_info");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DetectionInfo");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  yolov8_msgs__msg__DetectionInfo * ros_message = (yolov8_msgs__msg__DetectionInfo *)raw_ros_message;
  {  // center_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->center_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "center_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // center_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->center_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "center_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // average_depth
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->average_depth);
    {
      int rc = PyObject_SetAttrString(_pymessage, "average_depth", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
