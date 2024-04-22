// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yolov8_msgs:msg/DetectionInfo.idl
// generated code does not contain a copyright notice

#ifndef YOLOV8_MSGS__MSG__DETAIL__DETECTION_INFO__STRUCT_HPP_
#define YOLOV8_MSGS__MSG__DETAIL__DETECTION_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__yolov8_msgs__msg__DetectionInfo __attribute__((deprecated))
#else
# define DEPRECATED__yolov8_msgs__msg__DetectionInfo __declspec(deprecated)
#endif

namespace yolov8_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DetectionInfo_
{
  using Type = DetectionInfo_<ContainerAllocator>;

  explicit DetectionInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->center_x = 0l;
      this->center_y = 0l;
      this->average_depth = 0.0;
      this->id = 0l;
    }
  }

  explicit DetectionInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->center_x = 0l;
      this->center_y = 0l;
      this->average_depth = 0.0;
      this->id = 0l;
    }
  }

  // field types and members
  using _center_x_type =
    int32_t;
  _center_x_type center_x;
  using _center_y_type =
    int32_t;
  _center_y_type center_y;
  using _average_depth_type =
    double;
  _average_depth_type average_depth;
  using _id_type =
    int32_t;
  _id_type id;

  // setters for named parameter idiom
  Type & set__center_x(
    const int32_t & _arg)
  {
    this->center_x = _arg;
    return *this;
  }
  Type & set__center_y(
    const int32_t & _arg)
  {
    this->center_y = _arg;
    return *this;
  }
  Type & set__average_depth(
    const double & _arg)
  {
    this->average_depth = _arg;
    return *this;
  }
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yolov8_msgs::msg::DetectionInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const yolov8_msgs::msg::DetectionInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yolov8_msgs::msg::DetectionInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yolov8_msgs::msg::DetectionInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yolov8_msgs::msg::DetectionInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yolov8_msgs::msg::DetectionInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yolov8_msgs::msg::DetectionInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yolov8_msgs::msg::DetectionInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yolov8_msgs::msg::DetectionInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yolov8_msgs::msg::DetectionInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yolov8_msgs__msg__DetectionInfo
    std::shared_ptr<yolov8_msgs::msg::DetectionInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yolov8_msgs__msg__DetectionInfo
    std::shared_ptr<yolov8_msgs::msg::DetectionInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectionInfo_ & other) const
  {
    if (this->center_x != other.center_x) {
      return false;
    }
    if (this->center_y != other.center_y) {
      return false;
    }
    if (this->average_depth != other.average_depth) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectionInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectionInfo_

// alias to use template instance with default allocator
using DetectionInfo =
  yolov8_msgs::msg::DetectionInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yolov8_msgs

#endif  // YOLOV8_MSGS__MSG__DETAIL__DETECTION_INFO__STRUCT_HPP_
