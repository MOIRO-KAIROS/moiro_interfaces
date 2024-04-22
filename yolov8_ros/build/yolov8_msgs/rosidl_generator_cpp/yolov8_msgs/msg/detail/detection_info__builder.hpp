// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolov8_msgs:msg/DetectionInfo.idl
// generated code does not contain a copyright notice

#ifndef YOLOV8_MSGS__MSG__DETAIL__DETECTION_INFO__BUILDER_HPP_
#define YOLOV8_MSGS__MSG__DETAIL__DETECTION_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolov8_msgs/msg/detail/detection_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolov8_msgs
{

namespace msg
{

namespace builder
{

class Init_DetectionInfo_id
{
public:
  explicit Init_DetectionInfo_id(::yolov8_msgs::msg::DetectionInfo & msg)
  : msg_(msg)
  {}
  ::yolov8_msgs::msg::DetectionInfo id(::yolov8_msgs::msg::DetectionInfo::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolov8_msgs::msg::DetectionInfo msg_;
};

class Init_DetectionInfo_average_depth
{
public:
  explicit Init_DetectionInfo_average_depth(::yolov8_msgs::msg::DetectionInfo & msg)
  : msg_(msg)
  {}
  Init_DetectionInfo_id average_depth(::yolov8_msgs::msg::DetectionInfo::_average_depth_type arg)
  {
    msg_.average_depth = std::move(arg);
    return Init_DetectionInfo_id(msg_);
  }

private:
  ::yolov8_msgs::msg::DetectionInfo msg_;
};

class Init_DetectionInfo_center_y
{
public:
  explicit Init_DetectionInfo_center_y(::yolov8_msgs::msg::DetectionInfo & msg)
  : msg_(msg)
  {}
  Init_DetectionInfo_average_depth center_y(::yolov8_msgs::msg::DetectionInfo::_center_y_type arg)
  {
    msg_.center_y = std::move(arg);
    return Init_DetectionInfo_average_depth(msg_);
  }

private:
  ::yolov8_msgs::msg::DetectionInfo msg_;
};

class Init_DetectionInfo_center_x
{
public:
  Init_DetectionInfo_center_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectionInfo_center_y center_x(::yolov8_msgs::msg::DetectionInfo::_center_x_type arg)
  {
    msg_.center_x = std::move(arg);
    return Init_DetectionInfo_center_y(msg_);
  }

private:
  ::yolov8_msgs::msg::DetectionInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolov8_msgs::msg::DetectionInfo>()
{
  return yolov8_msgs::msg::builder::Init_DetectionInfo_center_x();
}

}  // namespace yolov8_msgs

#endif  // YOLOV8_MSGS__MSG__DETAIL__DETECTION_INFO__BUILDER_HPP_
