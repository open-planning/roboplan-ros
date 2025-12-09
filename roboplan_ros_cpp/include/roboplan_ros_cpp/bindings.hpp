#pragma once

#include <nanobind/nanobind.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

namespace nb = nanobind;

/// @brief Template for converting Python ROS 2 message types to their C++ equivalents.
/// @details Conversion is done using rclcpp's serialization utility, which converts messages
/// to raw bytes and back.
/// @param py_msg The python ROS 2 message.
/// @return An equivalent C++ ROS 2 message.
template <typename ROSMsgType> ROSMsgType pyToMsg(nb::handle py_msg) {
  // Serialize with rclpy
  nb::module_ rclpy_serial = nb::module_::import_("rclpy.serialization");
  nb::object serialize_func = rclpy_serial.attr("serialize_message");
  nb::bytes serialized_data = nb::cast<nb::bytes>(serialize_func(py_msg));
  const auto data = serialized_data.c_str();
  size_t size = serialized_data.size();

  // Then deserialize with rclcpp
  rclcpp::SerializedMessage serialized_msg(size);
  auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
  memcpy(rcl_msg.buffer, data, size);
  rcl_msg.buffer_length = size;
  rclcpp::Serialization<ROSMsgType> serializer;
  ROSMsgType cpp_msg;
  serializer.deserialize_message(&serialized_msg, &cpp_msg);
  return cpp_msg;
}

/// @brief Template for converting C++ ROS 2 message types to their python equivalents.
/// @details Conversion is done using rclcpp's serialization utility, which converts messages
/// to raw bytes and back.
/// @param cpp_msg The C++ ROS 2 message.
/// @return An equivalent python ROS 2 message.
template <typename ROSMsgType>
nb::object msgToPy(const ROSMsgType& cpp_msg, nb::object py_msg_class) {
  // Serialize with rclcpp
  rclcpp::Serialization<ROSMsgType> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(&cpp_msg, &serialized_msg);

  // Then deserialize with rclpy
  auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
  nb::bytes serialized_data(reinterpret_cast<const char*>(rcl_msg.buffer), rcl_msg.buffer_length);
  nb::module_ rclpy_serial = nb::module_::import_("rclpy.serialization");
  nb::object deserialize_func = rclpy_serial.attr("deserialize_message");
  return deserialize_func(serialized_data, py_msg_class);
}
