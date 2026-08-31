#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace finenav_2d
{

/// Declare a parameter, or read it back if some other component already declared it.
///
/// `SimulatorHelper` composes several small publishers onto one shared node, and more
/// than one of them legitimately wants the same parameter -- `input_odom_topic` and
/// `path_base_frame` are each read by two publishers.  Calling `declare_parameter`
/// twice on one node throws `ParameterAlreadyDeclaredException`, so every component
/// that is not guaranteed to be the first declarer must go through this helper.
template<typename T>
T DeclareOrGetParameter(rclcpp::Node * node, const std::string & name, const T & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}

}  // namespace finenav_2d
