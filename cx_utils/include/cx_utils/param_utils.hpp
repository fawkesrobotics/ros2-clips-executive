// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#ifndef CX_UTILS__PARAMSUTILS_HPP
#define CX_UTILS__PARAMSUTILS_HPP

namespace cx {
namespace cx_utils {
/// Declares static ROS2 parameter and sets it to a given value if it was not
/// already declared
/* Declares static ROS2 parameter and sets it to a given value
 * if it was not already declared.
 *
 * \param[in] node A node in which given parameter to be declared
 * \param[in] param_name The name of parameter
 * \param[in] default_value Parameter value to initialize with
 * \param[in] parameter_descriptor Parameter descriptor (optional)
 */
template <typename NodeT>
void declare_parameter_if_not_declared(
    NodeT node, const std::string &param_name,
    const rclcpp::ParameterValue &default_value,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor =
        rcl_interfaces::msg::ParameterDescriptor()) {
  if (!node->has_parameter(param_name)) {
    RCLCPP_INFO(rclcpp::get_logger("cx_utils"), "Creating parameter: %s",
                param_name.c_str());
    node->declare_parameter(param_name, default_value, parameter_descriptor);
  }
}

/// Declares ROS2 parameter and sets it to a given value
/* Declares static ROS2 parameter with given type if it was not already
 * declared.
 *
 * \param[in] node A node in which given parameter to be declared
 * \param[in] param_type The type of parameter
 * \param[in] default_value Parameter value to initialize with
 * \param[in] parameter_descriptor Parameter descriptor (optional)
 */
template <typename NodeT>
void declare_parameter_if_not_declared(
    NodeT node, const std::string &param_name,
    const rclcpp::ParameterType &param_type,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor =
        rcl_interfaces::msg::ParameterDescriptor()) {
  if (!node->has_parameter(param_name)) {
    RCLCPP_INFO(rclcpp::get_logger("cx_utils"), "Creating parameter: %s",
                param_name.c_str());
    node->declare_parameter(param_name, param_type, parameter_descriptor);
  }
}

/// Gets the type of plugin for the selected node and its plugin
/**
 * Gets the type of plugin for the selected node and its plugin.
 * Actually seeks for the value of "<plugin_name>.plugin" parameter.
 *
 * \param[in] node Selected node
 * \param[in] plugin_name The name of plugin the type of which is being searched
 * for \return A string containing the type of plugin (the value of
 * "<plugin_name>.plugin" parameter)
 */
template <typename NodeT>
std::string get_plugin_type_param(NodeT node, const std::string &plugin_name) {
  declare_parameter_if_not_declared(node, plugin_name + ".plugin",
                                    rclcpp::PARAMETER_STRING);
  std::string plugin_type;
  try {
    if (!node->get_parameter(plugin_name + ".plugin", plugin_type)) {
      RCLCPP_FATAL(node->get_logger(), "'plugin' param not defined for %s",
                   plugin_name.c_str());
      exit(-1);
    }

  } catch (rclcpp::exceptions::ParameterUninitializedException &ex) {
    RCLCPP_FATAL(node->get_logger(),
                 "'plugin' param not defined for %s. Error: %s",
                 plugin_name.c_str(), ex.what());
    exit(-1);
  }

  return plugin_type;
}
} // namespace cx_utils
} // namespace cx

#endif // !CX_UTILS__PARAMSUTILS_HPP
