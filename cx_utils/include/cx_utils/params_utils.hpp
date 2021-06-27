#ifndef CX_UTILS__PARAMSUTILS_HPP
#define CX_UTILS__PARAMSUTILS_HPP

namespace cx {

//  !BASED ON NAVIGATION2 UTILS

/// Declares ROS2 parameter and sets it to a given value
template <typename NodeT>
void declare_parameter_if_not_declared(
    NodeT node, const std::string &param_name,
    const rclcpp::ParameterType &param_type,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor =
        rcl_interfaces::msg::ParameterDescriptor()) {
  if (!node->has_parameter(param_name)) {
    RCLCPP_INFO(rclcpp::get_logger("Params_Utils"), "Creating parameter: %s",
                param_name.c_str());
    node->declare_parameter(param_name, param_type, parameter_descriptor);
  }
}

/// Gets the type of plugin for the selected node and its plugin
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
} // namespace cx

#endif // !CX_UTILS__PARAMSUTILS_HPP
