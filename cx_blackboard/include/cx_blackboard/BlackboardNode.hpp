#ifndef CX_BLACKBOARD__BLACKBOARDNODE_HPP_
#define CX_BLACKBOARD__BLACKBOARDNODE_HPP_

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "cx_blackboard/InterfaceManager.hpp"

#include "cx_msgs/srv/open_interface.hpp"

namespace cx {
class Blackboard : public rclcpp_lifecycle::LifecycleNode {
public:
  Blackboard();
  // ~Blackboard();

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
  // CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
  // CallbackReturn on_error(const rclcpp_lifecycle::State &state);

  void open_interface_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::OpenInterface::Request> request,
      const std::shared_ptr<cx_msgs::srv::OpenInterface::Response> response);

  std::unique_ptr<cx::InterfaceManager> imngr_;

private:
  rclcpp::Service<cx_msgs::srv::OpenInterface>::SharedPtr
      open_interface_service_;
};
} // namespace cx

#endif // !CX_BLACKBOARD__BLACKBOARDNODE_HPP_