#ifndef CX_FEATURES__CLIPSFEATURSMANAGER_HPP_
#define CX_FEATURES__CLIPSFEATURSMANAGER_HPP_

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "cx_clips/CLIPSEnvManagerNode.h"
#include "cx_core/ClipsFeature.hpp"
#include "cx_msgs/srv/clips_feature_context.hpp"

namespace cx {

class ClipsFeaturesManager : public rclcpp_lifecycle::LifecycleNode {
public:
  ClipsFeaturesManager();
  ~ClipsFeaturesManager();

  // sets the member pointer to the existing clips env manager to extract the
  // environments
  void pre_configure(std::shared_ptr<cx::CLIPSEnvManagerNode> &manager_node);

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
  // CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
  // CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
  // CallbackReturn on_error(const rclcpp_lifecycle::State &state);

  void feature_init_context_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::ClipsFeatureContext::Request> request,
      const std::shared_ptr<cx_msgs::srv::ClipsFeatureContext::Response>
          response);

  void feature_destroy_context_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::ClipsFeatureContext::Request> request,
      const std::shared_ptr<cx_msgs::srv::ClipsFeatureContext::Response>
          response);

  // MOVE TO PRIVATE LATER!
  std::map<std::string, std::shared_ptr<cx::ClipsFeature>> features_;
  std::shared_ptr<cx::CLIPSEnvManagerNode> clips_env_manager_node_;

private:
  rclcpp::Service<cx_msgs::srv::ClipsFeatureContext>::SharedPtr
      feature_init_context_service_;
  rclcpp::Service<cx_msgs::srv::ClipsFeatureContext>::SharedPtr
      feature_destroy_context_service_;
};

} // namespace cx

#endif // !CX_FEATURES__CLIPSFEATURSMANAGER_HPP_
