// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#ifndef CX_FEATURES_CLIPS_FEATURE_MANAGER_HPP_
#define CX_FEATURES_CLIPS_FEATURE_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "cx_feature/clips_feature.hpp"

#include "cx_msgs/srv/list_clips_plugin.hpp"
#include "cx_msgs/srv/load_clips_plugin.hpp"
#include "cx_msgs/srv/unload_clips_plugin.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"

namespace cx {
using EnvsMap =
    std::unordered_map<std::string, cx::LockSharedPtr<clips::Environment>>;

class ClipsFeatureManager {
public:
  ClipsFeatureManager();
  ~ClipsFeatureManager();

  using FeaturesMap = std::unordered_map<std::string, cx::ClipsFeature::Ptr>;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                 const std::string &name, LockSharedPtr<EnvsMap> &envs);
  /**
   * @brief Cleanup resources
   */
  void cleanup();

  /**
   * @brief Activate feature manager
   */
  void activate();
  void activate_env(const std::string &env_name,
                    LockSharedPtr<clips::Environment> &env);

  /**
   * @brief Deactivate feature manager
   */
  void deactivate();
  void deactivate_env(const std::string &env_name,
                      LockSharedPtr<clips::Environment> &env);

  /**
   * @brief Reset feature manager
   */
  void reset();

  void feature_init_context(const std::string &env_name,
                            const std::string &feature_name);

  void clips_request_feature(clips::Environment *env, clips::UDFValue *out,
                             const std::string &feature_name);

  void load_plugin_cb(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::LoadCLIPSPlugin::Request> request,
      const std::shared_ptr<cx_msgs::srv::LoadCLIPSPlugin::Response> response);
  void unload_plugin_cb(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::UnloadCLIPSPlugin::Request> request,
      const std::shared_ptr<cx_msgs::srv::UnloadCLIPSPlugin::Response>
          response);
  void list_plugin_cb(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::ListCLIPSPlugin::Request> request,
      const std::shared_ptr<cx_msgs::srv::ListCLIPSPlugin::Response> response);

private:
  rclcpp::Service<cx_msgs::srv::LoadCLIPSPlugin>::SharedPtr
      load_plugin_service_;
  rclcpp::Service<cx_msgs::srv::UnloadCLIPSPlugin>::SharedPtr
      unload_plugin_service_;
  rclcpp::Service<cx_msgs::srv::ListCLIPSPlugin>::SharedPtr
      list_plugin_service_;

  std::vector<std::string> feature_names_vector_;
  // Pluginlib class loaders
  pluginlib::ClassLoader<cx::ClipsFeature> pg_loader_;
  std::vector<std::string> feature_ids_;

  LockSharedPtr<EnvsMap> envs_;

  std::unordered_map<std::string, std::vector<std::string>> loaded_plugins_;

  FeaturesMap features_;

  std::string name_;

  rclcpp::Logger logger_{rclcpp::get_logger("CLIPSPluginManager")};
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
};

} // namespace cx

#endif // !CX_FEATURES_CLIPS_FEATURE_MANAGER_HPP_
