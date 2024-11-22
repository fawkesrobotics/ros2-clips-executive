// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#ifndef CX_CLIPS__CLIPSENVMANAGER_H_
#define CX_CLIPS__CLIPSENVMANAGER_H_

#include <algorithm>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "cx_clips_env_manager/clips_plugin_manager.hpp"
#include "cx_utils/lock_shared_ptr.hpp"

#include "cx_msgs/srv/create_clips_env.hpp"
#include "cx_msgs/srv/destroy_clips_env.hpp"

#include <spdlog/spdlog.h>

namespace cx {
class ClipsPluginManager;
class ClipsExecutive;

class CLIPSEnvManager : public rclcpp_lifecycle::LifecycleNode {
  friend ClipsPluginManager;
  friend ClipsExecutive;

public:
  CLIPSEnvManager();
  ~CLIPSEnvManager();
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  // using EnvsMap = std::unordered_map<std::string,
  // cx::LockSharedPtr<clips::Environment>>;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
  CallbackReturn on_error(const rclcpp_lifecycle::State &state);

  // std::map<std::string, LockSharedPtr<clips::Environment>>
  // getEnvironments() const;

  // LockSharedPtr<clips::Environment>
  // getEnvironmentByName(const std::string &env_name);

  void create_env_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::CreateClipsEnv::Request> request,
      const std::shared_ptr<cx_msgs::srv::CreateClipsEnv::Response> response);

  void destroy_env_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::DestroyClipsEnv::Request> request,
      const std::shared_ptr<cx_msgs::srv::DestroyClipsEnv::Response> response);

private:
  LockSharedPtr<clips::Environment> new_env(const std::string &env_name);

  bool delete_env(const std::string &env_name);
  // void assert_plugins(LockSharedPtr<clips::Environment> &clips,
  //                     bool immediate_assert);
  // void add_functions(const std::string &env_name);
  // bool guarded_load(LockSharedPtr<clips::Environment> &env, const std::string
  // &filename);

  // ROS2 SERVICES
  rclcpp::Service<cx_msgs::srv::CreateClipsEnv>::SharedPtr create_env_service_;
  rclcpp::Service<cx_msgs::srv::DestroyClipsEnv>::SharedPtr
      destroy_env_service_;

private:
  ClipsPluginManager plugin_manager_;

  cx::LockSharedPtr<EnvsMap> envs_;
};
} // namespace cx

#endif // !CX_CLIPS__CLIPSENVMANAGER_H_
