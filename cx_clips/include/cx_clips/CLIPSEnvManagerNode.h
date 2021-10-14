/***************************************************************************
 *  CLIPSEnvManagerNode.h
 *
 *  Created: 25 June 2021
 *  Copyright  2021  Ivaylo Doychev
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef CX_CLIPS__CLIPSENVMANAGERNODE_H_
#define CX_CLIPS__CLIPSENVMANAGERNODE_H_

#include <algorithm>
#include <clipsmm.h>
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

#include "cx_core/ClipsFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

#include "cx_msgs/msg/clips_context.hpp"

#include "cx_msgs/srv/add_clips_features.hpp"
#include "cx_msgs/srv/clips_feature_context.hpp"
#include "cx_msgs/srv/clips_remove_features.hpp"
#include "cx_msgs/srv/create_clips_env.hpp"
#include "cx_msgs/srv/destroy_clips_env.hpp"

namespace cx {
class ClipsFeaturesManager;
class ClipsExecutive;

class CLIPSEnvManagerNode : public rclcpp_lifecycle::LifecycleNode {
  friend ClipsFeaturesManager;
  friend ClipsExecutive;

public:
  typedef struct {
    LockSharedPtr<CLIPS::Environment> env;
    std::list<std::string> req_feat;
  } ClipsEnvData;

  CLIPSEnvManagerNode();
  // ~CLIPSEnvManagerNode();
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
  // CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
  // CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
  // CallbackReturn on_error(const rclcpp_lifecycle::State &state);

  std::map<std::string, LockSharedPtr<CLIPS::Environment>>
  getEnvironments() const;

  LockSharedPtr<CLIPS::Environment>
  getEnvironmentByName(const std::string &env_name);

  void create_env_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::CreateClipsEnv::Request> request,
      const std::shared_ptr<cx_msgs::srv::CreateClipsEnv::Response> response);

  void destroy_env_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::DestroyClipsEnv::Request> request,
      const std::shared_ptr<cx_msgs::srv::DestroyClipsEnv::Response> response);

  void add_clips_features_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::AddClipsFeatures::Request> request,
      const std::shared_ptr<cx_msgs::srv::AddClipsFeatures::Response> response);

  void assert_can_remove_features_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::ClipsRemoveFeatures::Request> request,
      const std::shared_ptr<cx_msgs::srv::ClipsRemoveFeatures::Response>
          response);

  void remove_features_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::ClipsRemoveFeatures::Request> request,
      const std::shared_ptr<cx_msgs::srv::ClipsRemoveFeatures::Response>
          response);

private:
  LockSharedPtr<CLIPS::Environment>
  new_env(const std::string &log_component_name);
  void assert_features(LockSharedPtr<CLIPS::Environment> &clips,
                       bool immediate_assert);
  void add_functions(const std::string &env_name,
                     LockSharedPtr<CLIPS::Environment> &clips);
  void guarded_load(const std::string &env_name, const std::string &filename);
  CLIPS::Value clips_request_feature(const std::string &env_name,
                                     const std::string &feature_name);
  CLIPS::Values clips_now();
  CLIPS::Values clips_now_systime();

  // Function for service clients
  void call_feature_context_initialisation(const std::string &env_name,
                                           const std::string &feature_name);
  // Function for service clients
  void call_feature_context_destroy(const std::string &env_name,
                                    const std::string &feature_name);

  // ROS2 SERVICES
  rclcpp::Service<cx_msgs::srv::CreateClipsEnv>::SharedPtr create_env_service_;
  rclcpp::Service<cx_msgs::srv::DestroyClipsEnv>::SharedPtr
      destroy_env_service_;
  rclcpp::Service<cx_msgs::srv::AddClipsFeatures>::SharedPtr
      add_features_service_;
  rclcpp::Service<cx_msgs::srv::ClipsRemoveFeatures>::SharedPtr
      assert_can_remove_features_service_;
  rclcpp::Service<cx_msgs::srv::ClipsRemoveFeatures>::SharedPtr
      remove_features_service_;

  rclcpp::Client<cx_msgs::srv::ClipsFeatureContext>::SharedPtr
      destroy_feature_context_client;

  /*Establish callback group for the destroy context client as it calls the
   * clips feature manager service and waits for a result*/
  rclcpp::CallbackGroup::SharedPtr callback_group_;

private:
  std::string clips_dir_;

  std::map<std::string, ClipsEnvData> envs_;
  std::set<std::string> features_set;
};
} // namespace cx

#endif // !CX_CLIPS__CLIPSENVMANAGERNODE_H_
