// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  CLIPSEnvManagerClient.hpp
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

#ifndef CX_CLIPS__CLIPSENVMANAGERCLIENT_HPP_
#define CX_CLIPS__CLIPSENVMANAGERCLIENT_HPP_

#include <list>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "cx_utils/NodeThread.hpp"

#include "cx_msgs/srv/add_clips_features.hpp"
#include "cx_msgs/srv/clips_remove_features.hpp"
#include "cx_msgs/srv/create_clips_env.hpp"
#include "cx_msgs/srv/destroy_clips_env.hpp"

namespace cx {

class CLIPSEnvManagerClient {
public:
  CLIPSEnvManagerClient(const std::string &node_name);
  // ~CLIPSEnvManagerClient();

  bool createNewClipsEnvironment(const std::string &env_name,
                                 const std::string &log_name);
  bool destroyClipsEnvironment(const std::string &env_name);
  bool addFeatures(const std::vector<std::string> &features);
  bool assertCanRemoveClipsFeatures(const std::vector<std::string> &features);
  bool removeClipsFeatures(const std::vector<std::string> &features);

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<cx::NodeThread> thread_;

  rclcpp::Client<cx_msgs::srv::AddClipsFeatures>::SharedPtr
      add_clips_features_client_;
  rclcpp::Client<cx_msgs::srv::CreateClipsEnv>::SharedPtr create_env_client_;
  rclcpp::Client<cx_msgs::srv::DestroyClipsEnv>::SharedPtr destroy_env_client_;
  rclcpp::Client<cx_msgs::srv::ClipsRemoveFeatures>::SharedPtr
      assert_can_remove_features_client_;
  rclcpp::Client<cx_msgs::srv::ClipsRemoveFeatures>::SharedPtr
      remove_features_client_;
  // destroy_env_client_;
  /*Establish callback group as it calls the
   * clips node manager service and waits for a result*/

  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

} // namespace cx
#endif // !CX_CLIPS__CLIPSENVMANAGERCLIENT_HPP_
