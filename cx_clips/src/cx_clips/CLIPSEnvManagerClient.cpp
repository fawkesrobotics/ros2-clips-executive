/***************************************************************************
 *  CLIPSEnvManagerClient.cpp
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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "cx_clips/CLIPSEnvManagerClient.hpp"

namespace cx {

using namespace std::chrono_literals;

CLIPSEnvManagerClient::CLIPSEnvManagerClient(const std::string &node_name) {
  node_ = rclcpp::Node::make_shared(node_name);

  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  create_env_client_ = node_->create_client<cx_msgs::srv::CreateClipsEnv>(
      "clips_manager/create_env");

  destroy_env_client_ = node_->create_client<cx_msgs::srv::DestroyClipsEnv>(
      "clips_manager/destroy_env");

  add_clips_features_client_ =
      node_->create_client<cx_msgs::srv::AddClipsFeatures>(
          "clips_manager/add_clips_features");

  assert_can_remove_features_client_ =
      node_->create_client<cx_msgs::srv::ClipsRemoveFeatures>(
          "clips_manager/assert_can_remove_features",
          rmw_qos_profile_services_default, callback_group_);

  remove_features_client_ =
      node_->create_client<cx_msgs::srv::ClipsRemoveFeatures>(
          "clips_manager/remove_features");

  thread_ = std::make_unique<cx::NodeThread>(node_->get_node_base_interface());
}
// CLIPSEnvManagerClient::~CLIPSEnvManagerClient() {}

//! Works Async
bool CLIPSEnvManagerClient::createNewClipsEnvironment(
    const std::string &env_name, const std::string &log_name) {

  while (!create_env_client_->wait_for_service(5s)) {
    if (!rclcpp::ok()) {

      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   create_env_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                create_env_client_->get_service_name());
  }

  auto req = std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
  req->env_name = env_name;
  req->log_name = log_name;

  using ServiceResponseFuture =
      rclcpp::Client<cx_msgs::srv::CreateClipsEnv>::SharedFuture;
  auto response_received_callback = [this,
                                     &env_name](ServiceResponseFuture future) {
    if (!future.get()->success) {

      RCLCPP_WARN(node_->get_logger(), "Couldn't create env! Error: %s",
                  future.get()->error.c_str());
    }
  };

  auto future_res =
      create_env_client_->async_send_request(req, response_received_callback);
  return true;

  // auto status = future_res.wait_for(3s);

  // if (status == std::future_status::ready) {
  //   if (future_res.get()->success) {
  //     return true;
  //   } else {
  //     RCLCPP_ERROR(node_->get_logger(), "ERROR --- %s",
  //                  future_res.get()->error.c_str());
  //     return false;
  //   }
  // } else {
  //   RCLCPP_ERROR(node_->get_logger(), "%s: timed out waiting for
  //   response!",
  //                create_env_client_->get_service_name());
  //   return false;
  // }

  // // HARD CODED 20 S -> maybe add as param!
  // if (rclcpp::spin_until_future_complete(node_, future_res, 10s) !=
  //     rclcpp::FutureReturnCode::SUCCESS) {
  //   RCLCPP_ERROR(node_->get_logger(), "%s: timed out waiting for
  //   response!",
  //                create_env_client_->get_service_name());
  //   return false;
  // }

  // if (future_res.get()->success) {
  //   return true;
  // } else {
  //   RCLCPP_ERROR(node_->get_logger(), "ERROR --- %s",
  //                future_res.get()->error.c_str());
  //   return false;
  // }
}

//! Works Async
bool CLIPSEnvManagerClient::destroyClipsEnvironment(
    const std::string &env_name) {

  while (!destroy_env_client_->wait_for_service(5s)) {
    if (!rclcpp::ok()) {

      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   destroy_env_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                destroy_env_client_->get_service_name());
  }

  auto req = std::make_shared<cx_msgs::srv::DestroyClipsEnv::Request>();
  req->env_name = env_name;

  using ServiceResponseFuture =
      rclcpp::Client<cx_msgs::srv::DestroyClipsEnv>::SharedFuture;
  auto response_received_callback = [this,
                                     &env_name](ServiceResponseFuture future) {
    RCLCPP_INFO(node_->get_logger(), "Destroyed env %s async!",
                env_name.c_str());
  };

  auto future_res =
      destroy_env_client_->async_send_request(req, response_received_callback);
  return true;

  // auto future_res = destroy_env_client_->async_send_request(req);

  // // HARD CODED 20 S -> maybe add as param!
  // if (rclcpp::spin_until_future_complete(node_, future_res, 20s) !=
  //     rclcpp::FutureReturnCode::SUCCESS) {
  //   RCLCPP_ERROR(node_->get_logger(), "%s: timed out waiting for
  //   response!",
  //                destroy_env_client_->get_service_name());
  //   return false;
  // }

  // if (future_res.get()->success) {
  //   return true;
  // } else {
  //   RCLCPP_ERROR(node_->get_logger(), "ERROR --- %s",
  //                future_res.get()->error.c_str());
  //   return false;
  // }
}

//! Works Async
bool CLIPSEnvManagerClient::addFeatures(
    const std::vector<std::string> &features) {
  RCLCPP_WARN(node_->get_logger(), "%s: Adding corresponding fts!",
              add_clips_features_client_->get_service_name());

  while (!add_clips_features_client_->wait_for_service(5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   add_clips_features_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                add_clips_features_client_->get_service_name());
  }

  auto req = std::make_shared<cx_msgs::srv::AddClipsFeatures::Request>();
  req->features = features;

  using ServiceResponseFuture =
      rclcpp::Client<cx_msgs::srv::AddClipsFeatures>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    if (future.get()->success) {
      RCLCPP_INFO(node_->get_logger(), "Features added successfully!");
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Following features couldn't get added:");
      std::vector<std::string> feats = future.get()->failed_features;
      for (auto &feat : feats) {
        RCLCPP_WARN(node_->get_logger(), "%s", feat.c_str());
      }
    }
  };

  auto future_res = add_clips_features_client_->async_send_request(
      req, response_received_callback);

  return true;
}

//! Works Async
bool CLIPSEnvManagerClient::assertCanRemoveClipsFeatures(
    const std::vector<std::string> &features) {

  while (!assert_can_remove_features_client_->wait_for_service(5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   assert_can_remove_features_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                assert_can_remove_features_client_->get_service_name());
  }

  auto req = std::make_shared<cx_msgs::srv::ClipsRemoveFeatures::Request>();
  req->features = features;

  using ServiceResponseFuture =
      rclcpp::Client<cx_msgs::srv::ClipsRemoveFeatures>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    if (future.get()->success) {
      RCLCPP_INFO(node_->get_logger(),
                  "All requested features can be removed!");
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Not all requested features can be removed! Error: %s",
                  future.get()->error.c_str());
    }
  };

  auto future_res = assert_can_remove_features_client_->async_send_request(
      req, response_received_callback);

  return true;
  // auto future_res =
  // assert_can_remove_features_client_->async_send_request(req);

  // if (rclcpp::spin_until_future_complete(node_, future_res, 5s) !=
  //     rclcpp::FutureReturnCode::SUCCESS) {
  //   RCLCPP_ERROR(node_->get_logger(), "%s: timed out waiting for
  //   response!",
  //                assert_can_remove_features_client_->get_service_name());
  //   return false;
  // }

  // if (future_res.get()->success) {
  //   RCLCPP_INFO(node_->get_logger(), "All requested features can be
  //   removed!"); return true;
  // }

  // return false;
}

//! Works Async
bool CLIPSEnvManagerClient::removeClipsFeatures(
    const std::vector<std::string> &features) {

  while (!remove_features_client_->wait_for_service(5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   remove_features_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                remove_features_client_->get_service_name());
  }

  auto req = std::make_shared<cx_msgs::srv::ClipsRemoveFeatures::Request>();
  req->features = features;

  using ServiceResponseFuture =
      rclcpp::Client<cx_msgs::srv::ClipsRemoveFeatures>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    if (future.get()->success) {
      RCLCPP_INFO(node_->get_logger(), "All requested features were removed!");
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Couldn't remove all requested features!");
    }
  };

  auto future_res = remove_features_client_->async_send_request(
      req, response_received_callback);

  // auto future_res = remove_features_client_->async_send_request(req);

  // if (rclcpp::spin_until_future_complete(node_, future_res, 15s) !=
  //     rclcpp::FutureReturnCode::SUCCESS) {
  //   RCLCPP_ERROR(node_->get_logger(), "%s: timed out waiting for
  //   response!",
  //                remove_features_client_->get_service_name());
  //   return false;
  // }

  return true;
}

} // namespace cx
