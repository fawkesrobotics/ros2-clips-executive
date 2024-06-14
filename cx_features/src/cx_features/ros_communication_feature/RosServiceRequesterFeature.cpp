// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  RosTopicServiceRequesterFeature.cpp
 *
 *  Created: November 13th, 2023
 *  Copyright  2023 Daniel Swoboda
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

#include <map>
#include <memory>
#include <string>

#include "cx_core/ClipsFeature.hpp"
#include "cx_features/RosServiceRequesterFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;

namespace cx {

RosServiceRequesterFeature::RosServiceRequesterFeature()
    : Node("ros_topic_feature_node") {}
RosServiceRequesterFeature::~RosServiceRequesterFeature() {}

std::string RosServiceRequesterFeature::getFeatureName() const {
  return clips_feature_name;
}

void RosServiceRequesterFeature::initialise(const std::string &feature_name) {
  clips_feature_name = feature_name;

  spin_thread_ =
      std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });
}

bool RosServiceRequesterFeature::clips_context_init(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialising context for feature %s",
              clips_feature_name.c_str());

  envs_[env_name] = clips;

  // add base implementations for ros communication
  // all of these need to be implemented given the corresponding types
  clips->add_function(
      "ros-" + msg_name_ + "-create-request",
      sigc::slot<void, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this, &RosServiceRequesterFeature::create_request),
          env_name)));
  clips->add_function(
      "ros-" + msg_name_ + "-set-field-request",
      sigc::slot<void, std::string, std::string, CLIPS::Value>(sigc::bind<0>(
          sigc::mem_fun(*this, &RosServiceRequesterFeature::set_field_request),
          env_name)));
  clips->add_function(
      "ros-" + msg_name_ + "-set-array-request",
      sigc::slot<void, std::string, std::string, CLIPS::Values>(sigc::bind<0>(
          sigc::mem_fun(*this, &RosServiceRequesterFeature::set_array_request),
          env_name)));
  clips->add_function(
      "ros-" + msg_name_ + "-request",
      sigc::slot<void, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this, &RosServiceRequesterFeature::request_from_node),
          env_name)));

  // add base fact templates
  clips->build("(deftemplate ros-" + msg_name_ + "-response\
            (slot service (type STRING)) (slot success (type SYMBOL)))");

  return true;
}

bool RosServiceRequesterFeature::clips_context_destroyed(
    const std::string &env_name) {

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Destroying clips context!");
  envs_.erase(env_name);

  return true;
}

void RosServiceRequesterFeature::create_request(
    const std::string &env_name, const std::string &service_name) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Creating request for service %s %s", service_name.c_str(),
              env_name.c_str());

  request_clients_[env_name][service_name] =
      this->create_client<std_srvs::srv::SetBool>(service_name);
  requests_[env_name][service_name] =
      std::make_shared<std_srvs::srv::SetBool::Request>();
}

void RosServiceRequesterFeature::set_field_request(
    const std::string &env_name, const std::string &service_name,
    const std::string &field_name, CLIPS::Value value) {
  if (field_name == "data") {
    requests_[env_name][service_name]->data =
        (value.as_string() == "TRUE") ? true : false;
  }
}

void RosServiceRequesterFeature::set_array_request(
    const std::string &env_name, const std::string &service_name,
    const std::string &field_name, CLIPS::Values values) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Setting array %s for service %s %s not supported ",
              field_name.c_str(), service_name.c_str(), env_name.c_str());
  (void)values;
}

void RosServiceRequesterFeature::request_from_node(
    const std::string &env_name, const std::string &service_name) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Requesting service %s %s", service_name.c_str(),
              env_name.c_str());

  auto response_callback =
      [=](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response) {
        service_callback(response, service_name, env_name);
      };
  request_clients_[env_name][service_name]->async_send_request(
      requests_[env_name][service_name], response_callback);
}

void RosServiceRequesterFeature::service_callback(
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response,
    std::string service_name, std::string env_name) {
  cx::LockSharedPtr<CLIPS::Environment> &clips = envs_[env_name];
  std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));

  // remove old responses facts
  std::vector<CLIPS::Fact::pointer> facts = {};
  CLIPS::Fact::pointer fact = envs_[env_name]->get_facts();
  while (fact) {
    if (fact->get_template()->name() == "ros-" + msg_name_ + "-response" &&
        fact->slot_value("service")[0].as_string() == service_name) {
      facts.push_back(fact);
    }
    fact = fact->next();
  }
  for (auto &fact : facts) {
    fact->retract();
  }
  auto result = response.get();
  // assert the newest responses
  envs_[env_name]->assert_fact("(ros-" + msg_name_ + "-response (service \"" +
                               service_name + "\") (success " +
                               ((result->success) ? "TRUE ))" : "FALSE ))"));
}

} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::RosServiceRequesterFeature, cx::ClipsFeature)
