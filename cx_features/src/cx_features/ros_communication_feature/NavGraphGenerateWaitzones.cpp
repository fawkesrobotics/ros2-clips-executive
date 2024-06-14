/***************************************************************************
 *  NavGraphGenerateWaitzones.cpp
 *
 *  Automatically Generated: 2023-11-30 14:38:37
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
#include "cx_features/NavGraphGenerateWaitzones.hpp"
#include "cx_utils/LockSharedPtr.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;

namespace cx {

NavGraphGenerateWaitzones::NavGraphGenerateWaitzones()
    : Node("ros_service_requester_feature_node") {}
NavGraphGenerateWaitzones::~NavGraphGenerateWaitzones() {}

std::string NavGraphGenerateWaitzones::getFeatureName() const {
  return clips_feature_name;
}

void NavGraphGenerateWaitzones::initialise(const std::string &feature_name) {
  clips_feature_name = feature_name;

  spin_thread_ =
      std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });
}

bool NavGraphGenerateWaitzones::clips_context_init(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialising context for feature %s",
              clips_feature_name.c_str());

  envs_[env_name] = clips;

  // add base implementations for ros communication
  // all of these need to be implemented given the corresponding types
  clips->add_function(
      "ros-nav_graph_generate_waitzones-create-request",
      sigc::slot<void, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this, &NavGraphGenerateWaitzones::create_request),
          env_name)));
  clips->add_function(
      "ros-nav_graph_generate_waitzones-set-field-request",
      sigc::slot<void, std::string, std::string, CLIPS::Value>(sigc::bind<0>(
          sigc::mem_fun(*this, &NavGraphGenerateWaitzones::set_field_request),
          env_name)));
  clips->add_function(
      "ros-nav_graph_generate_waitzones-set-array-request",
      sigc::slot<void, std::string, std::string, CLIPS::Values>(sigc::bind<0>(
          sigc::mem_fun(*this, &NavGraphGenerateWaitzones::set_array_request),
          env_name)));
  clips->add_function(
      "ros-nav_graph_generate_waitzones-request",
      sigc::slot<void, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this, &NavGraphGenerateWaitzones::request_from_node),
          env_name)));

  // add base fact templates
  clips->build("(deftemplate ros-nav_graph_generate_waitzones-response\
            (slot service (type STRING)) \
            (slot success (type STRING)) \
            )");

  return true;
}

bool NavGraphGenerateWaitzones::clips_context_destroyed(
    const std::string &env_name) {

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Destroying clips context!");
  envs_.erase(env_name);

  return true;
}

void NavGraphGenerateWaitzones::create_request(
    const std::string &env_name, const std::string &service_name) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Creating request for service %s %s", service_name.c_str(),
              env_name.c_str());

  request_clients_[env_name][service_name] =
      this->create_client<cx_msgs::srv::NavGraphInterfaceGenerateWaitzones>(service_name);
  requests_[env_name][service_name] =
      std::make_shared<cx_msgs::srv::NavGraphInterfaceGenerateWaitzones::Request>();
}

void NavGraphGenerateWaitzones::set_field_request(
    const std::string &env_name, const std::string &service_name,
    const std::string &field_name, CLIPS::Value value) {
  (void)service_name;
  (void)env_name;
  (void)value;
  (void)field_name;
}

void NavGraphGenerateWaitzones::set_array_request(
    const std::string &env_name, const std::string &service_name,
    const std::string &field_name, CLIPS::Values values) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Setting array %s for service %s %s not supported ",
              field_name.c_str(), service_name.c_str(), env_name.c_str());
  (void)service_name;
  (void)field_name;
  (void)env_name;
  (void)values;
}

void NavGraphGenerateWaitzones::request_from_node(
    const std::string &env_name, const std::string &service_name) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Requesting service %s %s", service_name.c_str(),
              env_name.c_str());

  auto response_callback =
      [=](rclcpp::Client<cx_msgs::srv::NavGraphInterfaceGenerateWaitzones>::SharedFuture response) {
        service_callback(response, service_name, env_name);
      };
  request_clients_[env_name][service_name]->async_send_request(
      requests_[env_name][service_name], response_callback);
}

void NavGraphGenerateWaitzones::service_callback(
    rclcpp::Client<cx_msgs::srv::NavGraphInterfaceGenerateWaitzones>::SharedFuture response,
    std::string service_name, std::string env_name) {
  cx::LockSharedPtr<CLIPS::Environment> &clips = envs_[env_name];
  std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));

  // remove old responses facts
  std::vector<CLIPS::Fact::pointer> facts = {};
  CLIPS::Fact::pointer old_fact = envs_[env_name]->get_facts();
  while (old_fact) {
    if (old_fact->get_template()->name() == "ros-nav_graph_generate_waitzones-response" &&
        old_fact->slot_value("service")[0].as_string() == service_name) {
      facts.push_back(old_fact);
    }
    old_fact = old_fact->next();
  }
  for (auto &old_fact : facts) {
    old_fact->retract();
  }

  // assert the newest responses
  auto result = response.get();
    CLIPS::Template::pointer fact_template = envs_[env_name]->get_template("ros-nav_graph_generate_waitzones-message");
  CLIPS::Fact::pointer fact = CLIPS::Fact::create(*(envs_[env_name].get_obj()), fact_template);

  fact->set_slot("success", result->success ? "TRUE" : "FALSE");
  fact->set_slot("service", service_name);

  envs_[env_name]->assert_fact(fact);
}

} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::NavGraphGenerateWaitzones, cx::ClipsFeature)