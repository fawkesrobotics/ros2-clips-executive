// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  RosTopicPublisherFeature.cpp
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
#include "cx_features/RosTopicPublisherFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;

namespace cx {

RosTopicPublisherFeature::RosTopicPublisherFeature()
    : Node("ros_topic_publisher_feature_node") {}
RosTopicPublisherFeature::~RosTopicPublisherFeature() {}

std::string RosTopicPublisherFeature::getFeatureName() const {
  return clips_feature_name;
}

void RosTopicPublisherFeature::initialise(const std::string &feature_name) {
  clips_feature_name = feature_name;

  spin_thread_ =
      std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });
}

bool RosTopicPublisherFeature::clips_context_destroyed(
    const std::string &env_name) {

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Destroying clips context!");
  envs_.erase(env_name);

  return true;
}

bool RosTopicPublisherFeature::clips_context_init(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialising context for feature %s",
              clips_feature_name.c_str());

  envs_[env_name] = clips;

  clips->add_function(
      "ros-" + msg_name_ + "-set-field-publish",
      sigc::slot<void, std::string, std::string, CLIPS::Value>(sigc::bind<0>(
          sigc::mem_fun(*this, &RosTopicPublisherFeature::set_field_publish),
          env_name)));
  clips->add_function(
      "ros-" + msg_name_ + "-set-array-publish",
      sigc::slot<void, std::string, std::string, CLIPS::Values>(sigc::bind<0>(
          sigc::mem_fun(*this, &RosTopicPublisherFeature::set_array_publish),
          env_name)));
  clips->add_function(
      "ros-" + msg_name_ + "-create-message",
      sigc::slot<void, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this, &RosTopicPublisherFeature::create_message),
          env_name)));
  clips->add_function(
      "ros-" + msg_name_ + "-create-publisher",
      sigc::slot<void, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this, &RosTopicPublisherFeature::creater_publisher),
          env_name)));
  clips->add_function(
      "ros-" + msg_name_ + "-publish",
      sigc::slot<void, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this, &RosTopicPublisherFeature::publish_to_topic),
          env_name)));

  return true;
}

void RosTopicPublisherFeature::create_message(const std::string &env_name,
                                              const std::string &topic) {
  messages_[env_name][topic] = std_msgs::msg::String();
}

void RosTopicPublisherFeature::set_field_publish(const std::string &env_name,
                                                 const std::string &topic,
                                                 const std::string &field,
                                                 CLIPS::Value value) {
  if (field == "data") {
    messages_[env_name][topic].data = value.as_string();
  }
}

void RosTopicPublisherFeature::set_array_publish(const std::string &env_name,
                                                 const std::string &topic,
                                                 const std::string &field,
                                                 CLIPS::Values values) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Setting array %s for service %s %s not supported ",
              field.c_str(), topic.c_str(), env_name.c_str());
  (void)values;
}

void RosTopicPublisherFeature::publish_to_topic(const std::string &env_name,
                                                const std::string &topic_name) {
  publishers_[env_name][topic_name]->publish(messages_[env_name][topic_name]);
}

void RosTopicPublisherFeature::creater_publisher(
    const std::string &env_name, const std::string &topic_name) {
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  publishers_[env_name][topic_name] =
      this->create_publisher<std_msgs::msg::String>(topic_name, 10);
}

} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::RosTopicPublisherFeature, cx::ClipsFeature)
