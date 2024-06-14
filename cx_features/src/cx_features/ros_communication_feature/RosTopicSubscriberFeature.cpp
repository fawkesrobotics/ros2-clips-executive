// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  RosTopicSubscriberFeature.cpp
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
#include "cx_features/RosTopicSubscriberFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;

namespace cx {

RosTopicSubscriberFeature::RosTopicSubscriberFeature()
    : Node("ros_topic_feature_node") {}
RosTopicSubscriberFeature::~RosTopicSubscriberFeature() {}

std::string RosTopicSubscriberFeature::getFeatureName() const {
  return clips_feature_name;
}

void RosTopicSubscriberFeature::initialise(const std::string &feature_name) {
  clips_feature_name = feature_name;

  spin_thread_ =
      std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });
}

bool RosTopicSubscriberFeature::clips_context_init(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialising context for feature %s",
              clips_feature_name.c_str());

  envs_[env_name] = clips;

  // add functions for subscribing and unsubscribing to the clips environment
  clips->add_function(
      "ros-" + msg_name_ + "-subscribe",
      sigc::slot<void, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this, &RosTopicSubscriberFeature::subscribe_to_topic),
          env_name)));
  clips->add_function(
      "ros-" + msg_name_ + "-unsubscribe",
      sigc::slot<void, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this,
                        &RosTopicSubscriberFeature::unsubscribe_from_topic),
          env_name)));

  // add fact templates
  clips->build("(deftemplate ros-" + msg_name_ + "-subscribed \
            (slot topic (type STRING)))");
  clips->build("(deftemplate ros-" + msg_name_ + "-message \
            (slot topic (type STRING)) (slot data (type STRING)))");

  return true;
}

bool RosTopicSubscriberFeature::clips_context_destroyed(
    const std::string &env_name) {

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Destroying clips context!");
  envs_.erase(env_name);

  return true;
}

void RosTopicSubscriberFeature::subscribe_to_topic(
    const std::string &env_name, const std::string &topic_name) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "Subscribing to topic %s",
              topic_name.c_str());

  auto it = subscriptions_[env_name].find(topic_name);

  if (it != subscriptions_[env_name].end()) {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
                "Already subscribed to topic %s", topic_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
                "Creating subscription to topic %s", topic_name.c_str());
    subscriptions_[env_name][topic_name] =
        this->create_subscription<std_msgs::msg::String>(
            topic_name, 10, [=](const std_msgs::msg::String::SharedPtr msg) {
              topic_callback(msg, topic_name, env_name);
            });
    envs_[env_name]->assert_fact("(ros-" + msg_name_ + "-subscribed (topic \"" +
                                 topic_name + "\"))");
  }
}

void RosTopicSubscriberFeature::unsubscribe_from_topic(
    const std::string &env_name, const std::string &topic_name) {

  auto it = subscriptions_[env_name].find(topic_name);

  if (it != subscriptions_[env_name].end()) {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
                "Unsubscribing from topic %s", topic_name.c_str());
    subscriptions_[env_name].erase(topic_name);

    // remove old message facts
    std::vector<CLIPS::Fact::pointer> facts = {};
    CLIPS::Fact::pointer fact = envs_[env_name]->get_facts();
    while (fact) {
      if ((fact->get_template()->name() == "ros-" + msg_name_ + "-message" &&
           fact->slot_value("topic")[0].as_string() == topic_name) ||
          fact->get_template()->name() == "ros-" + msg_name_ + "-subscribed") {
        facts.push_back(fact);
      }
      fact = fact->next();
    }
    for (auto &fact : facts) {
      fact->retract();
    }
  }
}

void RosTopicSubscriberFeature::topic_callback(
    const std_msgs::msg::String::SharedPtr msg, std::string topic_name,
    std::string env_name) {
  cx::LockSharedPtr<CLIPS::Environment> &clips = envs_[env_name];
  std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));

  // remove old message facts
  std::vector<CLIPS::Fact::pointer> facts = {};
  CLIPS::Fact::pointer fact = envs_[env_name]->get_facts();
  while (fact) {
    if (fact->get_template()->name() == "ros-" + msg_name_ + "-message" &&
        fact->slot_value("topic")[0].as_string() == topic_name) {
      facts.push_back(fact);
    }
    fact = fact->next();
  }
  for (auto &fact : facts) {
    fact->retract();
  }

  // assert the newest message
  envs_[env_name]->assert_fact("(ros-" + msg_name_ + "-message (topic \"" +
                               topic_name + "\") (data \"" + msg->data +
                               "\"))");
}

} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::RosTopicSubscriberFeature, cx::ClipsFeature)
