// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  {{name_camel}}.cpp
 *
 *  Automatically Generated: {{ gen_date }}
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
// clang-format off
#include <map>
#include <memory>
#include <string>

#include "cx_core/ClipsFeature.hpp"
#include "{{name_snake}}.hpp"
#include "cx_utils/LockSharedPtr.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;

namespace cx {

{{name_camel}}::{{name_camel}}()
    : Node("{{name_snake}}_msg_feature_node") {}
{{name_camel}}::~{{name_camel}}() {}

std::string {{name_camel}}::getFeatureName() const {
  return clips_feature_name;
}

void {{name_camel}}::initialise(const std::string &feature_name) {
  clips_feature_name = feature_name;

  spin_thread_ =
      std::thread([this]() { rclcpp::spin(this->get_node_base_interface()); });
}

bool {{name_camel}}::clips_context_destroyed(
    const std::string &env_name) {

  RCLCPP_INFO(get_logger(),
              "Destroying clips context!");
  for(const auto& fun : function_names_) {
     clips::RemoveUDF(envs_[env_name].get_obj().get(), fun.c_str());
  }
  clips::Deftemplate *curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-subscriber");
  clips::Undeftemplate(curr_tmpl, envs_[env_name].get_obj().get());
  curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-publisher");
  clips::Undeftemplate(curr_tmpl, envs_[env_name].get_obj().get());
  curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-msg");
  clips::Undeftemplate(curr_tmpl, envs_[env_name].get_obj().get());
  envs_.erase(env_name);

  return true;
}

bool {{name_camel}}::clips_context_init(const std::string &env_name,
    LockSharedPtr<clips::Environment> &clips) {
  RCLCPP_INFO(get_logger(),
              "Initialising context for feature %s",
              clips_feature_name.c_str());

  envs_[env_name] = clips;

{% set template_part = "registration" %}
{% set template_type = "" %}
{% include 'get_field.jinja.cpp' with context %}
{% include 'set_field.jinja.cpp' with context %}

  std::string fun_name;

  fun_name = "{{name_kebab}}-msg-create";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "e", 0, 0, "",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue *out) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      using namespace clips;
      *out = instance->create_message(env);
    },
    "create_message", this);

  fun_name = "{{name_kebab}}-create-publisher";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";sy",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue topic;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &topic);

      instance->create_new_publisher(env, topic.lexemeValue->contents);
    },
    "create_new_publisher", this);

  fun_name = "{{name_kebab}}-destroy-publisher";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";sy",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue topic;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &topic);

      instance->destroy_publisher(env, topic.lexemeValue->contents);
    },
    "destroy_publisher", this);

  fun_name = "{{name_kebab}}-publish";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 2, 2, ";e;sy",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue message, topic;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &message);
      clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &topic);

      instance->publish_to_topic(env, static_cast<{{message_type}}*>(message.externalAddressValue->contents), topic.lexemeValue->contents);
    },
    "publish_to_topic", this);

  fun_name = "{{name_kebab}}-msg-destroy";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";e",
    [](clips::Environment */*env*/, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue message;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &message);

      instance->destroy_msg(static_cast<{{message_type}}*>(message.externalAddressValue->contents));
    },
    "destroy_msg", this);

  fun_name = "{{name_kebab}}-create-subscriber";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";s",
    [](clips::Environment *env, clips::UDFContext *udfc, clips::UDFValue * /*out*/) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        clips::UDFValue topic;

        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &topic);
        instance->subscribe_to_topic(env,topic.lexemeValue->contents);
    },
    "subscribe_to_topic", this);

  fun_name = "{{name_kebab}}-destroy-subscriber";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";s",
    [](clips::Environment *env, clips::UDFContext *udfc, clips::UDFValue * /*out*/) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        clips::UDFValue topic;

        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &topic);
        instance->unsubscribe_from_topic(env, topic.lexemeValue->contents);
    },
    "unsubscribe_from_topic", this);


  // add fact templates
  clips::Build(clips.get_obj().get(),"(deftemplate {{name_kebab}}-subscriber \
            (slot topic (type STRING)))");
  clips::Build(clips.get_obj().get(),"(deftemplate {{name_kebab}}-publisher \
            (slot topic (type STRING)))");
  clips::Build(clips.get_obj().get(),"(deftemplate {{name_kebab}}-msg \
            (slot topic (type STRING) ) \
            (slot msg-ptr (type EXTERNAL-ADDRESS)) \
            )");


  return true;
}

{% set template_part = "definition" %}
{% set template_slots = slots %}
{% set template_type = "" %}
{% include 'get_field.jinja.cpp' with context %}
{% include 'set_field.jinja.cpp' with context %}

void {{name_camel}}::publish_to_topic(clips::Environment *env, {{message_type}} *msg, const std::string &topic_name) {
  bool found_env = false;
  std::string env_name;

  for (auto &entry : envs_) {
    if (entry.second.get_obj().get() == env) {
      env_name = entry.first;
      found_env = true;
      break;
    }
  }
  if (!found_env) {
    RCLCPP_ERROR(get_logger(),
                 "Unable to determine environment from raw pointer");
    return;
  }
  publishers_[env_name][topic_name]->publish(*msg);
}

void {{name_camel}}::create_new_publisher(clips::Environment *env, const std::string &topic_name) {
  bool found_env = false;
  std::string env_name;

  for (auto &entry : envs_) {
    if (entry.second.get_obj().get() == env) {
      env_name = entry.first;
      found_env = true;
      break;
    }
  }
  if (!found_env) {
    RCLCPP_ERROR(get_logger(),
                 "Unable to determine environment from raw pointer");
    return;
  }
  publishers_[env_name][topic_name] =
      this->create_publisher<{{message_type}}>(topic_name, 10);
    clips::AssertString(envs_[env_name].get_obj().get(), ("({{name_kebab}}-publisher (topic \"" +
                                 topic_name + "\"))").c_str());
}

void {{name_camel}}::destroy_publisher(clips::Environment *env, const std::string &topic_name) {
  bool found_env = false;
  std::string env_name;

  for (auto &entry : envs_) {
    if (entry.second.get_obj().get() == env) {
      env_name = entry.first;
      found_env = true;
      break;
    }
  }
  if (!found_env) {
    RCLCPP_ERROR(get_logger(),
                 "Unable to determine environment from raw pointer");
    return;
  }

  auto outer_it = publishers_.find(env_name);
  if (outer_it != publishers_.end()) {
      // Check if topic_name exists in the inner map
      auto& inner_map = outer_it->second;
      auto inner_it = inner_map.find(topic_name);
      if (inner_it != inner_map.end()) {
          // Remove the topic_name entry from the inner map
          inner_map.erase(inner_it);
      } else {
          RCLCPP_WARN(this->get_logger(), "Topic %s not found in environment %s", topic_name.c_str(), env_name.c_str());
      }
  } else {
      RCLCPP_WARN(this->get_logger(), "Environment %s not found", env_name.c_str());
  }
}

void {{name_camel}}::subscribe_to_topic(clips::Environment *env,
    const std::string &topic_name) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "Subscribing to topic %s",
              topic_name.c_str());
  bool found_env = false;
  std::string env_name;

  for (auto &entry : envs_) {
    if (entry.second.get_obj().get() == env) {
      env_name = entry.first;
      found_env = true;
      break;
    }
  }
  if (!found_env) {
    RCLCPP_ERROR(rclcpp::get_logger(clips_feature_name),
                 "Unable to determine environment from raw pointer");
    return;
  }

  auto it = subscriptions_[env_name].find(topic_name);

  if (it != subscriptions_[env_name].end()) {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
                "Already subscribed to topic %s", topic_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
                "Creating subscription to topic %s", topic_name.c_str());
    subscriptions_[env_name][topic_name] =
        this->create_subscription<{{message_type}}>(
            topic_name, 10, [this,topic_name, env_name](const {{message_type}}::SharedPtr msg) {
              topic_callback(msg, topic_name, env_name);
            });
    clips::AssertString(envs_[env_name].get_obj().get(), ("({{name_kebab}}-subscriber (topic \"" +
                                 topic_name + "\"))").c_str());
  }
}

void {{name_camel}}::unsubscribe_from_topic(clips::Environment *env,
    const std::string &topic_name) {
  bool found_env = false;
  std::string env_name;

  for (auto &entry : envs_) {
    if (entry.second.get_obj().get() == env) {
      env_name = entry.first;
      found_env = true;
      break;
    }
  }
  if (!found_env) {
    RCLCPP_ERROR(rclcpp::get_logger(clips_feature_name),
                 "Unable to determine environment from raw pointer");
    return;
  }

  auto it = subscriptions_[env_name].find(topic_name);

  if (it != subscriptions_[env_name].end()) {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
                "Unsubscribing from topic %s", topic_name.c_str());
    subscriptions_[env_name].erase(topic_name);
  }

  clips::Eval(env, ("(do-for-all-facts ((?f {{name_kebab}}-subscriber)) (eq (str-cat ?f:topic) (str-cat " + topic_name + "))  (retract ?f))").c_str(), NULL);
  clips::Eval(env, ("(do-for-all-facts ((?f {{name_kebab}}-msg)) (eq (str-cat ?f:topic) (str-cat " + topic_name + "))  (retract ?f))").c_str(), NULL);
}

void {{name_camel}}::topic_callback(
    const {{message_type}}::SharedPtr msg, std::string topic_name,
    std::string env_name) {
  cx::LockSharedPtr<clips::Environment> &clips = envs_[env_name];
  std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));
  messages_[msg.get()] = msg;

  // assert the newest message
  clips::FactBuilder *fact_builder = clips::CreateFactBuilder(clips.get_obj().get(), "{{name_kebab}}-msg");
  clips::FBPutSlotString(fact_builder,"topic",topic_name.c_str());
  clips::FBPutSlotCLIPSExternalAddress(fact_builder,"msg-ptr", clips::CreateCExternalAddress(clips.get_obj().get(), msg.get()));
  clips::FBAssert(fact_builder);
  clips::FBDispose(fact_builder);
}

clips::UDFValue {{name_camel}}::create_message(clips::Environment *env) {
  std::shared_ptr<{{message_type}}> ptr = std::make_shared<{{message_type}}>();
  messages_[ptr.get()] = ptr;
  clips::UDFValue res;
  res.externalAddressValue = clips::CreateCExternalAddress(env, ptr.get());
  return res;
}

void {{name_camel}}::destroy_msg({{message_type}} *msg) {
  auto it = messages_.find(msg);
  if (it != messages_.end()) {
      messages_.erase(it);
  }
}

} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::{{name_camel}}, cx::ClipsFeature)
