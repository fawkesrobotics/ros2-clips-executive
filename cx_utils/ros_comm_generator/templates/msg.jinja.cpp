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
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-create-subscriber");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-destroy-subscriber");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-set-field");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-get-field");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-create-msg");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-create-publisher");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-destroy-publisher");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-publish");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "{{name_kebab}}-destroy-msg");
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
  clips::AddUDF(
      clips.get_obj().get(), "{{name_kebab}}-set-field", "v", 3, 3, ";e;sy;*",
      [](clips::Environment */*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        clips::UDFValue message, field, value;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &message);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field);
        clips::UDFNthArgument(udfc, 3, ANY_TYPE_BITS, &value);

        instance->set_field_publish(static_cast<{{message_type}}*>(message.externalAddressValue->contents), field.lexemeValue->contents, value, udfc);
      },
      "set_field_publish", this);
  clips::AddUDF(
      clips.get_obj().get(), "{{name_kebab}}-get-field", "*", 2, 2, ";e;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        clips::UDFValue message, field;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &message);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field);
        *out = instance->get_field(env, static_cast<{{message_type}}*>(message.externalAddressValue->contents), field.lexemeValue->contents);
      },
      "get_field", this);

  clips::AddUDF(
      clips.get_obj().get(), "{{name_kebab}}-create-msg", "e", 0, 0, "",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        using namespace clips;
        *out = instance->create_message(env);
      },
      "create_message", this);

  clips::AddUDF(
    clips.get_obj().get(), "{{name_kebab}}-create-publisher", "v", 1, 1, ";sy",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue topic;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &topic);

      instance->create_new_publisher(env, topic.lexemeValue->contents);
    },
    "create_new_publisher", this);

  clips::AddUDF(
    clips.get_obj().get(), "{{name_kebab}}-destroy-publisher", "v", 1, 1, ";sy",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue topic;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &topic);

      instance->destroy_publisher(env, topic.lexemeValue->contents);
    },
    "destroy_publisher", this);

  clips::AddUDF(
    clips.get_obj().get(), "{{name_kebab}}-publish", "v", 2, 2, ";e;sy",
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

  clips::AddUDF(
    clips.get_obj().get(), "{{name_kebab}}-destroy-msg", "v", 1, 1, ";e",
    [](clips::Environment */*env*/, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue message;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &message);

      instance->destroy_msg(static_cast<{{message_type}}*>(message.externalAddressValue->contents));
    },
    "destroy_msg", this);

  clips::AddUDF(
    clips.get_obj().get(), "{{name_kebab}}-create-subscriber", "v", 1, 1, ";s",
    [](clips::Environment *env, clips::UDFContext *udfc, clips::UDFValue * /*out*/) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        clips::UDFValue topic;

        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &topic);
        instance->subscribe_to_topic(env,topic.lexemeValue->contents);
    },
    "subscribe_to_topic", this);

  clips::AddUDF(
    clips.get_obj().get(), "{{name_kebab}}-destroy-subscriber", "v", 1, 1, ";s",
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

clips::UDFValue {{name_camel}}::create_message(clips::Environment *env) {
  {{message_type}} *new_msg = new {{message_type}}();
  messages_.insert(new_msg);
  clips::UDFValue res;
  res.externalAddressValue = clips::CreateCExternalAddress(env, new_msg);
  return res;
}

   clips::UDFValue {{name_camel}}::get_field(clips::Environment *env,
                               {{message_type}} *msg,
                               const std::string &field) {
    clips::UDFValue res;
    size_t full_length = 0;
    clips::MultifieldBuilder *mb;
{%- for slot in slots %}
{%- if not slot.array %}
  if (field == "{{ slot.name }}") {
{%- if slot.clips_type == "STRING" %}
    res.lexemeValue = clips::CreateString(env, std::string(msg->{{ slot.name }}).c_str());
{%- endif %}
{%- if slot.clips_type == "INTEGER" %}
    res.integerValue = clips::CreateInteger(env, msg->{{ slot.name }});
{%- endif %}
{%- if slot.clips_type == "FLOAT" %}
    res.floatValue = clips::CreateFloat(env, msg->{{ slot.name }};
{%- endif %}
{%- if slot.clips_type == "BOOLEAN" %}
    res.lexemeValue = clips::CreateSymbol(env, msg->{{ slot.name }} ? "TRUE" : "FALSE");
{%- endif %}
{%- if slot.clips_type == "EXTERNAL-ADDRESS" %}
    res.externalAddressValue = clips::CreateCExternalAddress(env, &(msg->{{ slot.name }}));
{%- endif %}
  }
{%- else %}
  if (field == "{{ slot.name }}") {
  full_length = msg->{{ slot.name }}.size();
  mb = clips::CreateMultifieldBuilder(env, full_length);
  for (size_t i = 0; i < full_length; i++) {
{%- if slot.clips_type == "STRING" %}
      clips::MBAppendString(mb,std::string(msg->{{ slot.name }}[i]).c_str());
{%- endif %}
{%- if slot.clips_type == "INTEGER" %}
      clips::MBAppendInteger(mb,msg->{{ slot.name }}[i]);
{%- endif %}
{%- if slot.clips_type == "FLOAT" %}
      clips::MBAppendFloat(mb,msg->{{ slot.name }}[i]);
{%- endif %}
{%- if slot.clips_type == "BOOLEAN" %}
      clips::MBAppendSymbol(mb,msg->{{ slot.name }}[i] ? "TRUE" : "FALSE");
{%- endif %}
{%- if slot.clips_type == "EXTERNAL-ADDRESS" %}
      clips::MBAppendCLIPSExternalAddress(mb, clips::CreateCExternalAddress(env, &(msg->{{ slot.name }}[i])));
{%- endif %}
    }
  res.multifieldValue = clips::MBCreate(mb);
  clips::MBDispose(mb);
  }
{%- endif -%}
{%- endfor %}
  (void) full_length;
  (void) mb;
  return res;
}

void {{name_camel}}::set_field_publish({{message_type}} *msg,
                                       const std::string &field,
                                       clips::UDFValue value, clips::UDFContext *udfc) {
  clips::Multifield *multi;

{%- for slot in slots %}
{%- if not slot.array %}
  if (field == "{{ slot.name }}") {
{%- if slot.clips_type == "STRING" %}
    msg->{{ slot.name }} = value.lexemeValue->contents;
{%- endif %}
{%- if slot.clips_type == "INTEGER" %}
    msg->{{ slot.name }} = value.integerValue->contents;
{%- endif %}
{%- if slot.clips_type == "FLOAT" %}
    msg->{{ slot.name }} = value.floatValue->contents;
{%- endif %}
{%- if slot.clips_type == "BOOLEAN" %}
    msg->{{ slot.name }} = std::string(value.lexemeValue->contents).compare(std::string("TRUE")) ? false : true;
{%- endif %}
{%- if slot.clips_type == "EXTERNAL-ADDRESS" %}
    msg->{{ slot.name }} = *static_cast<{{slot.type}} *>(value.externalAddressValue->contents);
{%- endif %}
  }
{%- else %}
  if (field == "{{ slot.name }}") {
    multi = value.multifieldValue;
  {%- if slot.clips_type == "STRING" %}
    std::vector<{{slot.cpp_type}}> value_vector_{{slot.name}};
    for (size_t i = 0; i < multi->length; i++) {
      switch (multi->contents[i].header->type) {
      case STRING_TYPE:
      case SYMBOL_TYPE:
        value_vector_{{slot.name}}.push_back(multi->contents[i].lexemeValue->contents);
        break;
      default:
        RCLCPP_ERROR(get_logger(),
                     "Unexpected Type %i (expected STRING/SYMBOL) of %li nth argument of UDF {{name_kebab}}-set-field",
                     multi->contents[i].header->type, i);
        clips::UDFThrowError(udfc);
      }
    }

    msg->{{ slot.name }} = value_vector_{{slot.name}};
  {%- endif %}
  {%- if slot.clips_type == "INTEGER" %}
    std::vector<{{slot.cpp_type}}> value_vector_{{slot.name}};
    for (size_t i = 0; i < multi->length; i++) {
      switch (multi->contents[i].header->type) {
      case INTEGER_TYPE:
        value_vector_{{slot.name}}.push_back(multi->contents[i].integerType->contents);
        break;
      default:
        RCLCPP_ERROR(get_logger(),
                     "Unexpected Type %i (expected INTEGER) of %li nth argument of UDF {{name_kebab}}-set-field",
                     multi->contents[i].header->type, i);
        clips::UDFThrowError(udfc);
      }
    }
    msg->{{ slot.name }} = value_vector_{{slot.name}};
  {%- endif %}
  {%- if slot.clips_type == "FLOAT" %}
    std::vector<{{slot.cpp_type}}> value_vector_{{slot.name}};
    for (size_t i = 0; i < multi->length; i++) {
      switch (multi->contents[i].header->type) {
      case FLOAT_TYPE:
        value_vector_{{slot.name}}.push_back(multi->contents[i].floatType->contents);
        break;
      default:
        RCLCPP_ERROR(get_logger(),
                     "Unexpected Type %i (expected FLOAT) of %li nth argument of UDF {{name_kebab}}-set-field",
                     multi->contents[i].header->type, i);
        clips::UDFThrowError(udfc);
      }
    }
    msg->{{ slot.name }} = value_vector_{{slot.name}};
  {%- endif %}
  {%- if slot.clips_type == "EXTERNAL-ADDRESS" %}
    std::vector<{{slot.type}}> value_vector_{{slot.name}};
    for (size_t i = 0; i < multi->length; i++) {
      switch (multi->contents[i].header->type) {
      case EXTERNAL_ADDRESS_TYPE:
        value_vector_{{slot.name}}.push_back(*static_cast<{{slot.type}}*>((multi->contents[i].externalAddressValue->contents)));
        break;
      default:
        RCLCPP_ERROR(get_logger(),
                     "Unexpected Type %i (expected FLOAT) of %li nth argument of UDF {{name_kebab}}-set-field",
                     multi->contents[i].header->type, i);
        clips::UDFThrowError(udfc);
      }
    }
    msg->{{ slot.name }} = value_vector_{{slot.name}};
  {%- endif %}
  }
{%- endif -%}
{%- endfor %}
  (void)multi;
  (void)udfc;
}

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
            topic_name, 10, [=](const {{message_type}}::SharedPtr msg) {
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
  {{message_type}} *heap_msg = new {{message_type}}();
  *heap_msg = *msg;
   messages_.insert(heap_msg);

  // assert the newest message
  clips::FactBuilder *fact_builder = clips::CreateFactBuilder(clips.get_obj().get(), "{{name_kebab}}-msg");
  clips::FBPutSlotString(fact_builder,"topic",topic_name.c_str());
  clips::FBPutSlotCLIPSExternalAddress(fact_builder,"msg-ptr", clips::CreateCExternalAddress(clips.get_obj().get(), heap_msg));
  clips::FBAssert(fact_builder);
  clips::FBDispose(fact_builder);
}

void {{name_camel}}::destroy_msg({{message_type}} *msg) {
  auto it = messages_.find(msg);
  if (it != messages_.end()) {
      // If found, delete the element
      messages_.erase(it);
      delete msg;
  }
}


} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::{{name_camel}}, cx::ClipsFeature)
