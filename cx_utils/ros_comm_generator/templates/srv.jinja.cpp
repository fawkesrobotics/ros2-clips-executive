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
#include <chrono>
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
  clips::Deftemplate *curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-client");
  clips::Undeftemplate(curr_tmpl, envs_[env_name].get_obj().get());
  curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-service");
  clips::Undeftemplate(curr_tmpl, envs_[env_name].get_obj().get());
  curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-request");
  clips::Undeftemplate(curr_tmpl, envs_[env_name].get_obj().get());
  curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-response");
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
{% set template_type = "Request" %}
{% include 'get_field.jinja.cpp' with context %}
{% include 'set_field.jinja.cpp' with context %}
{% include 'create.jinja.cpp' with context %}
{% include 'destroy.jinja.cpp' with context %}
{% set template_type = "Response" %}
{% include 'get_field.jinja.cpp' with context %}
{% include 'set_field.jinja.cpp' with context %}
{% include 'create.jinja.cpp' with context %}
{% include 'destroy.jinja.cpp' with context %}

  std::string fun_name;

  fun_name = "{{name_kebab}}-create-service";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";sy",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue service_name;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &service_name);

      instance->create_new_service(env, service_name.lexemeValue->contents);
    },
    "create_new_service", this);

  fun_name = "{{name_kebab}}-destroy-service";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";sy",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue service_name;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &service_name);

      instance->destroy_service(env, service_name.lexemeValue->contents);
    },
    "destroy_service", this);

  fun_name = "{{name_kebab}}-create-client";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";s",
    [](clips::Environment *env, clips::UDFContext *udfc, clips::UDFValue * /*out*/) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        clips::UDFValue service_name;

        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &service_name);
        instance->create_new_client(env,service_name.lexemeValue->contents);
    },
    "create_new_client", this);

  fun_name = "{{name_kebab}}-destroy-client";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";s",
    [](clips::Environment *env, clips::UDFContext *udfc, clips::UDFValue * /*out*/) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        clips::UDFValue service_name;

        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &service_name);
        instance->destroy_client(env, service_name.lexemeValue->contents);
    },
    "destroy_client", this);

  fun_name = "{{name_kebab}}-send-request";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 2, 2, ";e;sy",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue request, service_name;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &request);
      clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &service_name);

      instance->send_request(env, static_cast<{{message_type}}::Request*>(request.externalAddressValue->contents), service_name.lexemeValue->contents);
    },
    "send_request", this);


  // add fact templates
  clips::Build(clips.get_obj().get(),"(deftemplate {{name_kebab}}-client \
            (slot service (type STRING)))");
  clips::Build(clips.get_obj().get(),"(deftemplate {{name_kebab}}-service \
            (slot name (type STRING)))");
  clips::Build(clips.get_obj().get(),"(deftemplate {{name_kebab}}-response \
            (slot service (type STRING) ) \
            (slot msg-ptr (type EXTERNAL-ADDRESS)) \
            )");


  return true;
}

{% set template_part = "definition" %}
{% set template_slots = request_slots %}
{% set template_type = "Request" %}
{% include 'set_field.jinja.cpp' with context %}
{% include 'get_field.jinja.cpp' with context %}
{% include 'create.jinja.cpp' with context %}
{% include 'destroy.jinja.cpp' with context %}

{% set template_slots = response_slots %}
{% set template_type = "Response" %}
{% include 'set_field.jinja.cpp' with context %}
{% include 'get_field.jinja.cpp' with context %}
{% include 'create.jinja.cpp' with context %}
{% include 'destroy.jinja.cpp' with context %}

void {{name_camel}}::send_request(clips::Environment *env, {{message_type}}::Request *req, const std::string &service_name) {
  using namespace std::chrono_literals;
  {{message_type}}::Request::SharedPtr req_shared = std::make_shared<{{message_type}}::Request>(*req);
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

   // Handle the result asynchronously to not block clips engine potentially endlessly
  std::thread([this, req_shared, service_name, env_name]() {
  while (!clients_[env_name][service_name]->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "service %s not available, waiting again...", service_name.c_str());
  }
  cx::LockSharedPtr<clips::Environment> &clips = envs_[env_name];
  std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));
  auto future = clients_[env_name][service_name]->async_send_request(req_shared);
  auto resp = future.get();
   responses_.try_emplace(resp.get(), resp);
    clips::FactBuilder *fact_builder = clips::CreateFactBuilder(clips.get_obj().get(), "{{name_kebab}}-response");
    clips::FBPutSlotString(fact_builder,"service",service_name.c_str());
    clips::FBPutSlotCLIPSExternalAddress(fact_builder,"msg-ptr", clips::CreateCExternalAddress(clips.get_obj().get(), resp.get()));
    clips::FBAssert(fact_builder);
    clips::FBDispose(fact_builder);

  }).detach();
}

void {{name_camel}}::create_new_service(clips::Environment *env, const std::string &service_name) {
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
  services_[env_name][service_name] =
      this->create_service<{{message_type}}>(service_name, [this, env_name, service_name](const std::shared_ptr<{{message_type}}::Request> request,
    std::shared_ptr<{{message_type}}::Response> response) {
    this->service_callback(request, response, service_name, env_name);
  });
    clips::AssertString(envs_[env_name].get_obj().get(), ("({{name_kebab}}-service (name \"" +
                                 service_name + "\"))").c_str());
}

void {{name_camel}}::destroy_service(clips::Environment *env, const std::string &service_name) {
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

  auto outer_it = services_.find(env_name);
  if (outer_it != services_.end()) {
      // Check if service_name exists in the inner map
      auto& inner_map = outer_it->second;
      auto inner_it = inner_map.find(service_name);
      if (inner_it != inner_map.end()) {
          // Remove the service_name entry from the inner map
          inner_map.erase(inner_it);
      } else {
          RCLCPP_WARN(this->get_logger(), "Service %s not found in environment %s", service_name.c_str(), env_name.c_str());
      }
  } else {
      RCLCPP_WARN(this->get_logger(), "Environment %s not found", env_name.c_str());
  }
}

void {{name_camel}}::create_new_client(clips::Environment *env,
    const std::string &service_name) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "Creating client for serive %s",
              service_name.c_str());
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

  auto it = clients_[env_name].find(service_name);

  if (it != clients_[env_name].end()) {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
                "There already exists a client for service %s", service_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
                "Creating client for service %s", service_name.c_str());
    clients_[env_name][service_name] =
        this->create_client<{{message_type}}>(service_name);
    clips::AssertString(envs_[env_name].get_obj().get(), ("({{name_kebab}}-client (service \"" + service_name + "\"))").c_str());
  }
}

void {{name_camel}}::destroy_client(clips::Environment *env,
    const std::string &service_name) {
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

  auto it = clients_[env_name].find(service_name);

  if (it != clients_[env_name].end()) {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
                "Destroying client for service %s", service_name.c_str());
    clients_[env_name].erase(service_name);
  }

  clips::Eval(env, ("(do-for-all-facts ((?f {{name_kebab}}-client)) (eq (str-cat ?f:service) (str-cat " + service_name + "))  (retract ?f))").c_str(), NULL);
}

void {{name_camel}}::service_callback(const std::shared_ptr<{{message_type}}::Request> request,
                        std::shared_ptr<{{message_type}}::Response> response,
    std::string service_name,
    std::string env_name) {
  cx::LockSharedPtr<clips::Environment> &clips = envs_[env_name];
  std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));

  // call a user-defined function
  clips::Deffunction *dec_fun = clips::FindDeffunction(clips.get_obj().get(),"{{name_kebab}}-service-callback");
  if(!dec_fun) {
    RCLCPP_WARN(get_logger(), "{{name_kebab}}-service-callback not defined, skip callback");
    return;
  }
  clips::FunctionCallBuilder *fcb = clips::CreateFunctionCallBuilder(clips.get_obj().get(),3);
  clips::FCBAppendString(fcb, service_name.c_str());
  clips::FCBAppendCLIPSExternalAddress(fcb, clips::CreateCExternalAddress(clips.get_obj().get(), request.get()));
  clips::FCBAppendCLIPSExternalAddress(fcb, clips::CreateCExternalAddress(clips.get_obj().get(), response.get()));
  clips::FCBCall(fcb,"{{name_kebab}}-service-callback",NULL);
  clips::FCBDispose(fcb);
}

} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::{{name_camel}}, cx::ClipsFeature)
