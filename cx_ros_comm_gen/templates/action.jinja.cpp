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
    : Node("{{name_snake}}_msg_feature_node") {
    clips_worker_thread_ = std::thread([this] () {
  while (!stop_flag_) {
      std::function<void()> task;
      {
          std::unique_lock<std::mutex> lock(queue_mutex_);
          cv_.wait(lock, [&] { return !task_queue_.empty(); });
          task = std::move(task_queue_.front());
          task_queue_.pop();
      }
      task();
  }});
}

{{name_camel}}::~{{name_camel}}() {
  stop_flag_ = true;
  cv_.notify_all();

  if (clips_worker_thread_.joinable()) {
      clips_worker_thread_.join();
  }
}

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
  curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-server");
  clips::Undeftemplate(curr_tmpl, envs_[env_name].get_obj().get());
  curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-goal-response");
  clips::Undeftemplate(curr_tmpl, envs_[env_name].get_obj().get());
  curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-goal-feedback");
  clips::Undeftemplate(curr_tmpl, envs_[env_name].get_obj().get());
  curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-wrapped-result");
  clips::Undeftemplate(curr_tmpl, envs_[env_name].get_obj().get());
  curr_tmpl = clips::FindDeftemplate(envs_[env_name].get_obj().get(), "{{name_kebab}}-accepted-goal");
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
{% set template_type = "Goal" %}
{% include 'get_field.jinja.cpp' with context %}
{% include 'set_field.jinja.cpp' with context %}
{% include 'create.jinja.cpp' with context %}
{% include 'destroy.jinja.cpp' with context %}
{% set template_type = "Feedback" %}
{% include 'get_field.jinja.cpp' with context %}
{% include 'set_field.jinja.cpp' with context %}
{% include 'create.jinja.cpp' with context %}
{% include 'destroy.jinja.cpp' with context %}
{% set template_type = "Result" %}
{% include 'get_field.jinja.cpp' with context %}
{% include 'set_field.jinja.cpp' with context %}
{% include 'create.jinja.cpp' with context %}
{% include 'destroy.jinja.cpp' with context %}
  std::string fun_name;

  fun_name = "{{name_kebab}}-create-server";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";sy",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue server_name;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &server_name);

      instance->create_new_server(env, server_name.lexemeValue->contents);
    },
    "create_new_server", this);

  fun_name = "{{name_kebab}}-destroy-server";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";sy",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue server_name;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &server_name);

      instance->destroy_server(env, server_name.lexemeValue->contents);
    },
    "destroy_server", this);

  fun_name = "{{name_kebab}}-create-client";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";s",
    [](clips::Environment *env, clips::UDFContext *udfc, clips::UDFValue * /*out*/) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        clips::UDFValue server_name;

        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &server_name);
        instance->create_new_client(env,server_name.lexemeValue->contents);
    },
    "create_new_client", this);

  fun_name = "{{name_kebab}}-destroy-client";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";s",
    [](clips::Environment *env, clips::UDFContext *udfc, clips::UDFValue * /*out*/) {
        auto *instance = static_cast<{{name_camel}} *>(udfc->context);
        clips::UDFValue server_name;

        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &server_name);
        instance->destroy_client(env, server_name.lexemeValue->contents);
    },
    "destroy_client", this);

  fun_name = "{{name_kebab}}-send-goal";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 2, 2, ";e;sy",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue goal, server_name;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &goal);
      clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &server_name);

      instance->send_goal(env, static_cast<{{message_type}}::Goal*>(goal.externalAddressValue->contents), server_name.lexemeValue->contents);
    },
    "send_goal", this);

  fun_name = "{{name_kebab}}-server-goal-handle-abort";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 2, 2, ";e;e",
    [](clips::Environment * /*env*/, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue goal_handle, result;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &goal_handle);
      clips::UDFNthArgument(udfc, 2, EXTERNAL_ADDRESS_BIT, &result);

       try {
        instance->server_goal_handle_abort(goal_handle.externalAddressValue->contents, result.externalAddressValue->contents, udfc);
       } catch (std::out_of_range &e) {
         RCLCPP_ERROR(instance->get_logger(), "Unknown goal handle or result pointer %s", e.what());
       }
    },
    "server_goal_handle_abort", this);

  fun_name = "{{name_kebab}}-server-goal-handle-succeed";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 2, 2, ";e;e",
    [](clips::Environment * /*env*/, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue goal_handle, result;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &goal_handle);
      clips::UDFNthArgument(udfc, 2, EXTERNAL_ADDRESS_BIT, &result);

       try {
        instance->server_goal_handle_succeed(goal_handle.externalAddressValue->contents, result.externalAddressValue->contents, udfc);
       } catch (std::out_of_range &e) {
         RCLCPP_ERROR(instance->get_logger(), "Unknown goal handle or result pointer %s", e.what());
       }
    },
    "server_goal_handle_succeed", this);

  fun_name = "{{name_kebab}}-server-goal-handle-canceled";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 2, 2, ";e;e",
    [](clips::Environment * /*env*/, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue goal_handle, result;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &goal_handle);
      clips::UDFNthArgument(udfc, 2, EXTERNAL_ADDRESS_BIT, &result);

       try {
        instance->server_goal_handle_canceled(goal_handle.externalAddressValue->contents, result.externalAddressValue->contents, udfc);
       } catch (std::out_of_range &e) {
         RCLCPP_ERROR(instance->get_logger(), "Unknown goal handle or result pointer %s", e.what());
       }
    },
    "server_goal_handle_canceled", this);

  fun_name = "{{name_kebab}}-server-goal-handle-get-goal";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "e", 1, 1, ";e",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue *out) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue goal_handle;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &goal_handle);
       try {
        *out = instance->server_goal_handle_get_goal(env, goal_handle.externalAddressValue->contents, udfc);
       } catch (std::out_of_range &e) {
         RCLCPP_ERROR(instance->get_logger(), "Unknown goal handle %s", e.what());
       }
    },
    "server_goal_handle_get_goal", this);

  fun_name = "{{name_kebab}}-server-goal-handle-get-goal-id";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "m", 1, 1, ";e",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue *out) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue goal_handle;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &goal_handle);
       try {
         *out = instance->server_goal_handle_get_goal_id(env, goal_handle.externalAddressValue->contents, udfc);
       } catch (std::out_of_range &e) {
         RCLCPP_ERROR(instance->get_logger(), "Unknown goal handle %s", e.what());
       }
    },
    "server_goal_handle_get_goal_id", this);

{% set template_type = "ServerGoalHandle" %}
{% set template_call_fun = "IsCanceling" %}
{% set template_ret_type = "Bool" %}
{% include 'ret_fun.jinja.cpp' with context %}
{% set template_call_fun = "IsActive" %}
{% include 'ret_fun.jinja.cpp' with context %}
{% set template_call_fun = "IsExecuting" %}
{% include 'ret_fun.jinja.cpp' with context %}
{% set template_call_fun = "Execute" %}
{% include 'void_fun.jinja.cpp' with context %}

{% set template_type = "ClientGoalHandle" %}
{% set template_call_fun = "IsFeedbackAware" %}
{% include 'ret_fun.jinja.cpp' with context %}
{% set template_call_fun = "IsResultAware" %}
{% include 'ret_fun.jinja.cpp' with context %}
{% set template_call_fun = "GetStatus" %}
{% set template_ret_type = "Integer" %}
{% include 'ret_fun.jinja.cpp' with context %}

  fun_name = "{{name_kebab}}-server-goal-handle-publish-feedback";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "v", 2, 2, ";e;e",
    [](clips::Environment * /*env*/, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue goal_handle, feedback;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &goal_handle);
      clips::UDFNthArgument(udfc, 2, EXTERNAL_ADDRESS_BIT, &feedback);
       try {
        instance->server_goal_handle_publish_feedback(goal_handle.externalAddressValue->contents, feedback.externalAddressValue->contents, udfc);
       } catch (std::out_of_range &e) {
         RCLCPP_ERROR(instance->get_logger(), "Unknown goal handle or feedback %s", e.what());
       }
    },
    "server_goal_handle_publish_feedback", this);

  fun_name = "{{name_kebab}}-client-goal-handle-get-goal-id";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "m", 1, 1, ";e",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue *out) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue goal_handle;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &goal_handle);
       try {
        *out = instance->client_goal_handle_get_goal_id(env, goal_handle.externalAddressValue->contents, udfc);
       } catch (std::out_of_range &e) {
         RCLCPP_ERROR(instance->get_logger(), "Unknown goal handle %s", e.what());
       }
    },
    "client_goal_handle_get_goal_id", this);

  fun_name = "{{name_kebab}}-client-goal-handle-get-goal-stamp";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "d", 1, 1, ";e",
    [](clips::Environment *env, clips::UDFContext *udfc,
       clips::UDFValue *out) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue goal_handle;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &goal_handle);
       try {
        *out = instance->client_goal_handle_get_goal_stamp(env, goal_handle.externalAddressValue->contents, udfc);
       } catch (std::out_of_range &e) {
         RCLCPP_ERROR(instance->get_logger(), "Unknown goal handle %s", e.what());
       }
    },
    "client_goal_handle_get_goal_stamp", this);

  fun_name = "{{name_kebab}}-client-goal-handle-destroy";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "d", 1, 1, ";e",
    [](clips::Environment */*env*/, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue data_ptr;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &data_ptr);

      instance->client_goal_handle_destroy(data_ptr.externalAddressValue->contents);
    },
    "client_goal_handle_destroy", this);

  fun_name = "{{name_kebab}}-server-goal-handle-destroy";
  function_names_.insert(fun_name);
  clips::AddUDF(
    clips.get_obj().get(), fun_name.c_str(), "d", 1, 1, ";e",
    [](clips::Environment */*env*/, clips::UDFContext *udfc,
       clips::UDFValue * /*out*/) {
      auto *instance = static_cast<{{name_camel}} *>(udfc->context);
      clips::UDFValue data_ptr;
      using namespace clips;
      clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &data_ptr);

      instance->server_goal_handle_destroy(data_ptr.externalAddressValue->contents);
    },
    "server_goal_handle_destroy", this);

  // add fact templates
  clips::Build(clips.get_obj().get(),"(deftemplate {{name_kebab}}-client \
            (slot server (type STRING)))");
  clips::Build(clips.get_obj().get(),"(deftemplate {{name_kebab}}-server \
            (slot name (type STRING)))");
  clips::Build(clips.get_obj().get(),"(deftemplate {{name_kebab}}-goal-response \
            (slot server (type STRING) ) \
            (slot client-goal-handle-ptr (type EXTERNAL-ADDRESS)) \
            )");
  clips::Build(clips.get_obj().get(),"(deftemplate {{name_kebab}}-goal-feedback \
            (slot server (type STRING) ) \
            (slot client-goal-handle-ptr (type EXTERNAL-ADDRESS)) \
            (slot feedback-ptr (type EXTERNAL-ADDRESS)) \
            )");
  clips::Build(clips.get_obj().get(),"(deftemplate {{name_kebab}}-wrapped-result \
            (slot server (type STRING) ) \
            (slot uuid (type STRING)) \
            (slot code (type SYMBOL)) \
            (slot result-ptr (type EXTERNAL-ADDRESS)) \
            )");
  clips::Build(clips.get_obj().get(),"(deftemplate {{name_kebab}}-accepted-goal \
            (slot server (type STRING) ) \
            (slot server-goal-handle-ptr (type EXTERNAL-ADDRESS)) \
            )");
  return true;
}

{% set template_part = "definition" %}
{% set template_slots = goal_slots %}
{% set template_type = "Goal" %}
{% include 'set_field.jinja.cpp' with context %}
{% include 'get_field.jinja.cpp' with context %}
{% include 'create.jinja.cpp' with context %}
{% include 'destroy.jinja.cpp' with context %}

{% set template_slots = result_slots %}
{% set template_type = "Result" %}
{% include 'set_field.jinja.cpp' with context %}
{% include 'get_field.jinja.cpp' with context %}
{% include 'create.jinja.cpp' with context %}
{% include 'destroy.jinja.cpp' with context %}

{% set template_slots = feedback_slots %}
{% set template_type = "Feedback" %}
{% include 'set_field.jinja.cpp' with context %}
{% set template_fun_name = "get_field_feedback" %}
{% include 'get_field.jinja.cpp' with context %}
{% include 'create.jinja.cpp' with context %}

void {{name_camel}}::feedback_destroy({{message_type}}::Feedback *fb) {
  auto it = feedbacks_.find(fb);
  if (it != feedbacks_.end()) {
      feedbacks_.erase(it);
  }
  auto const_it = const_feedbacks_.find(fb);
  if (const_it != const_feedbacks_.end()) {
      const_feedbacks_.erase(const_it);
  }
}

void {{name_camel}}::send_goal(clips::Environment *env, {{message_type}}::Goal *goal, const std::string &server_name) {
  using namespace std::chrono_literals;
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

   // Handle the request asynchronously to not block clips engine potentially endlessly
  std::thread([this, goal, server_name, env_name]() {
  while (!clients_[env_name][server_name]->wait_for_action_server(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the server. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "server %s not available, waiting again...", server_name.c_str());
  }
  cx::LockSharedPtr<clips::Environment> &clips = envs_[env_name];
   auto send_goal_options = rclcpp_action::Client<{{message_type}}>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this, &clips, server_name](const std::shared_ptr<rclcpp_action::ClientGoalHandle<{{message_type}}>> &goal_handle) {
    task_queue_.push([this, &clips, server_name, goal_handle]() {
      std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));
      client_goal_handles_.try_emplace(goal_handle.get(), goal_handle);
      clips::FactBuilder *fact_builder = clips::CreateFactBuilder(clips.get_obj().get(), "{{name_kebab}}-goal-response");
      clips::FBPutSlotString(fact_builder,"server",server_name.c_str());
      clips::FBPutSlotCLIPSExternalAddress(fact_builder,"client-goal-handle-ptr", clips::CreateCExternalAddress(clips.get_obj().get(), goal_handle.get()));
      clips::FBAssert(fact_builder);
      clips::FBDispose(fact_builder);
    });
    cv_.notify_one();  // Notify the worker thread
  };

  send_goal_options.feedback_callback = [this, &clips, server_name](
      std::shared_ptr<rclcpp_action::ClientGoalHandle<{{message_type}}>> goal_handle,
      const std::shared_ptr<const {{message_type}}::Feedback> feedback) {
    // this indirection is necessary as the feedback callback is called while an internal lock is acquired.
    // The same lock is acquired in calls lige get_status().
    // This can cause a deadlock if get_status() is called from within clips right after a callback is received.
    // clips lock is still held, hence callback can't proceed, but internal lock is already ackquired when this callback is invoked.
    std::lock_guard<std::mutex> lock(queue_mutex_);
    task_queue_.push([this, &clips, server_name, goal_handle, feedback]() {
      // Enqueue the task to avoid directly locking the handle_mutex_ in a callback
    std::lock_guard<std::mutex> lock(queue_mutex_);
      std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));
      const_feedbacks_.try_emplace(const_cast<void *>(static_cast<const void*>(feedback.get())), feedback);
      client_goal_handles_.try_emplace(goal_handle.get(), goal_handle);
      clips::FactBuilder *fact_builder = clips::CreateFactBuilder(clips.get_obj().get(), "{{name_kebab}}-goal-feedback");
      clips::FBPutSlotString(fact_builder,"server",server_name.c_str());
      clips::FBPutSlotCLIPSExternalAddress(fact_builder,"client-goal-handle-ptr", clips::CreateCExternalAddress(clips.get_obj().get(), goal_handle.get()));
      clips::FBPutSlotCLIPSExternalAddress(fact_builder,"feedback-ptr", clips::CreateCExternalAddress(clips.get_obj().get(), const_cast<void *>(static_cast<const void *>(feedback.get()))));
      clips::FBAssert(fact_builder);
      clips::FBDispose(fact_builder);
    });
    cv_.notify_one();  // Notify the worker thread
  };

  send_goal_options.result_callback = [this, &clips, server_name](const rclcpp_action::ClientGoalHandle<{{message_type}}>::WrappedResult &wrapped_result) {
    task_queue_.push([this, &clips, server_name, wrapped_result]() {
      results_.try_emplace(wrapped_result.result.get(), wrapped_result.result);
      clips::FactBuilder *fact_builder = clips::CreateFactBuilder(clips.get_obj().get(), "{{name_kebab}}-wrapped-result");
      clips::FBPutSlotString(fact_builder,"server",server_name.c_str());
      clips::FBPutSlotString(fact_builder,"goal-id",rclcpp_action::to_string(wrapped_result.goal_id).c_str());
      std::string code_str = "UNKNOWN";
      switch(wrapped_result.code) {
        case rclcpp_action::ResultCode::UNKNOWN: break;
        case rclcpp_action::ResultCode::SUCCEEDED: code_str = "SUCCEEDED"; break;
        case rclcpp_action::ResultCode::CANCELED: code_str = "CANCELED"; break;
        case rclcpp_action::ResultCode::ABORTED: code_str = "ABORTED"; break;
      }
      results_.try_emplace(wrapped_result.result.get(), wrapped_result.result);
      clips::FBPutSlotSymbol(fact_builder,"code", code_str.c_str());
      clips::FBPutSlotCLIPSExternalAddress(fact_builder,"result-ptr", clips::CreateCExternalAddress(clips.get_obj().get(), wrapped_result.result.get()));
      clips::FBAssert(fact_builder);
      clips::FBDispose(fact_builder);
    });
    cv_.notify_one();  // Notify the worker thread
  };

  clients_[env_name][server_name]->async_send_goal(*goal, send_goal_options);
  }).detach();
}

void {{name_camel}}::create_new_server(clips::Environment *env, const std::string &server_name) {
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
  auto handle_goal = [this, env_name, server_name](const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const {{message_type}}::Goal> goal) {
    cx::LockSharedPtr<clips::Environment> &clips = envs_[env_name];
    std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));
    clips::Deffunction *dec_fun = clips::FindDeffunction(clips.get_obj().get(),"{{name_kebab}}-handle-goal-callback");
    if(!dec_fun) {
      RCLCPP_INFO(get_logger(), "Accepting goal per default");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    clips::FunctionCallBuilder *fcb = clips::CreateFunctionCallBuilder(clips.get_obj().get(),3);
    clips::FCBAppendString(fcb, server_name.c_str());
    clips::FCBAppendCLIPSExternalAddress(fcb, clips::CreateCExternalAddress(clips.get_obj().get(), const_cast<void*>(static_cast<const void *>(goal.get()))));
    clips::FCBAppendString(fcb, rclcpp_action::to_string(uuid).c_str());
    clips::CLIPSValue ret;
    // no need to delete goal pointer manually, it is not copied
    clips::FCBCall(fcb,"{{name_kebab}}-handle-goal-callback",&ret);
    clips::FCBDispose(fcb);
    return static_cast<rclcpp_action::GoalResponse>(ret.integerValue->contents);
  };
  auto handle_cancel = [this, env_name, server_name](const std::shared_ptr<rclcpp_action::ServerGoalHandle<{{message_type}}>> goal_handle) {

    cx::LockSharedPtr<clips::Environment> &clips = envs_[env_name];
    std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));
    clips::Deffunction *dec_fun = clips::FindDeffunction(clips.get_obj().get(),"{{name_kebab}}-cancel-goal-callback");
    if(!dec_fun) {
      RCLCPP_INFO(get_logger(), "Accepting goal cancellation per default");
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    // store the goal handle to ensure it is not cleaned up implicitly
    // server_goal_handles_.try_emplace(goal_handle.get(),goal_handle);

    clips::FunctionCallBuilder *fcb = clips::CreateFunctionCallBuilder(clips.get_obj().get(),3);
    clips::FCBAppendString(fcb, server_name.c_str());
    clips::FCBAppendCLIPSExternalAddress(fcb, clips::CreateCExternalAddress(clips.get_obj().get(), const_cast<void *>(static_cast<const void *>(goal_handle->get_goal().get()))));
    clips::FCBAppendCLIPSExternalAddress(fcb, clips::CreateCExternalAddress(clips.get_obj().get(), goal_handle.get()));
    clips::CLIPSValue ret;
    clips::FCBCall(fcb,"{{name_kebab}}-cancel-goal-callback",&ret);
    clips::FCBDispose(fcb);
    return static_cast<rclcpp_action::CancelResponse>(ret.integerValue->contents);
  };

  auto handle_accepted = [this, env_name, server_name](const std::shared_ptr<rclcpp_action::ServerGoalHandle<{{message_type}}>> goal_handle) {
    cx::LockSharedPtr<clips::Environment> &clips = envs_[env_name];
    std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));
    // store the goal handle to ensure it is not cleaned up implicitly
    server_goal_handles_.try_emplace(goal_handle.get(),goal_handle);
    clips::FactBuilder *fact_builder = clips::CreateFactBuilder(clips.get_obj().get(), "{{name_kebab}}-accepted-goal");
    clips::FBPutSlotString(fact_builder,"server",server_name.c_str());
    clips::FBPutSlotCLIPSExternalAddress(fact_builder,"server-goal-handle-ptr", clips::CreateCExternalAddress(clips.get_obj().get(), goal_handle.get()));
    clips::FBAssert(fact_builder);
    clips::FBDispose(fact_builder);
  };
  servers_[env_name][server_name] =
      rclcpp_action::create_server<{{message_type}}>(this, server_name, handle_goal, handle_cancel, handle_accepted);
}

void {{name_camel}}::destroy_server(clips::Environment *env, const std::string &server_name) {
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

  auto outer_it = servers_.find(env_name);
  if (outer_it != servers_.end()) {
      // Check if server_name exists in the inner map
      auto& inner_map = outer_it->second;
      auto inner_it = inner_map.find(server_name);
      if (inner_it != inner_map.end()) {
          // Remove the server_name entry from the inner map
          inner_map.erase(inner_it);
      } else {
          RCLCPP_WARN(this->get_logger(), "Service %s not found in environment %s", server_name.c_str(), env_name.c_str());
      }
  } else {
      RCLCPP_WARN(this->get_logger(), "Environment %s not found", env_name.c_str());
  }

  clips::Eval(env, ("(do-for-all-facts ((?f {{name_kebab}}-server)) (eq (str-cat ?f:name) (str-cat " + server_name + "))  (retract ?f))").c_str(), NULL);
}

void {{name_camel}}::create_new_client(clips::Environment *env,
    const std::string &server_name) {
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

  auto it = clients_[env_name].find(server_name);

  if (it != clients_[env_name].end()) {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
                "There already exists a client for server %s", server_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
                "Creating client for server %s", server_name.c_str());
    clients_[env_name][server_name] =
        rclcpp_action::create_client<{{message_type}}>(this,server_name);
    clips::AssertString(envs_[env_name].get_obj().get(), ("({{name_kebab}}-client (server \"" + server_name + "\"))").c_str());
  }
}

void {{name_camel}}::destroy_client(clips::Environment *env,
    const std::string &server_name) {
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

  auto it = clients_[env_name].find(server_name);

  if (it != clients_[env_name].end()) {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
                "Destroying client for server %s", server_name.c_str());
    clients_[env_name].erase(server_name);
  }

  clips::Eval(env, ("(do-for-all-facts ((?f {{name_kebab}}-client)) (eq (str-cat ?f:server) (str-cat " + server_name + "))  (retract ?f))").c_str(), NULL);
}


void {{name_camel}}::server_goal_handle_abort(void *goal_handle_raw, void *result_raw, clips::UDFContext *udfc) {
  auto goal_handle = server_goal_handles_.at(goal_handle_raw);
  if(!goal_handle) {
    RCLCPP_ERROR(get_logger(), "server-goal-handle-abort: Invalid pointer to ServerGoalHandle");
    clips::UDFThrowError(udfc);
    return;
  }
  auto result = results_.at(result_raw);
  if(!result) {
    RCLCPP_ERROR(get_logger(), "server-goal-handle-abort: Invalid pointer to Result");
    clips::UDFThrowError(udfc);
    return;
  }
  goal_handle->abort(result);
}

void {{name_camel}}::server_goal_handle_succeed(void *goal_handle_raw, void *result_raw, clips::UDFContext *udfc) {
  auto goal_handle = server_goal_handles_.at(goal_handle_raw);
  if(!goal_handle) {
    RCLCPP_ERROR(get_logger(), "server-goal-handle-succeed: Invalid pointer to ServerGoalHandle");
    clips::UDFThrowError(udfc);
    return;
  }
  auto result = results_.at(result_raw);
  if(!result) {
    RCLCPP_ERROR(get_logger(), "server-goal-handle-succeed: Invalid pointer to Result");
    clips::UDFThrowError(udfc);
    return;
  }
  goal_handle->succeed(result);
}

void {{name_camel}}::server_goal_handle_canceled(void *goal_handle_raw, void *result_raw, clips::UDFContext *udfc) {
  auto goal_handle = server_goal_handles_.at(goal_handle_raw);
  if(!goal_handle) {
    RCLCPP_ERROR(get_logger(), "server-goal-handle-canceled: Invalid pointer to ServerGoalHandle");
    clips::UDFThrowError(udfc);
    return;
  }
  auto result = results_.at(result_raw);
  if(!result) {
    RCLCPP_ERROR(get_logger(), "server-goal-handle-canceled: Invalid pointer to Result");
    clips::UDFThrowError(udfc);
    return;
  }
  goal_handle->canceled(result);
}

clips::UDFValue {{name_camel}}::server_goal_handle_get_goal(clips::Environment *env, void *goal_handle_raw, clips::UDFContext *udfc) {
  clips::UDFValue res;
  auto goal_handle = server_goal_handles_.at(goal_handle_raw);
  if(!goal_handle) {
    RCLCPP_ERROR(get_logger(), "server-goal-handle-get-goal: Invalid pointer to ServerGoalHandle");
    clips::UDFThrowError(udfc);
    return res;
  }
  const std::shared_ptr<const {{message_type}}::Goal> goal = goal_handle->get_goal();
  res.externalAddressValue = clips::CreateCExternalAddress(env, (void *) goal.get());
  return res;
}

clips::UDFValue {{name_camel}}::server_goal_handle_get_goal_id(clips::Environment *env, void *goal_handle_raw, clips::UDFContext *udfc) {
  clips::UDFValue res;
  auto goal_handle = server_goal_handles_.at(goal_handle_raw);
  if(!goal_handle) {
    RCLCPP_ERROR(get_logger(), "server-goal-handle-get-goal-id: Invalid pointer to ServerGoalHandle");
    clips::UDFThrowError(udfc);
    return res;
  }
  rclcpp_action::GoalUUID goal_id = goal_handle->get_goal_id();
  res.lexemeValue = clips::CreateString(env, rclcpp_action::to_string(goal_id).c_str());
  res.begin = 0;
  res.range = -1;
  return res;
}

{% set template_ret_type = "Bool" %}
{% set template_type = "ServerGoalHandle" %}
{% set template_call_fun = "is_canceling" %}
{% include 'ret_fun.jinja.cpp' with context %}
{% set template_call_fun = "IsActive" %}
{% include 'ret_fun.jinja.cpp' with context %}
{% set template_call_fun = "IsExecuting" %}
{% include 'ret_fun.jinja.cpp' with context %}
{% set template_call_fun = "Execute" %}
{% include 'void_fun.jinja.cpp' with context %}

{% set template_type = "ClientGoalHandle" %}
{% set template_call_fun = "IsFeedbackAware" %}
{% include 'ret_fun.jinja.cpp' with context %}
{% set template_call_fun = "IsResultAware" %}
{% include 'ret_fun.jinja.cpp' with context %}
{% set template_call_fun = "GetStatus" %}
{% set template_ret_type = "Integer" %}
{% include 'ret_fun.jinja.cpp' with context %}

void {{name_camel}}::server_goal_handle_publish_feedback(void *goal_handle_raw, void *feedback_raw, clips::UDFContext *udfc) {
  auto goal_handle = server_goal_handles_.at(goal_handle_raw);
  if(!goal_handle) {
    RCLCPP_ERROR(get_logger(), "server-goal-handle-publish-feedback: Invalid pointer to ServerGoalHandle");
    clips::UDFThrowError(udfc);
    return;
  }
  auto it = const_feedbacks_.find(feedback_raw);
  if(it != const_feedbacks_.end()) {
    RCLCPP_ERROR(get_logger(), "server-goal-handle-publish-feedback: Cannot publish const feedback");
    clips::UDFThrowError(udfc);
    return;
  }
  auto feedback = feedbacks_.at(feedback_raw);
  if(!feedback) {
    RCLCPP_ERROR(get_logger(), "server-goal-handle-publish-feedback: Invalid pointer to Feedback");
    clips::UDFThrowError(udfc);
    return;
  }
  goal_handle->publish_feedback(feedbacks_.at(feedback_raw));
}

clips::UDFValue {{name_camel}}::client_goal_handle_get_goal_id(clips::Environment *env, void *goal_handle_raw, clips::UDFContext *udfc) {
  auto goal_handle = client_goal_handles_.at(goal_handle_raw);
  clips::UDFValue res;
  std::string uuid_str;
  if(goal_handle) {
    rclcpp_action::GoalUUID goal_id = goal_handle->get_goal_id();
	uuid_str = rclcpp_action::to_string(goal_id);
  } else {
    RCLCPP_ERROR(get_logger(), "client-goal-handle-get-goal-id: Invalid pointer to ClientGoalHandle");
    clips::UDFThrowError(udfc);
    uuid_str = "";
  }
  res.lexemeValue = clips::CreateString(env, uuid_str.c_str());
  return res;
}

clips::UDFValue {{name_camel}}::client_goal_handle_get_goal_stamp(clips::Environment *env, void *goal_handle_raw, clips::UDFContext *udfc) {
  auto goal_handle = client_goal_handles_.at(goal_handle_raw);
  clips::UDFValue res;
  if(goal_handle) {
  res.floatValue = clips::CreateFloat(env, goal_handle->get_goal_stamp().seconds());
  } else {
     RCLCPP_ERROR(get_logger(), "client-goal-handle-get-goal-id: Invalid pointer to ClientGoalHandle");
     res.floatValue = clips::CreateFloat(env, 0.0);
     clips::UDFThrowError(udfc);
  }
  return res;
}

void {{name_camel}}::client_goal_handle_destroy(void *g) {
  auto it = client_goal_handles_.find(g);
  if (it != client_goal_handles_.end()) {
      client_goal_handles_.erase(it);
  }
}

void {{name_camel}}::server_goal_handle_destroy(void *g) {
  auto it = server_goal_handles_.find(g);
  if (it != server_goal_handles_.end()) {
      server_goal_handles_.erase(it);
  }
}


} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::{{name_camel}}, cx::ClipsFeature)
