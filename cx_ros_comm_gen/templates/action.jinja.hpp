// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  {{name_camel}}.hpp
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
#ifndef CX_FEATURES__{{name_upper}}_HPP_
#define CX_FEATURES__{{name_upper}}_HPP_

#include <map>
#include <memory>
#include <string>

#include "cx_core/ClipsFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"
#include "cx_utils/NodeThread.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include {{message_include_path}}

namespace cx {

class {{name_camel}} : public ClipsFeature, public rclcpp::Node {
public:
  {{name_camel}}();
  ~{{name_camel}}();

  void initialise(const std::string &feature_name) override;

  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<clips::Environment> &clips) override;
  bool clips_context_destroyed(const std::string &env_name) override;

  std::string getFeatureName() const;

private:
  std::queue<std::function<void()>> task_queue_;
  std::mutex queue_mutex_;
  std::condition_variable cv_;
  std::thread clips_worker_thread_;
  bool stop_flag_ = false;

  std::map<std::string, LockSharedPtr<clips::Environment>> envs_;
  std::thread spin_thread_;
  std::map<std::string,
           std::map<std::string,
                    rclcpp_action::Server<{{message_type}}>::SharedPtr>>
      servers_;
  std::map<std::string,
           std::map<std::string,
                    rclcpp_action::Client<{{message_type}}>::SharedPtr>>
      clients_;
  std::unordered_map<void* , std::shared_ptr<rclcpp_action::ServerGoalHandle<{{message_type}}>>> server_goal_handles_;
  std::unordered_map<void* , std::shared_ptr<rclcpp_action::ClientGoalHandle<{{message_type}}>>> client_goal_handles_;
  std::unordered_map<void *,{{message_type}}::Goal::SharedPtr> goals_;
  std::unordered_map<void*, std::shared_ptr<{{message_type}}::Result>> results_;
  std::unordered_map<void*, std::shared_ptr<{{message_type}}::Feedback>> feedbacks_;
  std::unordered_map<void*, std::shared_ptr<const {{message_type}}::Feedback>> const_feedbacks_;

  std::unordered_set<std::string> function_names_;

{% set template_part = "declaration" %}
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

  void send_goal(clips::Environment *env, {{message_type}}::Goal *msg, const std::string &server_name);

  void create_new_client(clips::Environment *env, const std::string &server_name);

  void destroy_client(clips::Environment *env, const std::string &server_name);

  void create_new_server(clips::Environment *env, const std::string &server_name);

  void destroy_server(clips::Environment *env, const std::string &server_name);

  void client_goal_handle_destroy(void *handle_ptr);

  void server_goal_handle_destroy(void *handle_ptr);

  void server_goal_handle_abort(void *goal_handle_raw, void *result_raw, clips::UDFContext *udfc);
  void server_goal_handle_succeed(void *goal_handle_raw, void *result_raw, clips::UDFContext *udfc);
  void server_goal_handle_canceled(void *goal_handle_raw, void *result_raw, clips::UDFContext *udfc);
  clips::UDFValue server_goal_handle_get_goal(clips::Environment *env, void *goal_handle_raw, clips::UDFContext *udfc);
  clips::UDFValue server_goal_handle_get_goal_id(clips::Environment *env, void *goal_handle_raw, clips::UDFContext *udfc);
  void server_goal_handle_publish_feedback(void *goal_handle_raw, void *feedback_raw, clips::UDFContext *udfc);
  clips::UDFValue client_goal_handle_get_goal_id(clips::Environment *env, void *goal_handle_raw, clips::UDFContext *udfc);
  clips::UDFValue client_goal_handle_get_goal_stamp(clips::Environment *env, void *goal_handle_raw, clips::UDFContext *udfc);
};

} // namespace cx
#endif // !CX_FEATURES__{{name_upper}}_HPP_
