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
#ifndef CX_PLUGINS__{{name_upper}}_HPP_
#define CX_PLUGINS__{{name_upper}}_HPP_

#include <map>
#include <memory>
#include <string>

#include "cx_plugin/clips_plugin.hpp"
#include "cx_utils/LockSharedPtr.hpp"
#include "cx_utils/NodeThread.hpp"

#include "rclcpp/rclcpp.hpp"
#include {{message_include_path}}

namespace cx {

class {{name_camel}} : public ClipsPlugin, public rclcpp::Node {
public:
  {{name_camel}}();
  ~{{name_camel}}();

  void initialize() override;
  void finalize() override;

  bool clips_env_init(LockSharedPtr<clips::Environment> &env) override;
  bool clips_env_destroyed(LockSharedPtr<clips::Environment> &env) override;

private:
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  std::unique_ptr<rclcpp::Logger> logger_;

  std::mutex map_mtx_;

  bool stop_flag_;

  std::map<std::string,
           std::map<std::string,
                    rclcpp::Service<{{message_type}}>::SharedPtr>>
      services_;
  std::map<std::string,
           std::map<std::string,
                    rclcpp::Client<{{message_type}}>::SharedPtr>>
      clients_;
  std::unordered_map<void*, std::shared_ptr<{{message_type}}::Request>> requests_;
  std::unordered_map<void*, std::shared_ptr<{{message_type}}::Response>> responses_;

  std::unordered_set<std::string> function_names_;

{% set template_part = "declaration" %}
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

  void send_request(clips::Environment *env, {{message_type}}::Request *msg, const std::string &service_name);

  void create_new_client(clips::Environment *env, const std::string &service_name);

  void destroy_client(clips::Environment *env, const std::string &service_name);

  void create_new_service(clips::Environment *env, const std::string &service_name);

  void destroy_service(clips::Environment *env, const std::string &service_name);

  void service_callback(const std::shared_ptr<{{message_type}}::Request> request,
                        std::shared_ptr<{{message_type}}::Response> response,
                        std::string service_name, clips::Environment *env);
};

} // namespace cx
#endif // !CX_PLUGINS__{{name_upper}}_HPP_
