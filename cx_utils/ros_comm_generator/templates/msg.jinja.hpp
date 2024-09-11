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
  std::map<std::string, LockSharedPtr<clips::Environment>> envs_;
  std::thread spin_thread_;
  std::map<std::string,
           std::map<std::string,
                    rclcpp::Publisher<{{message_type}}>::SharedPtr>>
      publishers_;
  std::map<std::string,
           std::map<std::string,
                    rclcpp::Subscription<{{message_type}}>::SharedPtr>>
      subscriptions_;
  std::unordered_set<{{message_type}}*> messages_;

clips::UDFValue create_message(clips::Environment *env);

clips::UDFValue get_field(clips::Environment *env,
                          {{message_type}} *msg,
                          const std::string &field);

  void set_field_publish({{message_type }} *msg, const std::string &field, clips::UDFValue value, clips::UDFContext *udfc);

  void publish_to_topic(clips::Environment *env, {{message_type}} *msg, const std::string &topic_name);

  void create_new_publisher(clips::Environment *env, const std::string &topic_name);

  void destroy_publisher(clips::Environment *env, const std::string &topic_name);

  void destroy_msg({{message_type}} *msg);

  void subscribe_to_topic(clips::Environment *env, const std::string &topic_name);

  void unsubscribe_from_topic(clips::Environment *env, const std::string &topic_name);

  void topic_callback(const {{message_type}}::SharedPtr msg,
                      std::string topic_name, std::string env_name);
};

} // namespace cx
#endif // !CX_FEATURES__{{name_upper}}_HPP_
