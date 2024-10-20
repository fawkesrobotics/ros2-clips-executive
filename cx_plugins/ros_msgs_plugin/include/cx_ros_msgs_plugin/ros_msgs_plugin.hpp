// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#ifndef CX_PLUGINS__ROS_MSGS_PLUGIN_HPP_
#define CX_PLUGINS__ROS_MSGS_PLUGIN_HPP_

#include <string>

#include "cx_plugin/clips_plugin.hpp"
#include "cx_utils/LockSharedPtr.hpp"
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

namespace cx {

class RosMsgsPlugin : public ClipsPlugin {
public:
  RosMsgsPlugin();
  ~RosMsgsPlugin();

  void initialize();
  void finalize();

  bool clips_env_init(LockSharedPtr<clips::Environment> &env) override;
  bool clips_env_destroyed(LockSharedPtr<clips::Environment> &env) override;

private:
  clips::UDFValue create_message(clips::Environment *env,
                                 const std::string &type);
  void destroy_msg(void *msg);

  clips::UDFValue ros_message_member_to_udf_value(
      clips::Environment *env, void *deserialized_msg,
      const rosidl_typesupport_introspection_cpp::MessageMember &member);

  clips::CLIPSValue ros_to_clips_value(clips::Environment *env, void *val,
                                       uint8_t ros_type);

  clips::UDFValue get_field(clips::Environment *env, void *deserialized_msg,
                            const std::string &field, clips::UDFContext *udfc);

  void udf_value_to_ros_message_member(
      clips::Environment *env, void *deserialized_msg,
      const rosidl_typesupport_introspection_cpp::MessageMember &member,
      clips::UDFValue &val);

  void clips_to_ros_value(
      clips::Environment *env, const clips::CLIPSValue &val,
      const rosidl_typesupport_introspection_cpp::MessageMember &member,
      uint8_t *field_ptr);

  void set_field(clips::Environment *env, void *deserialized_msg,
                 const std::string &field, clips::UDFValue &val,
                 clips::UDFContext *udfc);

  //// Helper function to manage subscriptions
  void subscribe_to_topic(clips::Environment *env,
                          const std::string &topic_name,
                          const std::string &topic_type);

  std::shared_ptr<void> create_deserialized_msg(const std::string &topic_type);

  void topic_callback(std::shared_ptr<const rclcpp::SerializedMessage> msg,
                      const std::string &topic_name,
                      const std::string &topic_type, clips::Environment *env);
  void unsubscribe_from_topic(clips::Environment *env,
                              const std::string &topic_name);

  void create_new_publisher(clips::Environment *env,
                            const std::string &topic_name,
                            const std::string &topic_type);
  void destroy_publisher(clips::Environment *env,
                         const std::string &topic_name);
  void publish_to_topic(clips::Environment *env, void *deserialized_msg,
                        const std::string &topic_name);

  std::unique_ptr<rclcpp::Logger> logger_;

  rclcpp::CallbackGroup::SharedPtr cb_group_;

  std::unordered_set<std::string> function_names_;

  // env -> (topic_name -> subscription)
  std::map<std::string,
           std::map<std::string, std::shared_ptr<rclcpp::GenericSubscription>>>
      subscriptions_;

  // env -> (topic_name -> publisher)
  std::map<std::string,
           std::map<std::string, std::shared_ptr<rclcpp::GenericPublisher>>>
      publishers_;

  std::unordered_map<std::string, std::shared_ptr<rcpputils::SharedLibrary>>
      libs_;

  // void * of deserialized_msg -> <deserialized_msg, message_type>
  std::unordered_map<void *, std::pair<std::shared_ptr<void>, std::string>>
      messages_;
  // message_type -> message_info
  std::unordered_map<std::string, const rosidl_message_type_support_t *>
      type_support_cache_;
  std::unordered_map<
      std::string, const rosidl_typesupport_introspection_cpp::MessageMembers *>
      introspection_info_cache_;
};
} // namespace cx

#endif // !CX_PLUGINS__ROS_MSGS_PLUGIN_HPP_
