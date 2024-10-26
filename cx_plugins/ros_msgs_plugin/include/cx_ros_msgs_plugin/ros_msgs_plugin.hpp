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
  struct MessageInfo {
    void *msg_ptr;
    const rosidl_typesupport_introspection_cpp::MessageMembers *members;
    bool is_sub_msg;

    MessageInfo(
        const rosidl_typesupport_introspection_cpp::MessageMembers *members,
        void *parent_ptr = nullptr)
        : msg_ptr(nullptr), members(members) {
      is_sub_msg = (parent_ptr != nullptr);
      if (parent_ptr) {
        msg_ptr = parent_ptr;
      } else {
        // Allocate memory for the message
        msg_ptr = malloc(members->size_of_);
        if (members->init_function) {
          members->init_function(
              msg_ptr,
              rosidl_runtime_cpp::MessageInitialization::ALL); // Full
                                                               // initialization
        }
      }
    }

    ~MessageInfo() {
      if (!is_sub_msg) {
        // Finalize the message and free memory
        if (members && members->fini_function) {
          members->fini_function(msg_ptr);
        }
        free(msg_ptr);
      }
    }

    // Disable copy constructor and copy assignment to prevent double freeing
    MessageInfo(const MessageInfo &) = delete;
    MessageInfo &operator=(const MessageInfo &) = delete;

    // Enable move constructor and move assignment for efficiency
    MessageInfo(MessageInfo &&other) noexcept
        : msg_ptr(other.msg_ptr), members(other.members) {
      RCLCPP_WARN(rclcpp::get_logger("MESSAGE INFO"), "ASSIGN");
      other.msg_ptr = nullptr;
    }
  };
  struct MessageInfoHasher {
    std::size_t operator()(const MessageInfo &key) const {
      return std::hash<void *>()(key.msg_ptr) ^
             std::hash<const rosidl_typesupport_introspection_cpp::
                           MessageMembers *>()(key.members);
    }
  };

  clips::UDFValue create_message(clips::Environment *env,
                                 const std::string &type);
  void destroy_msg(void *msg);

  clips::UDFValue ros_msg_member_to_udf_value(
      clips::Environment *env, std::shared_ptr<MessageInfo> &msg_info,
      const rosidl_typesupport_introspection_cpp::MessageMember &member);
  std::shared_ptr<MessageInfo> process_nested_msg(
      void *nested_msg,
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

  std::shared_ptr<RosMsgsPlugin::MessageInfo>
  create_deserialized_msg(const std::string &topic_type);

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

  std::string get_msg_type(
      const rosidl_typesupport_introspection_cpp::MessageMembers *members);
  const rosidl_typesupport_introspection_cpp::MessageMembers *
  get_msg_members(const std::string &members);

  std::shared_ptr<MessageInfo>
  deserialize_msg(std::shared_ptr<const rclcpp::SerializedMessage> msg,
                  const std::string &msg_type);

  rclcpp::SerializedMessage serialize_msg(std::shared_ptr<MessageInfo> msg_info,
                                          const std::string &msg_type);

  void move_field_to_parent(
      void *parent_msg,
      const rosidl_typesupport_introspection_cpp::MessageMember *parent_member,
      void *source_msg);

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

  // MessageInfo* -> shared_ptr holding the MessageInfo*
  std::unordered_map<void *, std::shared_ptr<MessageInfo>> messages_;
  // parent msg MessageInfo* -> nested msg MessageInfo*
  std::unordered_map<void *, std::vector<void *>> sub_messages_;
  // message_type -> message_info
  std::unordered_map<std::string, const rosidl_message_type_support_t *>
      type_support_cache_;
};
} // namespace cx

#endif // !CX_PLUGINS__ROS_MSGS_PLUGIN_HPP_