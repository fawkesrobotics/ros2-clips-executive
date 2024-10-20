// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include <string>

#include "cx_ros_msgs_plugin/ros_msgs_plugin.hpp"
#include <cx_utils/clips_env_context.hpp>

#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <rcutils/types/uint8_array.h>

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;
namespace cx {
RosMsgsPlugin::RosMsgsPlugin() {}

RosMsgsPlugin::~RosMsgsPlugin() {}

void RosMsgsPlugin::initialize() {
  logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger(plugin_name_));
  auto node = parent_.lock();
  cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
}

void RosMsgsPlugin::finalize() {
  if (subscriptions_.size() > 0) {
    for (const auto &sub_map : subscriptions_) {
      for (const auto &sub : sub_map.second) {
        RCLCPP_WARN(*logger_,
                    "Environment %s has open %s subscription, cleaning up ...",
                    sub_map.first.c_str(), sub.first.c_str());
      }
    }
    subscriptions_.clear();
  }
  if (publishers_.size() > 0) {
    for (const auto &pub_map : publishers_) {
      for (const auto &pub : pub_map.second) {
        RCLCPP_WARN(*logger_,
                    "Environment %s has open %s publishers, cleaning up ...",
                    pub_map.first.c_str(), pub.first.c_str());
      }
    }
    publishers_.clear();
  }
  cb_group_.reset();
  if (messages_.size() > 0) {
    RCLCPP_WARN(*logger_, "Found %li message(s), cleaning up ...",
                messages_.size());
    messages_.clear();
  }
}

bool RosMsgsPlugin::clips_env_init(LockSharedPtr<clips::Environment> &env) {
  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  RCLCPP_INFO(*logger_, "Initializing plugin for environment %s",
              context->env_name_.c_str());

  std::string fun_name = "ros-msgs-get-field";
  function_names_.insert(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "*", 2, 2, ";e;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        auto *instance = static_cast<RosMsgsPlugin *>(udfc->context);
        clips::UDFValue data_ptr, field;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &data_ptr);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field);
        *out = instance->get_field(env, data_ptr.externalAddressValue->contents,
                                   field.lexemeValue->contents, udfc);
      },
      "get_field", this);

  fun_name = "ros-msgs-set-field";
  function_names_.insert(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "v", 3, 3, ";e;sy;*",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<RosMsgsPlugin *>(udfc->context);
        clips::UDFValue msg_ptr, field, value;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &msg_ptr);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field);
        clips::UDFNthArgument(udfc, 3, ANY_TYPE_BITS, &value);

        instance->set_field(env, msg_ptr.externalAddressValue->contents,
                            field.lexemeValue->contents, value, udfc);
      },
      "set_field", this);

  fun_name = "ros-msgs-create-msg";
  function_names_.insert(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "e", 1, 1, ";sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        auto *instance = static_cast<RosMsgsPlugin *>(udfc->context);
        clips::UDFValue type;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &type);
        *out = instance->create_message(env, type.lexemeValue->contents);
      },
      "create_message", this);

  fun_name = "ros-msgs-create-publisher";
  function_names_.insert(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "v", 2, 2, ";sy;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<RosMsgsPlugin *>(udfc->context);
        clips::UDFValue topic;
        clips::UDFValue type;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &topic);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &type);

        instance->create_new_publisher(env, topic.lexemeValue->contents,
                                       type.lexemeValue->contents);
      },
      "create_new_publisher", this);

  fun_name = "ros-msgs-destroy-publisher";
  function_names_.insert(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<RosMsgsPlugin *>(udfc->context);
        clips::UDFValue topic;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &topic);

        instance->destroy_publisher(env, topic.lexemeValue->contents);
      },
      "destroy_publisher", this);

  fun_name = "ros-msgs-publish";
  function_names_.insert(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "v", 2, 2, ";e;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<RosMsgsPlugin *>(udfc->context);
        clips::UDFValue message, topic;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &message);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &topic);

        instance->publish_to_topic(env, message.externalAddressValue->contents,
                                   topic.lexemeValue->contents);
      },
      "publish_to_topic", this);

  fun_name = "ros-msgs-destroy";
  function_names_.insert(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";e",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<RosMsgsPlugin *>(udfc->context);
        clips::UDFValue message;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, EXTERNAL_ADDRESS_BIT, &message);

        instance->destroy_msg(message.externalAddressValue->contents);
      },
      "destroy_msg", this);

  fun_name = "ros-msgs-create-subscription";
  function_names_.insert(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "v", 2, 2, ";sy;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<RosMsgsPlugin *>(udfc->context);
        clips::UDFValue topic;
        clips::UDFValue type;

        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &topic);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &type);
        instance->subscribe_to_topic(env, topic.lexemeValue->contents,
                                     type.lexemeValue->contents);
      },
      "subscribe_to_topic", this);

  fun_name = "ros-msgs-destroy-subscription";
  function_names_.insert(fun_name);
  clips::AddUDF(
      env.get_obj().get(), fun_name.c_str(), "v", 1, 1, ";s",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<RosMsgsPlugin *>(udfc->context);
        clips::UDFValue topic;

        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &topic);
        instance->unsubscribe_from_topic(env, topic.lexemeValue->contents);
      },
      "unsubscribe_from_topic", this);

  // add fact templates
  clips::Build(env.get_obj().get(), "(deftemplate ros-msgs-subscription \
            (slot topic (type STRING)) \
            (slot type (type STRING)))");
  clips::Build(env.get_obj().get(), "(deftemplate ros-msgs-publisher \
            (slot topic (type STRING)) \
            (slot type (type STRING)))");
  clips::Build(env.get_obj().get(), "(deftemplate ros-msgs-message \
            (slot topic (type STRING) ) \
            (slot msg-ptr (type EXTERNAL-ADDRESS)) \
            )");

  return true;
}

bool RosMsgsPlugin::clips_env_destroyed(
    LockSharedPtr<clips::Environment> &env) {
  for (const auto &fun : function_names_) {
    clips::RemoveUDF(env.get_obj().get(), fun.c_str());
  }
  clips::Deftemplate *curr_tmpl =
      clips::FindDeftemplate(env.get_obj().get(), "ros-msgs-subscription");
  if (curr_tmpl) {
    clips::Undeftemplate(curr_tmpl, env.get_obj().get());
  } else {
    RCLCPP_WARN(*logger_, "ros-msgs-subscription cant be undefined");
  }
  curr_tmpl = clips::FindDeftemplate(env.get_obj().get(), "ros-msgs-publisher");
  if (curr_tmpl) {
    clips::Undeftemplate(curr_tmpl, env.get_obj().get());
  } else {
    RCLCPP_WARN(*logger_, "ros-msgs-publisher cant be undefined");
  }
  curr_tmpl = clips::FindDeftemplate(env.get_obj().get(), "ros-msgs-message");
  if (curr_tmpl) {
    clips::Undeftemplate(curr_tmpl, env.get_obj().get());
  } else {
    RCLCPP_WARN(*logger_, "ros-msgs-message cant be undefined");
  }
  return true;
}

void RosMsgsPlugin::subscribe_to_topic(clips::Environment *env,
                                       const std::string &topic_name,
                                       const std::string &topic_type) {
  RCLCPP_INFO(*logger_, "Subscribing to topic %s %s", topic_name.c_str(),
              topic_type.c_str());
  auto context = CLIPSEnvContext::get_context(env);
  std::string env_name = context->env_name_;
  auto node = parent_.lock();
  if (!node) {
    RCLCPP_ERROR(*logger_, "Invalid reference to parent node");
  }

  auto it = subscriptions_[env_name].find(topic_name);

  if (it != subscriptions_[env_name].end()) {
    RCLCPP_WARN(*logger_, "Already subscribed to topic %s", topic_name.c_str());
  } else {
    RCLCPP_DEBUG(*logger_, "Creating subscription to topic %s",
                 topic_name.c_str());
    auto options = rclcpp::SubscriptionOptions();
    options.callback_group = cb_group_;
    if (!libs_.contains(topic_type)) {
      libs_[topic_type] =
          rclcpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");
    }
    if (!type_support_cache_.contains(topic_type)) {
      type_support_cache_[topic_type] = rclcpp::get_message_typesupport_handle(
          topic_type, "rosidl_typesupport_cpp", *libs_[topic_type]);
    }
    if (!introspection_info_cache_.contains(topic_type)) {
      auto *introspection_type_support = get_message_typesupport_handle(
          type_support_cache_[topic_type],
          rosidl_typesupport_introspection_cpp::typesupport_identifier);
      introspection_info_cache_[topic_type] = static_cast<
          const rosidl_typesupport_introspection_cpp::MessageMembers *>(
          introspection_type_support->data);
    }

    subscriptions_[env_name][topic_name] = node->create_generic_subscription(
        topic_name, topic_type, rclcpp::QoS(10),
        [this, env, topic_name,
         topic_type](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
          topic_callback(msg, topic_name, topic_type, env);
        },
        options);

    clips::AssertString(env, ("(ros-msgs-subscription (topic \"" + topic_name +
                              "\") (type \"" + topic_type + "\"))")
                                 .c_str());
  }
}

void RosMsgsPlugin::unsubscribe_from_topic(clips::Environment *env,
                                           const std::string &topic_name) {
  auto context = CLIPSEnvContext::get_context(env);
  std::string env_name = context->env_name_;

  auto it = subscriptions_[env_name].find(topic_name);

  if (it != subscriptions_[env_name].end()) {
    RCLCPP_DEBUG(rclcpp::get_logger(plugin_name_),
                 "Unsubscribing from topic %s", topic_name.c_str());
    subscriptions_[env_name].erase(topic_name);
  } else {
    RCLCPP_WARN(rclcpp::get_logger(plugin_name_),
                "No subscription of %s found to destroy, ignoring",
                topic_name.c_str());
  }

  clips::Eval(env,
              ("(do-for-all-facts ((?f ros-msgs-subscription)) (eq (str-cat "
               "?f:topic) (str-cat \"" +
               topic_name + "\"))  (retract ?f))")
                  .c_str(),
              NULL);
}

std::shared_ptr<void>
RosMsgsPlugin::create_deserialized_msg(const std::string &topic_type) {
  auto *introspection_info = introspection_info_cache_[topic_type];

  // Allocate memory for the message
  std::shared_ptr<void> default_msg = std::shared_ptr<void>(
      malloc(introspection_info->size_of_), // Allocate memory for the message
      [this, introspection_info](
          void *ptr) { // Custom deleter to finalize and free memory
        if (introspection_info->fini_function) {
          introspection_info->fini_function(
              ptr); // Call the fini function to clean up
        }
        free(ptr); // Free the allocated memory
      });

  // Initialize the message with default values
  if (introspection_info->init_function) {
    introspection_info->init_function(
        default_msg.get(),
        rosidl_runtime_cpp::MessageInitialization::ALL // Use the ALL flag for
                                                       // full initialization
    );
  }
  return default_msg;
}

void RosMsgsPlugin::topic_callback(
    std::shared_ptr<const rclcpp::SerializedMessage> msg,
    const std::string &topic_name, const std::string &topic_type,
    clips::Environment *env) {
  std::shared_ptr<void> deserialized_msg = create_deserialized_msg(topic_type);

  // Deserialize the message
  rcutils_uint8_array_t serialized_data;
  serialized_data.buffer =
      const_cast<uint8_t *>(msg->get_rcl_serialized_message().buffer);
  serialized_data.buffer_length =
      msg->get_rcl_serialized_message().buffer_length;
  serialized_data.buffer_capacity =
      msg->get_rcl_serialized_message().buffer_capacity;

  // Perform the deserialization
  rmw_ret_t result =
      rmw_deserialize(&serialized_data, type_support_cache_[topic_type],
                      deserialized_msg.get());

  if (result != RMW_RET_OK) {
    RCLCPP_ERROR(*logger_,
                 "ros-msgs: failed to deserialize message on callback");
    return;
  }
  auto context = CLIPSEnvContext::get_context(env);
  cx::LockSharedPtr<clips::Environment> &clips = context->env_lock_ptr_;
  std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));
  messages_[deserialized_msg.get()] =
      std::make_pair(deserialized_msg, topic_type);

  // assert the newest message
  clips::FactBuilder *fact_builder =
      clips::CreateFactBuilder(clips.get_obj().get(), "ros-msgs-message");
  clips::FBPutSlotString(fact_builder, "topic", topic_name.c_str());
  clips::FBPutSlotString(fact_builder, "type", topic_type.c_str());
  clips::FBPutSlotCLIPSExternalAddress(
      fact_builder, "msg-ptr",
      clips::CreateCExternalAddress(clips.get_obj().get(),
                                    deserialized_msg.get()));
  clips::FBAssert(fact_builder);
  clips::FBDispose(fact_builder);
}

void RosMsgsPlugin::create_new_publisher(clips::Environment *env,
                                         const std::string &topic_name,
                                         const std::string &topic_type) {
  auto context = CLIPSEnvContext::get_context(env);
  std::string env_name = context->env_name_;

  auto node = parent_.lock();
  if (!node) {
    RCLCPP_ERROR(*logger_, "Invalid reference to parent node");
  }

  auto it = publishers_[env_name].find(topic_name);

  if (it != publishers_[env_name].end()) {
    RCLCPP_WARN(*logger_, "Already publishing to topic %s", topic_name.c_str());
  } else {
    RCLCPP_DEBUG(*logger_, "Creating publisher to topic %s",
                 topic_name.c_str());
    auto options = rclcpp::PublisherOptions();
    options.callback_group = cb_group_;

    publishers_[context->env_name_][topic_name] =
        node->create_generic_publisher(topic_name, topic_type, rclcpp::QoS(10),
                                       options);
    clips::AssertString(env, ("(ros-msgs-publisher (topic \"" + topic_name +
                              "\") (type \"" + topic_type + "\"))")
                                 .c_str());
  }
}

void RosMsgsPlugin::destroy_publisher(clips::Environment *env,
                                      const std::string &topic_name) {
  auto context = CLIPSEnvContext::get_context(env);
  std::string env_name = context->env_name_;
  auto outer_it = publishers_.find(env_name);
  if (outer_it != publishers_.end()) {
    // Check if topic_name exists in the inner map
    auto &inner_map = outer_it->second;
    auto inner_it = inner_map.find(topic_name);
    if (inner_it != inner_map.end()) {
      // Remove the topic_name entry from the inner map
      RCLCPP_DEBUG(*logger_, "Destroying publisher for topic %s",
                   topic_name.c_str());
      inner_map.erase(inner_it);
    } else {
      RCLCPP_WARN(*logger_, "Publisher %s not found in environment %s",
                  topic_name.c_str(), env_name.c_str());
    }
  } else {
    RCLCPP_WARN(*logger_, "Environment %s not found", env_name.c_str());
  }
}

void RosMsgsPlugin::publish_to_topic(clips::Environment *env,
                                     void *deserialized_msg,
                                     const std::string &topic_name) {
  auto context = CLIPSEnvContext::get_context(env);

  if (!messages_.contains(deserialized_msg)) {
  }
  std::string message_type = messages_[deserialized_msg].second;

  rmw_serialized_message_t serialized_msg =
      rmw_get_zero_initialized_serialized_message();

  const size_t initial_capacity =
      introspection_info_cache_[message_type]->size_of_;
  ;
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rmw_serialized_message_init(&serialized_msg, initial_capacity, &allocator);

  // Use the RMW API to serialize the message
  rmw_ret_t ret = rmw_serialize(
      deserialized_msg, type_support_cache_[message_type], &serialized_msg);

  rclcpp::SerializedMessage ros2_serialized_msg;
  ros2_serialized_msg.reserve(serialized_msg.buffer_capacity);
  ros2_serialized_msg.get_rcl_serialized_message().buffer =
      serialized_msg.buffer;
  ros2_serialized_msg.get_rcl_serialized_message().buffer_capacity =
      serialized_msg.buffer_capacity;
  ros2_serialized_msg.get_rcl_serialized_message().buffer_length =
      serialized_msg.buffer_length;

  if (ret != RMW_RET_OK) {
    // Handle serialization failure
    RCLCPP_ERROR(*logger_, "Failed to serialize message: %s\n",
                 rmw_get_error_string().str);
    return;
  }
  publishers_[context->env_name_][topic_name]->publish(ros2_serialized_msg);
}

clips::UDFValue RosMsgsPlugin::create_message(clips::Environment *env,
                                              const std::string &type) {
  std::shared_ptr<void> ptr = create_deserialized_msg(type);
  messages_[ptr.get()] = {ptr, type};
  clips::UDFValue res;
  res.externalAddressValue = clips::CreateCExternalAddress(env, ptr.get());
  return res;
}

void RosMsgsPlugin::destroy_msg(void *msg) {
  auto it = messages_.find(msg);
  if (it != messages_.end()) {
    messages_.erase(it);
  }
}

clips::UDFValue RosMsgsPlugin::ros_message_member_to_udf_value(
    clips::Environment *env, void *deserialized_msg,
    const rosidl_typesupport_introspection_cpp::MessageMember &member) {
  clips::UDFValue res;
  res.begin = 0;
  res.range = -1;
  // Check if the member is an array
  if (member.is_array_) {
    // Determine the size of the array
    size_t array_size = member.array_size_;
    void *member_loc =
        reinterpret_cast<uint8_t *>(deserialized_msg) + member.offset_;

    if (array_size == 0) {
      array_size = member.size_function(member_loc);
    }
    clips::MultifieldBuilder *mb =
        clips::CreateMultifieldBuilder(env, array_size);
    clips::Multifield *mf;

    // Iterate over each element in the array
    for (size_t i = 0; i < array_size; ++i) {
      void *element = member.get_function(member_loc, i);
      if (!element) {
        RCLCPP_ERROR(*logger_, "Failed to retreive element");
        continue; // Skip if the element is null
      }
      clips::CLIPSValue elem =
          ros_to_clips_value(env, element, member.type_id_);
      clips::MBAppend(mb, &elem);
    }
    mf = clips::MBCreate(mb);
    clips::MBDispose(mb);
    res.multifieldValue = mf;
  } else {
    void *value =
        reinterpret_cast<void *>((uint8_t *)deserialized_msg + member.offset_);
    clips::CLIPSValue elem = ros_to_clips_value(env, value, member.type_id_);
    res.value = elem.value;
  }
  return res;
}

clips::CLIPSValue RosMsgsPlugin::ros_to_clips_value(clips::Environment *env,
                                                    void *val,
                                                    uint8_t ros_type) {
  clips::CLIPSValue res;
  switch (ros_type) {
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL: {
    bool *bool_value = reinterpret_cast<bool *>(val);
    res.value = clips::CreateBoolean(env, *bool_value);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE: {
    int8_t *byte_value = reinterpret_cast<int8_t *>(val);
    res.value = clips::CreateInteger(env, *byte_value);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8: {
    int8_t *int8_value = reinterpret_cast<int8_t *>(val);
    res.value = clips::CreateInteger(env, *int8_value);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8: {
    uint8_t *uint8_value = reinterpret_cast<uint8_t *>(val);
    res.value = clips::CreateInteger(env, *uint8_value);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16: {
    int16_t *int16_value = reinterpret_cast<int16_t *>(val);
    res.value = clips::CreateInteger(env, *int16_value);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16: {
    uint16_t *uint16_value = reinterpret_cast<uint16_t *>(val);
    res.value = clips::CreateInteger(env, *uint16_value);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32: {
    int32_t *int32_value = reinterpret_cast<int32_t *>(val);
    res.value = clips::CreateInteger(env, *int32_value);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32: {
    uint32_t *uint32_value = reinterpret_cast<uint32_t *>(val);
    res.value = clips::CreateInteger(env, *uint32_value);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64: {
    int64_t *int64_value = reinterpret_cast<int64_t *>(val);
    res.value = clips::CreateInteger(env, *int64_value);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64: {
    uint64_t *uint64_value = reinterpret_cast<uint64_t *>(val);
    res.value = clips::CreateInteger(env, *uint64_value);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT: {
    float *float_value = reinterpret_cast<float *>(val);
    res.value = clips::CreateFloat(env, *float_value);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE: {
    double *double_value = reinterpret_cast<double *>(val);
    res.value = clips::CreateFloat(env, *double_value);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING: {
    std::string *str_value = reinterpret_cast<std::string *>(val);
    res.value = clips::CreateString(env, str_value->c_str());
    break;
  }
  default:
    throw std::runtime_error("Unsupported field type.");
  }
  return res;
}

clips::UDFValue RosMsgsPlugin::get_field(clips::Environment *env,
                                         void *deserialized_msg,
                                         const std::string &field,
                                         clips::UDFContext *udfc) {

  clips::UDFValue res;
  res.begin = 0;
  res.range = -1;

  if (!deserialized_msg || !messages_.contains(deserialized_msg)) {
    RCLCPP_ERROR(*logger_, "ros-msgs-get-field: Invalid pointer ");
    clips::UDFThrowError(udfc);
    return res;
  }
  if (!messages_.contains(deserialized_msg)) {
  }
  std::string message_type = messages_[deserialized_msg].second;

  const rosidl_typesupport_introspection_cpp::MessageMembers *members =
      introspection_info_cache_[message_type];

  if (!members) {
  }

  // Search for the field in the message
  for (size_t i = 0; i < members->member_count_; ++i) {
    const auto &member = members->members_[i];
    if (field == member.name_) {
      return ros_message_member_to_udf_value(env, deserialized_msg, member);
    }
  }
  RCLCPP_ERROR(*logger_, "Failed to retrieve field %s", field.c_str());
  return res;
}

void RosMsgsPlugin::set_field(clips::Environment *env, void *deserialized_msg,
                              const std::string &field, clips::UDFValue &val,
                              clips::UDFContext *udfc) {

  if (!deserialized_msg || !messages_.contains(deserialized_msg)) {
    RCLCPP_ERROR(*logger_, "ros-msgs-get-field: Invalid pointer ");
    clips::UDFThrowError(udfc);
    return;
  }
  if (!messages_.contains(deserialized_msg)) {
  }
  std::string message_type = messages_[deserialized_msg].second;

  const rosidl_typesupport_introspection_cpp::MessageMembers *members =
      introspection_info_cache_[message_type];

  if (!members) {
  }

  // Search for the field in the message
  for (size_t i = 0; i < members->member_count_; ++i) {
    const auto &member = members->members_[i];
    if (field == member.name_) {
      udf_value_to_ros_message_member(env, deserialized_msg, member, val);
      return;
    }
  }
  RCLCPP_ERROR(*logger_, "Failed to retrieve field %s", field.c_str());
}

void RosMsgsPlugin::udf_value_to_ros_message_member(
    clips::Environment *env, void *deserialized_msg,
    const rosidl_typesupport_introspection_cpp::MessageMember &member,
    clips::UDFValue &val) {
  // Check if the member is an array
  uint8_t *field_ptr =
      static_cast<uint8_t *>(deserialized_msg) + member.offset_;
  if (member.is_array_) {
    if (val.header->type != MULTIFIELD_TYPE) {
    }
    member.resize_function(field_ptr, val.multifieldValue->length);
    uint8_t *field_index_ptr;
    for (size_t i = 0; i < val.multifieldValue->length; i++) {
      field_index_ptr =
          static_cast<uint8_t *>(member.get_function(field_ptr, i));
      clips_to_ros_value(env, val.multifieldValue->contents[i], member,
                         field_index_ptr);
    }
  } else {
    clips::CLIPSValue clips_val;
    clips_val.value = val.value;
    clips_to_ros_value(env, clips_val, member, field_ptr);
  }
}

void RosMsgsPlugin::clips_to_ros_value(
    clips::Environment *env, const clips::CLIPSValue &val,
    const rosidl_typesupport_introspection_cpp::MessageMember &member,
    uint8_t *field_ptr) {
  switch (member.type_id_) {
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL: {
    bool *bool_field = reinterpret_cast<bool *>(field_ptr);
    *bool_field = (strcmp(val.lexemeValue->contents, "TRUE") == 0);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8: {
    int8_t *int8_field = reinterpret_cast<int8_t *>(field_ptr);
    *int8_field = static_cast<int8_t>(val.integerValue->contents);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8: {
    uint8_t *uint8_field = reinterpret_cast<uint8_t *>(field_ptr);
    *uint8_field = static_cast<uint8_t>(val.integerValue->contents);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16: {
    int16_t *int16_field = reinterpret_cast<int16_t *>(field_ptr);
    *int16_field = static_cast<int16_t>(val.integerValue->contents);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16: {
    uint16_t *uint16_field = reinterpret_cast<uint16_t *>(field_ptr);
    *uint16_field = static_cast<uint16_t>(val.integerValue->contents);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32: {
    int32_t *int32_field = reinterpret_cast<int32_t *>(field_ptr);
    *int32_field = static_cast<int32_t>(val.integerValue->contents);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32: {
    uint32_t *uint32_field = reinterpret_cast<uint32_t *>(field_ptr);
    *uint32_field = static_cast<uint32_t>(val.integerValue->contents);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64: {
    int64_t *int64_field = reinterpret_cast<int64_t *>(field_ptr);
    *int64_field = static_cast<int64_t>(val.integerValue->contents);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64: {
    uint64_t *uint64_field = reinterpret_cast<uint64_t *>(field_ptr);
    *uint64_field = static_cast<uint64_t>(val.integerValue->contents);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT: {
    float *float_field = reinterpret_cast<float *>(field_ptr);
    *float_field = static_cast<float>(val.floatValue->contents);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE: {
    double *double_field = reinterpret_cast<double *>(field_ptr);
    *double_field = static_cast<double>(val.floatValue->contents);
    break;
  }
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING: {
    std::string *string_field = reinterpret_cast<std::string *>(field_ptr);
    const char *clips_string = val.lexemeValue->contents;
    *string_field = std::string(clips_string);
    break;
  }
  default:
    clips::Writeln(env, "Unsupported field type for setting value");
    // clips::UDFThrowError(udfc);
    throw std::runtime_error("Unsupported field type for setting value.");
  }
}

} // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::RosMsgsPlugin, cx::ClipsPlugin)
