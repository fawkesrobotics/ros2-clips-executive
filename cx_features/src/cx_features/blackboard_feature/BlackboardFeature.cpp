/***************************************************************************
 *  BlackboardFeature.cpp
 *
 *  Created: 28 August 2021
 *  Copyright  2021  Ivaylo Doychev
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

#include <regex>

#include "cx_features/BlackboardFeature.hpp"

#include <rosbag2_cpp/types/introspection_message.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosidl_runtime_c/message_initialization.h>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

namespace cx {

BlackboardFeature::BlackboardFeature() {}

BlackboardFeature::~BlackboardFeature() {}

std::string BlackboardFeature::getFeatureName() const {
  return clips_feature_name;
}

void BlackboardFeature::initialise(const std::string &feature_name) {
  clips_feature_name = feature_name;
}

bool BlackboardFeature::clips_context_init(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialising context for feature %s",
              clips_feature_name.c_str());
  envs_[env_name] = clips;
  clips->evaluate("(path-load \"blackboard.clp\")");
  clips->add_function(
      "blackboard-enable-time-read",
      sigc::slot<void>(sigc::bind<0>(
          sigc::mem_fun(*this,
                        &BlackboardFeature::clips_blackboard_enable_time_read),
          env_name)));
  clips->add_function(
      "blackboard-open",
      sigc::slot<void, std::string, std::string>(sigc::bind<0>(
          sigc::mem_fun(
              *this,
              &BlackboardFeature::clips_blackboard_open_interface_reading),
          env_name)));
  clips->add_function(
      "blackboard-open-reading",
      sigc::slot<void, std::string, std::string>(sigc::bind<0>(
          sigc::mem_fun(
              *this,
              &BlackboardFeature::clips_blackboard_open_interface_reading),
          env_name)));
  // clips->add_function(
  //   "blackboard-open-writing",
  //   sigc::slot<void, std::string, std::string>(sigc::bind<0>(
  //     sigc::mem_fun(*this,
  //     &BlackboardFeature::clips_blackboard_open_interface_writing),
  //     env_name)));
  // clips->add_function(
  //   "blackboard-close",
  //   sigc::slot<void, std::string, std::string>(
  //     sigc::bind<0>(sigc::mem_fun(*this,
  //     &BlackboardFeature::clips_blackboard_close_interface),
  //                   env_name)));
  // clips->add_function("blackboard-preload",
  //                     sigc::slot<void, std::string>(sigc::bind<0>(
  //                       sigc::mem_fun(*this,
  //                       &BlackboardFeature::clips_blackboard_preload),
  //                       env_name)));
  clips->add_function(
      "blackboard-read",
      sigc::slot<void>(sigc::bind<0>(
          sigc::mem_fun(*this, &BlackboardFeature::clips_blackboard_read),
          env_name)));
  // clips->add_function("blackboard-write",
  //                     sigc::slot<void, std::string>(sigc::bind<0>(
  //                       sigc::mem_fun(*this,
  //                       &BlackboardFeature::clips_blackboard_write),
  //                       env_name)));
  // clips->add_function("blackboard-get-info",
  //                     sigc::slot<void>(sigc::bind<0>(
  //                       sigc::mem_fun(*this,
  //                       &BlackboardFeature::clips_blackboard_get_info),
  //                       env_name)));
  // clips->add_function("blackboard-set",
  //                     sigc::slot<void, std::string, std::string,
  //                     CLIPS::Value>(sigc::bind<0>(
  //                       sigc::mem_fun(*this,
  //                       &BlackboardFeature::clips_blackboard_set),
  //                       env_name)));
  // clips->add_function(
  //   "blackboard-set-multifield",
  //   sigc::slot<void, std::string, std::string, CLIPS::Values>(
  //     sigc::bind<0>(sigc::mem_fun(*this,
  //     &BlackboardFeature::clips_blackboard_set_multifield),
  //                   env_name)));
  clips->add_function(
      "blackboard-create-msg",
      sigc::slot<CLIPS::Value, std::string, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this, &BlackboardFeature::clips_blackboard_create_msg),
          env_name)));
  // clips->add_function(
  //   "blackboard-list-msg-fields",
  //   sigc::slot<CLIPS::Values, void *>(
  //     sigc::bind<0>(sigc::mem_fun(*this,
  //     &BlackboardFeature::clips_blackboard_list_msg_fields),
  //                   env_name)));
  // clips->add_function(
  //   "blackboard-set-msg-field",
  //   sigc::slot<void, void *, std::string, CLIPS::Value>(
  //     sigc::bind<0>(sigc::mem_fun(*this,
  //     &BlackboardFeature::clips_blackboard_set_msg_field),
  //                   env_name)));
  // clips->add_function("blackboard-set-msg-multifield",
  //                     sigc::slot<void, void *, std::string,
  //                     CLIPS::Values>(sigc::bind<0>(
  //                       sigc::mem_fun(*this,
  //                                     &BlackboardFeature::clips_blackboard_set_msg_multifield),
  //                       env_name)));
  // clips->add_function("blackboard-send-msg",
  //                     sigc::slot<CLIPS::Value, void *>(sigc::bind<0>(
  //                       sigc::mem_fun(*this,
  //                       &BlackboardFeature::clips_blackboard_send_msg),
  //                       env_name)));

  return true;
}

bool BlackboardFeature::clips_context_destroyed(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Destroying clips context for feature %s!",
              clips_feature_name.c_str());

  // TODO Destroy the interface map and close all interfaces!

  envs_.erase(env_name);

  return true;
}

bool BlackboardFeature::create_bb_reader(const std::string &env_name,
                                         const std::string &i_type,
                                         const std::string &i_id,
                                         const std::string &reader_id,
                                         bool new_interface) {
  cx::InterfaceManager::Interface_Reader_Types_Variant reader;
  const std::string log_name = "BBCLIPS|" + env_name;

  try {
    if (i_type == "PddlGenInterface") {
      reader = std::make_shared<cx::InterfaceReader<
          cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
          cx_msgs::msg::PddlGenInterfaceMessages>>(reader_id, i_type, i_id);
      RCLCPP_INFO(rclcpp::get_logger(log_name), "After Pddl generation!");
    }

    if (new_interface) {
      std::vector<std::pair<
          std::string, cx::InterfaceManager::Interface_Reader_Types_Variant>>
          i_vec{std::make_pair(i_id, reader)};
      interfaces_[env_name].reading.insert({i_type, i_vec});
      return true;
    } else {
      interfaces_[env_name].reading[i_type].push_back(
          std::make_pair(i_id, reader));
      return true;
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(
        rclcpp::get_logger(log_name),
        "Failed to open interface for reading %s:%s, exception follows",
        i_type.c_str(), i_id.c_str());
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(clips_feature_name), e.what());
    return false;
  }

  return false;
}

cx::InterfaceManager::Interface_Reader_Types_Optional
BlackboardFeature::cast_bb_reader(const std::string &env_name,
                                  const std::string &i_type,
                                  const std::string &i_id) {
  const std::string log_name = "BBCLIPS|" + env_name;
  auto index = interfaces_.find(env_name);
  if (index == interfaces_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "No such blackboard clips environment: %s", env_name.c_str());
    return nullptr;
  }
  auto reader_index = index->second.reading.find(i_type);
  if (reader_index == index->second.reading.end()) {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Interface of type %s is not registered for environment %s",
                 i_type.c_str(), env_name.c_str());
  }

  cx::InterfaceManager::Interface_Reader_Types_Optional ret_reader;
  auto &i_list = index->second.reading[i_type];

  auto found_variant =
      std::find_if(i_list.begin(), i_list.end(),
                   [&i_id](const auto &i) -> bool { return i_id == i.first; });
  if (found_variant == std::end(i_list))
    return nullptr;

  try {

    if (i_type == "PddlGenInterface") {
      ret_reader = std::get<std::shared_ptr<cx::InterfaceReader<
          cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
          cx_msgs::msg::PddlGenInterfaceMessages>>>(found_variant->second);
    } else {
      throw std::runtime_error("No such interface in bb_feature!");
    }
  } catch (const std::bad_variant_access &e) {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Bad interface cast for the given type: %s and id: %s",
                 i_type.c_str(), i_id.c_str());
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(log_name), e.what());
    return nullptr;
  }

  return ret_reader;
}

bool BlackboardFeature::clips_assert_interface_type(const std::string &env_name,
                                                    const std::string &log_name,
                                                    const std::string &i_type) {
  std::string deftemplate =
      "(deftemplate " + i_type + "\n" + "  (slot id (type STRING))\n" +
      "  (multislot time (type INTEGER) (cardinality 2 2))\n";

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "In assert fact!");
  // Populate with interface fields

  using TypeSupport = rosidl_message_type_support_t;

  using namespace rosidl_typesupport_introspection_cpp;

  std::string i_library = "cx_msgs/" + i_type;

  auto library = rosbag2_cpp::get_typesupport_library(
      i_library, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  const auto *typesupport = rosbag2_cpp::get_typesupport_handle(
      i_library, rosidl_typesupport_introspection_cpp::typesupport_identifier,
      library);

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "After ts lib!");

  std::function<void(const TypeSupport *, std::string &deftemplate)>
      recursivelyAssertInterface;

  recursivelyAssertInterface = [&](const TypeSupport *type_data,
                                   std::string &deftemplate) {
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "Enter recursion");

    const auto *members = static_cast<const MessageMembers *>(type_data->data);
    RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "After members!");

    for (size_t i = 0; i < members->member_count_; i++) {
      const MessageMember &member = members->members_[i];
      RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "Before switch!");

      switch (member.type_id_) {
      case ROS_TYPE_FLOAT:
      case ROS_TYPE_DOUBLE:
        deftemplate += std::string() + "  (" +
                       ((member.is_array_) ? "multi" : "") + "slot " +
                       member.name_ + " (type FLOAT))\n";
        break;

      case ROS_TYPE_INT8:
      case ROS_TYPE_UINT8:
      case ROS_TYPE_INT16:
      case ROS_TYPE_UINT16:
      case ROS_TYPE_INT32:
      case ROS_TYPE_UINT32:
      case ROS_TYPE_INT64:
      case ROS_TYPE_UINT64: {
        deftemplate += std::string() + "  (" +
                       ((member.is_array_) ? "multi" : "") + "slot " +
                       member.name_ + " (type INTEGER))\n";
        break;
      }
      case ROS_TYPE_BOOLEAN:
        deftemplate += std::string() + "  (" +
                       ((member.is_array_) ? "multi" : "") + "slot " +
                       member.name_ +
                       " (type SYMBOL) (allowed-values TRUE FALSE))\n";
        break;
      case ROS_TYPE_STRING:
        deftemplate +=
            std::string() + "  (slot " + member.name_ + " (type STRING))\n";
        break;
      case ROS_TYPE_MESSAGE:
        recursivelyAssertInterface(member.members_, deftemplate);
        break;
      }
    }
  };
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "After declaration of recursiveness");

  recursivelyAssertInterface(typesupport, deftemplate);

  deftemplate += ")";

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "%s",
              deftemplate.c_str());
  std::string retract;
  std::string logstr;

  if (cfg_retract_early_) {
    retract = "(defrule " + i_type + "-cleanup\n" +
              "  (declare (salience -10000))\n" + "  ?f <- (" + i_type + ")\n" +
              "  =>\n"
              "  (retract ?f)\n"
              ")";
    logstr = "Defrule";
  } else {
    retract = "(deffunction " + i_type +
              "-cleanup-late (?id)\n"
              "  (delayed-do-for-all-facts ((?f " +
              i_type +
              "))\n"
              "    (eq ?f:id ?id)\n"
              "    (retract ?f)\n"
              "  )\n"
              ")";
    logstr = "Deffunction";
  }

  if (envs_[env_name]->build(deftemplate) && envs_[env_name]->build(retract)) {
    // Later DEBUG
    RCLCPP_INFO(rclcpp::get_logger(log_name), "Deftemplate:\n%s",
                deftemplate.c_str());
    RCLCPP_INFO(rclcpp::get_logger(log_name), "%s:\n%s", logstr.c_str(),
                retract.c_str());
    return true;
  } else {
    RCLCPP_WARN(rclcpp::get_logger(log_name),
                "Defining blackboard type for %s in %s failed", i_type.c_str(),
                env_name.c_str());
    return false;
  }
}

void BlackboardFeature::clips_blackboard_open_interface(
    const std::string &env_name, const std::string &i_type,
    const std::string &i_id, bool writing) {
  const std::string log_name = "BBCLIPS|" + env_name;
  const std::string owner = "CLIPS:" + env_name;
  const std::string uid = std::move(i_type + "::" + i_id);
  const std::string node_id = std::move(i_type + "_" + i_id);

  if (envs_.find(env_name) == envs_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Environment %s has not been registered "
                 "for blackboard feature",
                 env_name.c_str());
    return;
  }

  cx::LockSharedPtr<CLIPS::Environment> clips = envs_[env_name];

  auto &i_map = interfaces_[env_name];

  if (i_map.reading.find(i_type) == i_map.reading.end() &&
      i_map.writing.find(i_type) == i_map.writing.end()) {
    // no interface of this type registered yet, add deftemplate for it
    // subst with create_bb_writer later!
    bool i_created = writing
                         ? true
                         : create_bb_reader(env_name, i_type, i_id, node_id,
                                            /*ceate interface in map*/ true);
    if (!i_created)
      return;

    if (!clips_assert_interface_type(env_name, log_name, i_type)) {
      // blackboard_->close(iface);
      return;
    } else {
      RCLCPP_INFO(rclcpp::get_logger(log_name), "Added interface %s for %s",
                  uid.c_str(), writing ? "writing" : "reading");
      std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));
      clips->assert_fact_f(
          "(blackboard-interface (id \"%s\") (type \"%s\") (uid "
          "\"%s\") (writing %s))",
          i_id.c_str(), i_type.c_str(), uid.c_str(),
          writing ? "TRUE" : "FALSE");
      RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "After assert fact!");
    }
  } else {

    bool is_already_available = false;
    if (writing) {
      // if (cast_bb_writer() != nullptr)
      // is_already_available = true;
    } else {
      if (cast_bb_reader(env_name, i_type, i_id) != nullptr)
        is_already_available = true;
    }

    if (is_already_available)
      return;

    bool i_created = writing
                         ? true
                         : create_bb_reader(env_name, i_type, i_id, node_id,
                                            /*ceate interface in map*/ true);
    if (!i_created)
      return;
    std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));
    clips->assert_fact_f("(blackboard-interface (id \"%s\") (type \"%s\") (uid "
                         "\"%s\") (writing %s))",
                         i_id.c_str(), i_type.c_str(), uid.c_str(),
                         writing ? "TRUE" : "FALSE");
  }
}
void BlackboardFeature::clips_blackboard_enable_time_read(
    const std::string &env_name) {
  const std::string log_name = "BBCLIPS|" + env_name;
  if (envs_.find(env_name) == envs_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Cannot enable reading for environment %s "
                 "(not defined)",
                 env_name.c_str());
    return;
  }

  std::string bb_read_defrule = "(defrule blackboard-read\n"
                                "  (declare (salience 1000))\n"
                                "  (time $?)\n"
                                "  =>\n"
                                "  (blackboard-read)\n"
                                ")";
  std::lock_guard<std::mutex> guard(*(envs_[env_name].get_mutex_instance()));
  envs_[env_name]->build(bb_read_defrule);
}

void BlackboardFeature::clips_blackboard_open_interface_reading(
    const std::string &env_name, const std::string &i_type,
    const std::string &i_id) {
  clips_blackboard_open_interface(env_name, i_type, i_id, /* writing */ false);
}

// void BlackboardFeature::clips_blackboard_open_interface_writing(
//     const std::string &env_name, const std::string &i_i_type,
//     const std::string &i_id) {
//   clips_blackboard_open_interface(env_name, i_type, i_id, /* writing */
//   true);
// }

// void
// BlackboardFeature::clips_blackboard_close_interface(const std::string
// &env_name,const std::string &type, const std::string &id)
// {
//   const std::string log_name = "BBCLIPS|" + env_name;

// }

void BlackboardFeature::clips_blackboard_read(const std::string &env_name) {
  const std::string log_name = "BBCLIPS|" + env_name;

  if (interfaces_.find(env_name) == interfaces_.end())
    return;

  if (envs_.find(env_name) == envs_.end()) {
    // Environment not registered, big bug
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Environment %s not registered,"
                 " cannot read interfaces",
                 env_name.c_str());
    return;
  }
  std::lock_guard<std::mutex> guard(*(envs_[env_name].get_mutex_instance()));
  // CLIPS::Environment &env = **(envs_[env_name]);

  // for (auto &iface_map : interfaces_[env_name].reading) {
  //   for (auto i : iface_map.second.second) {
  //     i->read_from_interface();

  //     if (!cfg_retract_early_) {
  //       std::string fun =
  //           std::string("(") + i->type() + "-cleanup-late \"" + i->id() +
  //           "\")";
  //       env.evaluate(fun);
  //     }
  //     // TODO: IMPLEMENT READ FUNC
  //   }
  // }
}

CLIPS::Value
BlackboardFeature::clips_blackboard_create_msg(const std::string &env_name,
                                               const std::string &i_uid,
                                               const std::string &msg_type) {
  const std::string log_name = "BBCLIPS|" + env_name;

  if (interfaces_.find(env_name) == interfaces_.end()) {
    // return CLIPS::Value(new std::shared_ptr<Message>());
  }

  if (envs_.find(env_name) == envs_.end()) {
    // Environment not registered, big bug
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Environment %s not registered,"
                 " cannot create message",
                 env_name.c_str());
    // return CLIPS::Value(new std::shared_ptr<Message>());
  }
  std::lock_guard<std::mutex> guard(*(envs_[env_name].get_mutex_instance()));

  std::regex delimiter{"::"};
  std::string i_type, i_id;
  std::vector<std::string> parsed_uid(
      std::sregex_token_iterator(i_uid.begin(), i_uid.end(), delimiter, -1),
      {});

  i_type = parsed_uid[0];
  i_id = parsed_uid[1];

  if (interfaces_[env_name].reading.find(i_type) ==
      interfaces_[env_name].reading.end()) {
    RCLCPP_WARN(rclcpp::get_logger(log_name),
                "Can't create message for interface %s, because there is "
                "no opened interface with this type",
                i_type.c_str());
    // return CLIPS::Value(new std::shared_ptr<Message>());
  }
  auto i_reader = cast_bb_reader(env_name, i_type, i_id);
  if (i_reader == nullptr) {
    RCLCPP_WARN(rclcpp::get_logger(log_name),
                "Can't create message for interface %s, because there is no "
                "opened interface with that uid",
                i_uid.c_str());
    // return CLIPS::Value(new std::shared_ptr<Message>());
  }
  auto msg_ptr = i_reader.value()->create_interface_message(msg_type);
  // REPAIR LATER
  return CLIPS::Value(i_reader.value()->create_void_message(msg_ptr));
}

// void BlackboardFeature::clips_blackboard_set_msg_field(
//     const std::string &env_name, const std::string &i_type,
//     const std::string &i_id, void *msgptr, const std::string &field_name,
//     CLIPS::Value value) {
//   const std::string log_name = "BBCLIPS|" + env_name;
//   auto i_reader = cast_bb_reader(env_name, i_type, i_id);
//   auto m = i_reader->cast_void_message(msgptr);
//   if (!*m) {
//     RCLCPP_WARN(rclcpp::get_logger(log_name),
//                 "Can't set message field, the pointer is wrong.");
//     return;
//   }

//   bool set_success = set_field(env_name, field_name, value);
//   if (!set_success) {
//     RCLCPP_WARN(rclcpp::get_logger(log_name), "Can't set message field.");
//   }
// }

// /**
//    Set field of an InterfaceFieldIterator of an Interface or Message.
//    @index index in an array of the interface (leave default for non array
//    value)
//    @return if field could successfully be set
//  */
// bool BlackboardFeature::set_field(const std::string &env_name,
//                                   const std::string &field,
//                                   CLIPS::Value value) {}
} // namespace cx