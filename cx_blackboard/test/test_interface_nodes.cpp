#include <any>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "gtest/gtest.h"

#include "cx_msgs/msg/pddl_gen_interface.hpp"
#include "cx_msgs/msg/pddl_gen_interface_messages.hpp"
#include "cx_msgs/srv/pddl_gen_interface_srv.hpp"

#include "test_msgs/msg/strings.hpp"

#include <rosbag2_cpp/types/introspection_message.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosidl_runtime_c/message_initialization.h>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include "test_utils.h"
#include <ros2_introspection/ros2_introspection.hpp>

#include "cx_blackboard/BlackboardNode.hpp"
#include "cx_blackboard/Interface.hpp"
#include "cx_blackboard/InterfaceManager.hpp"
#include "cx_blackboard/InterfaceNode.hpp"
#include "cx_blackboard/InterfaceReader.hpp"
#include "cx_blackboard/InterfaceWriter.hpp"

using namespace std::chrono_literals;
using namespace Ros2Introspection;

class Mock {
private:
public:
  Mock(){};
  // ~Mock();

  void foo(void *msg) {
    msg = reinterpret_cast<cx_msgs::msg::PddlGenInterface *>(msg);
  }
};

// class InterfaceClient : public rclcpp::Node {
// private:
//
// public:
//   InterfaceClient() : rclcpp::Node("pddl_gen_interface_client") {}
//   // ~InterfaceClient();

//   rclcpp::Client<cx_msgs::srv::PddlGenInterfaceSrv>::SharedPtr
//       pddl_gen_interface_write_client =
//           create_client<cx_msgs::srv::PddlGenInterfaceSrv>(
//               "pddl_gen_interface_write");

//   bool write(std::shared_ptr<cx_msgs::msg::PddlGenInterface> input_msg) {
//     while (!pddl_gen_interface_write_client->wait_for_service(1s)) {
//       if (!rclcpp::ok()) {
//         RCLCPP_ERROR(get_logger(),
//                      "%s: timed out waiting for service availability",
//                      pddl_gen_interface_write_client->get_service_name());
//         return false;
//       }
//       RCLCPP_WARN(get_logger(), "%s: still waiting for service...",
//                   pddl_gen_interface_write_client->get_service_name());
//     }

//     auto req =
//     std::make_shared<cx_msgs::srv::PddlGenInterfaceSrv::Request>();
//     req->interface = *input_msg;
//     auto future_res =
//     pddl_gen_interface_write_client->async_send_request(req);

//     auto node_ = shared_from_this();
//     if (rclcpp::spin_until_future_complete(node_, future_res, 10s) !=
//         rclcpp::FutureReturnCode::SUCCESS) {
//       RCLCPP_ERROR(get_logger(), "%s: timed out waiting for response!",
//                    pddl_gen_interface_write_client->get_service_name());
//       return false;
//     }

//     // auto status = future_res.wait_for(10s);

//     // if (status == std::future_status::ready) {

//     if (future_res.get()->success) {
//       return true;
//     } else {
//       RCLCPP_ERROR(get_logger(), "ERROR --- %s",
//                    future_res.get()->error.c_str());
//       return false;
//     }
//     // } else {
//     //   RCLCPP_ERROR(get_logger(), "timed out waiting for response!");
//     //   return false;
//     // }
//   }
// };

// TEST(test_interface_nodes, test_context_init) {

//   auto testing_node = rclcpp::Node::make_shared("testing_node");
//   auto pddl_gen_interface =
//       std::make_shared<cx::Interface<cx_msgs::msg::PddlGenInterface>>(
//           "PddlGenInterface", "pddl-gen");

//   auto interface_client_node = std::make_shared<InterfaceClient>();

//   RCLCPP_INFO(testing_node->get_logger(),
//               "Initialised interface of type %s, id %s and uid %s",
//               pddl_gen_interface->type_.c_str(),
//               pddl_gen_interface->id_.c_str(),
//               pddl_gen_interface->uid_.c_str());

//   try {
//     RCLCPP_INFO(testing_node->get_logger(), "Trying to set interface
//     data..."); pddl_gen_interface->interface_data_ptr_->set__msg_id(2);
//     RCLCPP_INFO(testing_node->get_logger(), "Set msg_id");
//     pddl_gen_interface->interface_data_ptr_->set__final(true);
//     RCLCPP_INFO(testing_node->get_logger(), "Set final");

//     ASSERT_TRUE(pddl_gen_interface->interface_data_ptr_->msg_id == 2);
//     ASSERT_TRUE(pddl_gen_interface->interface_data_ptr_->final == true);

//   } catch (const std::exception &e) {
//     std::cerr << e.what() << '\n';
//   }

//   auto pddl_gen_interface_node =
//       std::make_shared<cx::InterfaceNode<cx_msgs::srv::PddlGenInterfaceSrv,
//                                          cx_msgs::msg::PddlGenInterface>>(
//           "pddl_gen_interface", pddl_gen_interface);

//   try {
//     RCLCPP_INFO(testing_node->get_logger(),
//                 "Assure that the interface ptr is pointed to the same
//                 target");
//     //
//     pddl_gen_interface_node->interface_->interface_data_ptr_->set__msg_id(2);
//     // RCLCPP_INFO(testing_node->get_logger(), "Set msg_id");
//     //
//     pddl_gen_interface_node->interface_->interface_data_ptr_->set__final(true);
//     // RCLCPP_INFO(testing_node->get_logger(), "Set final");

//     ASSERT_TRUE(
//         pddl_gen_interface_node->interface_->interface_data_ptr_->msg_id ==
//         2);
//     ASSERT_TRUE(
//         pddl_gen_interface_node->interface_->interface_data_ptr_->final ==
//         true);

//   } catch (const std::exception &e) {
//     std::cerr << e.what() << '\n';
//   }

//   auto msg = std::make_shared<cx_msgs::msg::PddlGenInterface>();
//   msg->set__msg_id(1);
//   msg->set__final(false);

//   // Executors
//   rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 1);

//   exe.add_node(pddl_gen_interface_node->get_node_base_interface());
//   // exe.add_node(interface_client_node->get_node_base_interface());

//   bool finish_exec = false;
//   std::thread t([&]() {
//     while (!finish_exec) {
//       exe.spin_some();
//     }
//   });

//   // // using lc_tr = lifecycle_msgs::msg::Transition;

//   // // manager_node->trigger_transition(lc_tr::TRANSITION_CONFIGURE);
//   // // features_manager->trigger_transition(lc_tr::TRANSITION_CONFIGURE);

//   {
//     rclcpp::Rate rate(10);
//     auto start_time = testing_node->now();
//     while ((testing_node->now() - start_time).seconds() < 0.5) {
//       rate.sleep();
//     }
//   }

//   // // manager_node->trigger_transition(lc_tr::TRANSITION_ACTIVATE);
//   // // features_manager->trigger_transition(lc_tr::TRANSITION_ACTIVATE);

//   // // {
//   // //   rclcpp::Rate rate(10);
//   // //   auto start_time = testing_node->now();
//   // //   while ((testing_node->now() - start_time).seconds() < 0.5) {
//   // //     rate.sleep();
//   // //   }
//   // // }

//   RCLCPP_INFO(testing_node->get_logger(), "TEST-NODE-START");

//   // // auto created_env = manager_node->getEnvironmentByName(env_name);
//   // try {

//   //   // ASSERT_TRUE(manager_client->addFeatures(flist));
//   for (size_t i = 0; i < 1000; i++) {
//     RCLCPP_INFO(testing_node->get_logger(), "Try %ld", i);

//     auto loop_msg = std::make_shared<cx_msgs::msg::PddlGenInterface>();
//     loop_msg->set__msg_id(i);
//     loop_msg->set__final(false);

//     ASSERT_TRUE(interface_client_node->write(loop_msg));
//     ASSERT_EQ(pddl_gen_interface_node->interface_->interface_data_ptr_->msg_id,
//               i);
//     ASSERT_EQ(pddl_gen_interface_node->interface_->interface_data_ptr_->final,
//               false);

//     ASSERT_EQ(pddl_gen_interface->interface_data_ptr_->msg_id, i);
//     ASSERT_EQ(pddl_gen_interface->interface_data_ptr_->final, false);
//   }

//   //   // ASSERT_FALSE();

//   // } catch (const std::exception &e) {
//   //   std::cerr << e.what() << '\n';
//   // }

//   RCLCPP_INFO(testing_node->get_logger(), "TEST-NODE-END");

//   finish_exec = true;
//   t.join();
// }

TEST(test_interface_nodes_, test_rosbag2_typesupport) {

  try {
    RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "TEST-NODE-START");

    // auto library = rosbag2_cpp::get_typesupport_library(
    //     "cx_msgs/msg/PddlGenInterface",
    //     "rosidl_typesupport_introspection_cpp");
    // auto string_typesupport = rosbag2_cpp::get_typesupport_handle(
    //     "cx_msgs/msg/PddlGenInterface",
    //     "rosidl_typesupport_introspection_cpp", library);

    // auto allocator_ = rcutils_get_default_allocator();
    // auto ros_msg = rosbag2_cpp::allocate_introspection_message(
    //     string_typesupport, &allocator_);

    // auto mock = std::make_shared<Mock>();
    // mock->foo(ros_msg->message);

    // RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "After intro_msg");

    // // rclcpp::SerializedMessage serialized_msg;

    // // auto seri =
    // // std::make_shared<rclcpp::SerializationBase>(string_typesupport);

    // // seri->deserialize_message(&serialized_msg, &ros_msg);

    // RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "TEST-NODE-START");
    // RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "Type ID: %s",
    //             string_typesupport->typesupport_identifier);
    // // auto p = static_cast<cx_msgs::msg::PddlGenInterface
    // *>(ros_msg->message);
    // // p->set__final(true);

    // auto msg_lib = rosbag2_cpp::get_typesupport_library(
    //     "test_msgs/Strings", "rosidl_typesupport_cpp");
    // auto pddl_ts = rosbag2_cpp::get_typesupport_handle(
    //     "test_msgs/msg/Strings", "rosidl_typesupport_cpp", msg_lib);

    // RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "Type ID: %s",
    //             pddl_ts->typesupport_identifier);

    // using rosidl_typesupport_introspection_cpp::MessageMember;
    // using rosidl_typesupport_introspection_cpp::MessageMembers;
    // using rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;

    // auto members = static_cast<
    //     const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    //     pddl_ts->data);

    // RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "Name: %s",
    //             members->message_name_);
    // RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "Size: %ld",
    //             members->size_of_);
    // RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "Count: %d",
    //             members->member_count_);

    // RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "Allocating space");
    // // // Allocate space to store the binary representation of the message
    // uint8_t *data = new uint8_t[members->size_of_];
    // RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "Initialising members");
    // // Initialise the message buffer according to the interface type
    // members->init_function(
    //     data, rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY);

    // for (size_t i = 0; i < members->member_count_; i++) {
    //   const MessageMember member = members->members_[i];

    //   RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "type id: %d",
    //               member.type_id_);
    //   RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "name: %s",
    //   member.name_);

    //   if (member.is_array_) {
    //     RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "Array");
    //   }
    //   if (member.type_id_ == ROS_TYPE_MESSAGE) {
    //     // recursion
    //     RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "ROS TYPE MSG");
    //   }
    // }
    using rosidl_typesupport_introspection_cpp::MessageMember;
    using rosidl_typesupport_introspection_cpp::MessageMembers;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;

    std::string topic_type = "cx_msgs/PddlGenInterfaceMessages";

    auto library = rosbag2_cpp::get_typesupport_library(
        topic_type,
        rosidl_typesupport_introspection_cpp::typesupport_identifier);
    const auto *typesupport = rosbag2_cpp::get_typesupport_handle(
        topic_type,
        rosidl_typesupport_introspection_cpp::typesupport_identifier, library);

    const auto *members =
        static_cast<const MessageMembers *>(typesupport->data);

    for (size_t i = 0; i < members->member_count_; i++) {
      const MessageMember &member = members->members_[i];
      if (member.is_array_) {
        RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "It is an array, %s",
                    member.name_);
      } else if (member.type_id_ == ROS_TYPE_MESSAGE) {
        // recursion
        RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "Complex ROS type %s",
                    member.name_);
      } else {
        RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "Normal one, %s",
                    member.name_);
      }
    }

    using TypeSupport = rosidl_message_type_support_t;

    std::function<void(const TypeSupport *)> recursivelyCreateTree;
    recursivelyCreateTree = [&](const TypeSupport *type_data) {
      const auto *members =
          static_cast<const MessageMembers *>(type_data->data);

      for (size_t i = 0; i < members->member_count_; i++) {
        const MessageMember &member = members->members_[i];
        RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "It is %s",
                    member.name_);
        if (member.is_array_) {
          // new_node->children().reserve(1);
          // new_node = new_node->addChild("#");
        }
        if (member.type_id_ == ROS_TYPE_MESSAGE) {
          // recursion
          RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "Recursion");
          recursivelyCreateTree(member.members_);
        }
      }
    };
    recursivelyCreateTree(typesupport);

    // rclcpp::SerializedMessage serialized_msg_;
    auto pddl_msg = cx_msgs::msg::PddlGenInterface();
    pddl_msg.final = true;
    pddl_msg.msg_id = 1;

    // rclcpp::SerializationBase serializer{typesupport};

    // serializer.serialize_message(&pddl_msg, &serialized_msg_);

    // auto serialized_msg =
    //     RmwInterface().serialize_message(pddl_msg, typesupport);

    // Parser parser;
    // FlatMessage flat_message;
    // RenamedValues renamed;

  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }
}

// TEST(test_interface_nodes_, test_blackboard_and_clients) {
//   auto testing_node = rclcpp::Node::make_shared("testing_node");

//   auto blackboard_node = std::make_shared<cx::Blackboard>();

//   // auto msg = std::make_shared<cx_msgs::msg::PddlGenInterface>();
//   // msg->set__msg_id(1);
//   // msg->set__final(false);

//   // Executors
//   rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 1);

//   exe.add_node(blackboard_node->get_node_base_interface());

//   bool finish_exec = false;
//   std::thread t([&]() {
//     while (!finish_exec) {
//       exe.spin_some();
//     }
//   });

//   using lc_tr = lifecycle_msgs::msg::Transition;

//   blackboard_node->trigger_transition(lc_tr::TRANSITION_CONFIGURE);

//   {
//     rclcpp::Rate rate(10);
//     auto start_time = testing_node->now();
//     while ((testing_node->now() - start_time).seconds() < 0.5) {
//       rate.sleep();
//     }
//   }

//   blackboard_node->trigger_transition(lc_tr::TRANSITION_ACTIVATE);

//   {
//     rclcpp::Rate rate(10);
//     auto start_time = testing_node->now();
//     while ((testing_node->now() - start_time).seconds() < 0.5) {
//       rate.sleep();
//     }
//   }

//   RCLCPP_INFO(testing_node->get_logger(), "TEST-NODE-START");
//   try {

//     auto i_reader = std::make_shared<cx::InterfaceReader<
//         cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
//         cx_msgs::msg::PddlGenInterfaceMessages>>(
//         "ClipsExecutive", "PddlGenInterface", "pddl-gen");
//     auto i_reader2 = std::make_shared<cx::InterfaceReader<
//         cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
//         cx_msgs::msg::PddlGenInterfaceMessages>>(
//         "ClipsExecutive_2", "PddlGenInterface", "pddl-gen");
//     auto i_reader3 = std::make_shared<cx::InterfaceReader<
//         cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
//         cx_msgs::msg::PddlGenInterfaceMessages>>(
//         "ClipsExecutive_3", "PddlGenInterface", "pddl-gen");
//     auto i_reader4 = std::make_shared<cx::InterfaceReader<
//         cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
//         cx_msgs::msg::PddlGenInterfaceMessages>>(
//         "ClipsExecutive_4", "PddlGenInterface", "pddl-gen");
//     auto i_reader5 = std::make_shared<cx::InterfaceReader<
//         cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
//         cx_msgs::msg::PddlGenInterfaceMessages>>(
//         "ClipsExecutive_5", "PddlGenInterface", "pddl-gen");
//     auto i_reader6 = std::make_shared<cx::InterfaceReader<
//         cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
//         cx_msgs::msg::PddlGenInterfaceMessages>>(
//         "ClipsExecutive_6", "PddlGenInterface", "pddl-gen");
//     auto i_reader7 = std::make_shared<cx::InterfaceReader<
//         cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
//         cx_msgs::msg::PddlGenInterfaceMessages>>(
//         "ClipsExecutive_7", "PddlGenInterface", "pddl-gen");
//     auto i_reader8 = std::make_shared<cx::InterfaceReader<
//         cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
//         cx_msgs::msg::PddlGenInterfaceMessages>>(
//         "ClipsExecutive_8", "PddlGenInterface", "pddl-gen");
//     auto i_reader9 = std::make_shared<cx::InterfaceReader<
//         cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
//         cx_msgs::msg::PddlGenInterfaceMessages>>(
//         "ClipsExecutive_9", "PddlGenInterface", "pddl-gen");
//     auto i_reader10 = std::make_shared<cx::InterfaceReader<
//         cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
//         cx_msgs::msg::PddlGenInterfaceMessages>>(
//         "ClipsExecutive_10", "PddlGenInterface", "pddl-gen");

//     auto i_writer = std::make_shared<cx::InterfaceWriter<
//         cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
//         cx_msgs::msg::PddlGenInterfaceMessages>>(
//         "Pddl_Planner", "PddlGenInterface", "pddl-gen");

//     auto iface = blackboard_node->imngr_->cast_interface("PddlGenInterface")
//                      .value()
//                      ->interface_;

//     // Test that Writer/readers were set
//     ASSERT_TRUE(iface->has_writer());
//     ASSERT_TRUE(iface->is_writer("Pddl_Planner"));
//     ASSERT_EQ(iface->list_writer(), "Pddl_Planner");
//     auto r_list = iface->list_readers();
//     ASSERT_EQ(r_list.front(), "ClipsExecutive");

//     RCLCPP_INFO(testing_node->get_logger(),
//                 "Data in interface --- msg_id: %d, final: false ---",
//                 iface->interface_data_ptr_->msg_id);
//     RCLCPP_INFO(testing_node->get_logger(),
//                 "Data in interface --- msg_id: %d, final: false ---",
//                 i_reader->get_interface_data()->msg_id);
//     RCLCPP_INFO(testing_node->get_logger(),
//                 "Data in interface --- msg_id: %d, final: false ---",
//                 i_writer->get_interface_data()->msg_id);

//     auto msg = std::make_shared<cx_msgs::msg::PddlGenInterface>();
//     msg->set__msg_id(1);
//     msg->set__final(true);

//     auto msgs = std::make_shared<cx_msgs::msg::PddlGenInterfaceMessages>(
//         rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY);

//     msgs->type = "generate_message";
//     msgs->generate_message.goal = "as";

//     i_reader->msgq_enqueue(msgs);

//     if (i_writer->i_msgq_.front().type == "generate_message") {
//       RCLCPP_INFO(testing_node->get_logger(), "Yes: Gen M 1");
//     }
//     if (i_writer->i_msgq_.front().type == "generate_message_2") {
//       RCLCPP_INFO(testing_node->get_logger(), "Yes: Gen M 2");
//     }
//     // if (srv->messages) {
//     //   RCLCPP_INFO(testing_node->get_logger(), "Yes: Gen M 2");
//     // }

//     // for (int i = 0; i < 1000; i++) {
//     //   RCLCPP_INFO(
//     //       testing_node->get_logger(),
//     //       "------------------------------------------------------------");

//     //   msg->set__msg_id(i % 2);
//     //   i_writer->write_in_interface(msg);
//     //   i_reader->read_from_interface();
//     //   i_reader2->read_from_interface();
//     //   i_reader3->read_from_interface();
//     //   i_reader4->read_from_interface();
//     //   i_reader5->read_from_interface();
//     //   i_reader6->read_from_interface();
//     //   i_reader7->read_from_interface();
//     //   i_reader8->read_from_interface();
//     //   i_reader9->read_from_interface();
//     //   RCLCPP_INFO(testing_node->get_logger(),
//     //               "Data in interface --- msg_id: %d, final: true ---",
//     //               iface->interface_data_ptr_->msg_id);
//     //   RCLCPP_INFO(testing_node->get_logger(),
//     //               "Data in interface --- msg_id: %d, final: true ---",
//     //               i_reader->get_interface_data()->msg_id);
//     //   RCLCPP_INFO(testing_node->get_logger(),
//     //               "Data in interface --- msg_id: %d, final: true ---",
//     //               i_writer->get_interface_data()->msg_id);
//     // }

//     // i_writer->write_in_interface(msg);

//     // i_reader->read_from_interface();
//     // i_reader->read_from_interface();
//     // i_reader->read_from_interface();

//     RCLCPP_INFO(testing_node->get_logger(),
//                 "Data in interface --- msg_id: %d, final: true ---",
//                 iface->interface_data_ptr_->msg_id);
//     RCLCPP_INFO(testing_node->get_logger(),
//                 "Data in interface --- msg_id: %d, final: true ---",
//                 i_reader->get_interface_data()->msg_id);
//     RCLCPP_INFO(testing_node->get_logger(),
//                 "Data in interface --- msg_id: %d, final: true ---",
//                 i_writer->get_interface_data()->msg_id);

//   } catch (const std::exception &e) {
//     std::cerr << e.what() << '\n';
//   }

//   RCLCPP_INFO(testing_node->get_logger(), "TEST-NODE-END");

//   finish_exec = true;
//   t.join();
// }
TEST(test_interface_nodes_, test_ros2_intro_with_reader) {
  auto testing_node = rclcpp::Node::make_shared("testing_node");
  auto blackboard_node = std::make_shared<cx::Blackboard>();
  // Executors
  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 1);

  exe.add_node(blackboard_node->get_node_base_interface());

  bool finish_exec = false;
  std::thread t([&]() {
    while (!finish_exec) {
      exe.spin_some();
    }
  });

  using lc_tr = lifecycle_msgs::msg::Transition;

  blackboard_node->trigger_transition(lc_tr::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start_time = testing_node->now();
    while ((testing_node->now() - start_time).seconds() < 0.5) {
      rate.sleep();
    }
  }

  blackboard_node->trigger_transition(lc_tr::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start_time = testing_node->now();
    while ((testing_node->now() - start_time).seconds() < 0.5) {
      rate.sleep();
    }
  }

  RCLCPP_INFO(testing_node->get_logger(), "TEST-NODE-START");
  try {

    std::vector<std::shared_ptr<void>> any_reader;

    auto i_reader = std::make_shared<cx::InterfaceReader<
        cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
        cx_msgs::msg::PddlGenInterfaceMessages>>(
        "ClipsExecutive", "PddlGenInterface", "pddl-gen");

    // auto serialized_msg =
    //     RmwInterface().serialize_message(pddl_msg, typesupport);

    // auto serialized_msg = rmw_get_zero_initialized_serialized_message();
    // auto serializer = rclcpp::SerializationBase(typesupport);
    // serializer.serialize_message(&pddl_msg, &serialized_msg);

    // Parser parser;
    // FlatMessage flat_message;
    // RenamedValues renamed;

    // parser.registerMessageType("PddlGenInterface", topic_type);

    // parser.deserializeIntoFlatMessage("PddlGenInterface", &serialized_msg,
    //                                   &flat_message, 100);

    // ConvertFlatMessageToRenamedValues(flat_message, renamed);

    // for (const auto &pair : renamed) {
    //   std::cout << pair.first << " = " << pair.second << std::endl;
    // }

    any_reader.push_back(i_reader);

    auto reader_casted = std::static_pointer_cast<cx::InterfaceReader<
        cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface,
        cx_msgs::msg::PddlGenInterfaceMessages>>(any_reader.front());

  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }

  RCLCPP_INFO(testing_node->get_logger(), "TEST-NODE-END");

  finish_exec = true;
  t.join();
}

TEST(test_interface_nodes, test_context_init) {
  try {

    auto blackboard = cx::Blackboard();
    blackboard.imngr_ = std::make_unique<cx::InterfaceManager>();
    blackboard.imngr_->generate_interface_node("PddlGenInterface",
                                               "PddlGenInterface", "pddl-gen");
    auto pddl_iface =
        blackboard.imngr_->cast_interface("PddlGenInterface").value();

    auto wrong_iface =
        blackboard.imngr_->cast_interface("WrongInterface").value();

    // RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "Type: %s",
    //             reader_casted->i_type_.c_str());
    // RCLCPP_INFO(rclcpp::get_logger("Type_Support"), "Type: %s",
    //             reader_casted->i_id_.c_str());

    pddl_iface->interface_->interface_data_ptr_->set__msg_id(2);

    ASSERT_EQ(pddl_iface->interface_->interface_data_ptr_->msg_id, 2);
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
