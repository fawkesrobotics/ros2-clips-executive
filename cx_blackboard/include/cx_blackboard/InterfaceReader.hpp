#ifndef CX_BLACKBOARD__INTERFACEREADER_HPP_
#define CX_BLACKBOARD__INTERFACEREADER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <rosidl_runtime_c/message_initialization.h>

#include "cx_utils/NodeThread.hpp"
#include "cx_utils/WaitService.hpp"

#include "cx_msgs/srv/open_interface.hpp"

using namespace std::chrono_literals;

namespace cx {

template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
class InterfaceReader {
public:
  InterfaceReader(const std::string &node_name, const std::string &i_type,
                  const std::string &i_id);

  bool open_interface_for_reading();
  bool read_from_interface();
  bool msgq_enqueue(typename InterfaceMessages::SharedPtr msg);

  virtual void
  update_readers_callback(typename InterfaceMessageType::SharedPtr msg);

  std::shared_ptr<InterfaceMessages>
  create_interface_message(const std::string &msg_type);

  void *create_void_message(std::shared_ptr<InterfaceMessages> msg);

  std::shared_ptr<InterfaceMessages> cast_void_message(void *message);
  // ~InterfaceReader();

  std::shared_ptr<InterfaceMessageType> get_interface_data();

protected:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<cx::NodeThread> thread_;

  typename rclcpp::Client<cx_msgs::srv::OpenInterface>::SharedPtr
      open_interface_client_;

  typename rclcpp::Client<InterfaceServiceType>::SharedPtr
      read_interface_client_;

  typename rclcpp::Client<InterfaceServiceType>::SharedPtr msgq_enqueue_client_;

  typename rclcpp::Subscription<InterfaceMessageType>::SharedPtr
      notify_readers_sub_;

  typename rclcpp::Subscription<InterfaceMessageType>::SharedPtr
      msgq_enqueue_pub_;

  const std::string i_type_;
  const std::string i_id_;
  InterfaceMessageType i_data_{};
  InterfaceMessages i_msg_{};

  // rclcpp::CallbackGroup::SharedPtr cb_grp_clt;
};

template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
InterfaceReader<InterfaceServiceType, InterfaceMessageType, InterfaceMessages>::
    InterfaceReader(const std::string &node_name, const std::string &i_type,
                    const std::string &i_id)
    : node_{rclcpp::Node::make_shared(node_name)}, i_type_{i_type}, i_id_{
                                                                        i_id} {
  RCLCPP_INFO(node_->get_logger(), "Initialising [%s]...", node_->get_name());

  // cb_grp_clt = node_->create_callback_group(
  //     rclcpp::CallbackGroupType::MutuallyExclusive);

  open_interface_client_ = node_->create_client<cx_msgs::srv::OpenInterface>(
      "blackboard/open_interface");

  RCLCPP_INFO(node_->get_logger(), "Registering %s as reader for %s",
              node_->get_name(), i_type_.c_str());
  if (!open_interface_for_reading()) {
    throw std::runtime_error("Couldn't register reader! Exception follows...");
  }

  read_interface_client_ =
      node_->create_client<InterfaceServiceType>(i_type_ + "_read");

  msgq_enqueue_client_ =
      node_->create_client<InterfaceServiceType>(i_type_ + "_msgq_enqueue");

  notify_readers_sub_ = node_->create_subscription<InterfaceMessageType>(
      i_type_ + "_notify_readers", rclcpp::QoS(100).reliable(),
      std::bind(&InterfaceReader<InterfaceServiceType, InterfaceMessageType,
                                 InterfaceMessages>::update_readers_callback,
                this, _1));

  // msgq_enqueue_pub_ =
  // node_->create_publisher<InterfaceServiceType::Request::>(
  //     i_type_ + "_msgq_enqueue", rclcpp::QoS(100).reliable());
  thread_ = std::make_unique<cx::NodeThread>(node_->get_node_base_interface());
  RCLCPP_INFO(node_->get_logger(), "Initialised [%s]!", node_->get_name());
}

template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
bool InterfaceReader<InterfaceServiceType, InterfaceMessageType,
                     InterfaceMessages>::open_interface_for_reading() {

  RCLCPP_INFO(node_->get_logger(), "Opening for reading!");

  while (!open_interface_client_->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   open_interface_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                open_interface_client_->get_service_name());
  }
  auto req = std::make_shared<cx_msgs::srv::OpenInterface::Request>();
  req->i_type = i_type_;
  req->i_id = i_id_;
  // Reader
  req->writing = false;
  req->i_owner = node_->get_name();

  auto future_res = open_interface_client_->async_send_request(req);

  if (rclcpp::spin_until_future_complete(node_, future_res, 10s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "%s: timed out waiting for response!",
                 open_interface_client_->get_service_name());
    return false;
  }

  if (future_res.get()->success) {
    RCLCPP_INFO(node_->get_logger(),
                "Successfully registered reader %s for interface %s!",
                node_->get_name(), i_type_.c_str());
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "Couldn't register reader %s for interface %s!",
                 node_->get_name(), i_type_.c_str());
    return false;
  }
}

template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
void InterfaceReader<InterfaceServiceType, InterfaceMessageType,
                     InterfaceMessages>::
    update_readers_callback(typename InterfaceMessageType::SharedPtr msg) {
  RCLCPP_INFO(node_->get_logger(), "Received updated msg, [%s]...",
              node_->get_name());

  // TODO: FIX LATER!
  i_data_ = *msg;

  RCLCPP_INFO(node_->get_logger(), "Updated the msg, [%s]...",
              node_->get_name());
}

template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
bool InterfaceReader<InterfaceServiceType, InterfaceMessageType,
                     InterfaceMessages>::read_from_interface() {
  RCLCPP_INFO(node_->get_logger(), "Reading from interface %s...",
              i_type_.c_str());

  while (!read_interface_client_->wait_for_service(2s)) {
    if (!rclcpp::ok()) {

      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   read_interface_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                read_interface_client_->get_service_name());
  }

  auto req = std::make_shared<typename InterfaceServiceType::Request>(
      rosidl_runtime_cpp::MessageInitialization::SKIP);
  auto future_res = read_interface_client_->async_send_request(req);
  // Make call and wait for response!
  auto future_status = wait_for_future_result(future_res, 3s);

  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "%s: timed out waiting for response!",
                 read_interface_client_->get_service_name());
    return false;
  }

  if (future_res.get()->success) {
    RCLCPP_INFO(node_->get_logger(), "Successfully read from interface %s!",
                i_type_.c_str());
    i_data_ = future_res.get()->interface;
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Couldn't  read from interface %s!",
                 i_type_.c_str());
    return false;
  }
}
template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
bool InterfaceReader<InterfaceServiceType, InterfaceMessageType,
                     InterfaceMessages>::
    msgq_enqueue(typename InterfaceMessages::SharedPtr msg) {

  RCLCPP_INFO(node_->get_logger(), "Queuing msg of type %s...",
              msg->type.c_str());

  while (!msgq_enqueue_client_->wait_for_service(2s)) {
    if (!rclcpp::ok()) {

      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   msgq_enqueue_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                msgq_enqueue_client_->get_service_name());
  }

  auto req = std::make_shared<typename InterfaceServiceType::Request>(
      rosidl_runtime_cpp::MessageInitialization::SKIP);
  req->messages = *msg;
  auto future_res = msgq_enqueue_client_->async_send_request(req);
  auto future_status = wait_for_future_result(future_res, 3s);

  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "%s: timed out waiting for response!",
                 msgq_enqueue_client_->get_service_name());
    return false;
  }

  if (future_res.get()->success) {
    RCLCPP_INFO(node_->get_logger(), "Successfully queued msg!");
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Couldn't queue msg!");
    return false;
  }
}

template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
std::shared_ptr<InterfaceMessageType>
InterfaceReader<InterfaceServiceType, InterfaceMessageType,
                InterfaceMessages>::get_interface_data() {
  return std::make_shared<InterfaceMessageType>(i_data_);
}

// ------------------------------------------------------
// HELPER FUNCTION FOR CX_BLACKBOARD

template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
std::shared_ptr<InterfaceMessages>
InterfaceReader<InterfaceServiceType, InterfaceMessageType, InterfaceMessages>::
    create_interface_message(const std::string &msg_type) {
  auto msg = std::make_shared<InterfaceMessages>(
      rosidl_runtime_cpp::MessageInitialization::SKIP);
  msg->type = msg_type;
  return msg;
}

template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
void *
InterfaceReader<InterfaceServiceType, InterfaceMessageType, InterfaceMessages>::
    create_void_message(std::shared_ptr<InterfaceMessages> msg) {
  return new std::shared_ptr<InterfaceMessages>(msg);
}
template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
std::shared_ptr<InterfaceMessages>
InterfaceReader<InterfaceServiceType, InterfaceMessageType,
                InterfaceMessages>::cast_void_message(void *message) {
  std::shared_ptr<InterfaceMessages> *msg =
      static_cast<std::shared_ptr<InterfaceMessages> *>(message);
  return msg;
}

// ------------------------------------------------------

} // namespace cx

#endif // !CX_BLACKBOARD__INTERFACEREADER_HPP_