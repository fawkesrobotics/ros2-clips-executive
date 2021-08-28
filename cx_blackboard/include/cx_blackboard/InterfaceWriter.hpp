#ifndef CX_BLACKBOARD__INTERFACEWRITER_HPP_
#define CX_BLACKBOARD__INTERFACEWRITER_HPP_

#include <list>
#include <memory>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include <rosidl_runtime_c/message_initialization.h>

#include "cx_msgs/srv/open_interface.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace cx {

template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
class InterfaceWriter {
public:
  InterfaceWriter(const std::string &node_name, const std::string &i_type,
                  const std::string &i_id);

  bool open_interface_for_writing();
  bool write_in_interface(typename InterfaceMessageType::SharedPtr msg);
  // ~InterfaceWriter();
  std::shared_ptr<InterfaceMessageType> get_interface_data();

  void msgq_enqueue_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<typename InterfaceServiceType::Request> request,
      const std::shared_ptr<typename InterfaceServiceType::Response> response);
  // MOVE TO PRIVATE
  std::list<InterfaceMessages> i_msgq_;

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<cx::NodeThread> thread_;

  typename rclcpp::Client<cx_msgs::srv::OpenInterface>::SharedPtr
      open_interface_client_;

  typename rclcpp::Client<InterfaceServiceType>::SharedPtr
      write_in_interface_client_;

  typename rclcpp::Service<InterfaceServiceType>::SharedPtr
      msgq_enqueue_service_;

  std::string i_type_;
  std::string i_id_;
  InterfaceMessageType i_data_{};
};

template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
InterfaceWriter<InterfaceServiceType, InterfaceMessageType, InterfaceMessages>::
    InterfaceWriter(const std::string &node_name, const std::string &i_type,
                    const std::string &i_id)
    : node_{rclcpp::Node::make_shared(node_name)}, i_type_{i_type}, i_id_{
                                                                        i_id} {
  RCLCPP_INFO(node_->get_logger(), "Initialising [%s]...", node_->get_name());

  open_interface_client_ = node_->create_client<cx_msgs::srv::OpenInterface>(
      "blackboard/open_interface");

  RCLCPP_INFO(node_->get_logger(), "Registering %s as writer for %s",
              node_->get_name(), i_type_.c_str());

  if (!open_interface_for_writing()) {
    throw std::runtime_error("Couldn't register writer! Exception follows...");
  }

  write_in_interface_client_ =
      node_->create_client<InterfaceServiceType>(i_type_ + "_write");

  msgq_enqueue_service_ = node_->create_service<InterfaceServiceType>(
      i_type_ + "_msgq_enqueue",
      std::bind(&InterfaceWriter<InterfaceServiceType, InterfaceMessageType,
                                 InterfaceMessages>::msgq_enqueue_callback,
                this, _1, _2, _3));

  thread_ = std::make_unique<cx::NodeThread>(node_->get_node_base_interface());

  RCLCPP_INFO(node_->get_logger(), "Initialised [%s]!", node_->get_name());
}

template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
bool InterfaceWriter<InterfaceServiceType, InterfaceMessageType,
                     InterfaceMessages>::open_interface_for_writing() {

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
  // Writer
  req->writing = true;
  req->i_owner = node_->get_name();

  auto future_res = open_interface_client_->async_send_request(req);

  if (rclcpp::spin_until_future_complete(node_, future_res, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "%s: timed out waiting for response!",
                 open_interface_client_->get_service_name());
    return false;
  }

  if (future_res.get()->success) {
    RCLCPP_INFO(node_->get_logger(),
                "Successfully registered writer %s for interface %s!",
                node_->get_name(), i_type_.c_str());
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "Couldn't register writer %s for interface %s!",
                 node_->get_name(), i_type_.c_str());
    return false;
  }
}

template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
bool InterfaceWriter<InterfaceServiceType, InterfaceMessageType,
                     InterfaceMessages>::
    write_in_interface(typename InterfaceMessageType::SharedPtr msg) {

  while (!write_in_interface_client_->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   write_in_interface_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                write_in_interface_client_->get_service_name());
  }

  auto req = std::make_shared<typename InterfaceServiceType::Request>(
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY);
  req->set__interface(*msg);
  auto future_res = write_in_interface_client_->async_send_request(req);
  auto future_status = wait_for_future_result(future_res, 5s);

  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "%s: timed out waiting for response!",
                 write_in_interface_client_->get_service_name());
    return false;
  }

  if (future_res.get()->success) {
    // Not sure if needed
    i_data_ = *msg;

    RCLCPP_INFO(node_->get_logger(), "Successfully written to interface %s!",
                i_type_.c_str());
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Couldn't write to interface %s!",
                 i_type_.c_str());
    return false;
  }
}

template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
void InterfaceWriter<InterfaceServiceType, InterfaceMessageType,
                     InterfaceMessages>::
    msgq_enqueue_callback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<typename InterfaceServiceType::Request> request,
        const std::shared_ptr<typename InterfaceServiceType::Response>
            response) {
  RCLCPP_INFO(node_->get_logger(), "In Msgq service!");
  response->success = false;

  try {
    i_msgq_.push_back(request->messages);
    response->success = true;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }

  RCLCPP_INFO(node_->get_logger(), "End of Msgq service!");
}

template <class InterfaceServiceType, class InterfaceMessageType,
          class InterfaceMessages>
std::shared_ptr<InterfaceMessageType>
InterfaceWriter<InterfaceServiceType, InterfaceMessageType,
                InterfaceMessages>::get_interface_data() {
  return std::make_shared<InterfaceMessageType>(i_data_);
}

} // namespace cx

#endif // !CX_BLACKBOARD__INTERFACEWRITER_HPP_
