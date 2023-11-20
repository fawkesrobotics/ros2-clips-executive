/***************************************************************************
 *  InterfaceNode.hpp
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

#ifndef CX_UTILS__INTERFACENODE_HPP_
#define CX_UTILS__INTERFACENODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "cx_blackboard/Interface.hpp"

namespace cx {

template <class InterfaceServiceType, class InterfaceMessageType>
class InterfaceNode : public rclcpp::Node {
public:
  InterfaceNode(const std::string &node_name,
                std::shared_ptr<Interface<InterfaceMessageType>> interface);
  // InterfaceServiceType &service_msg);
  // ~InterfaceNode();

  void write_in_interface_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<typename InterfaceServiceType::Request> request,
      const std::shared_ptr<typename InterfaceServiceType::Response> response);

  void read_interface_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<typename InterfaceServiceType::Request> request,
      const std::shared_ptr<typename InterfaceServiceType::Response> response);

  //   void msgq_enqueue_callback(
  //       const std::shared_ptr<rmw_request_id_t> request_header,
  //       const std::shared_ptr<typename InterfaceServiceType::Request>
  //       request, const std::shared_ptr<typename
  //       InterfaceServiceType::Response> response);

  // MOVE TO PRIVATE
  std::shared_ptr<Interface<InterfaceMessageType>> interface_;

private:
  typename rclcpp::Service<InterfaceServiceType>::SharedPtr
      write_in_interface_service_;

  typename rclcpp::Service<InterfaceServiceType>::SharedPtr
      read_interface_service_;

  // typename rclcpp::Service<InterfaceServiceType>::SharedPtr
  //     msgq_enqueue_service_;

  typename rclcpp::Publisher<InterfaceMessageType>::SharedPtr
      notify_readers_pub_;
};

using namespace std::placeholders;

template <class InterfaceServiceType, class InterfaceMessageType>
InterfaceNode<InterfaceServiceType, InterfaceMessageType>::InterfaceNode(
    const std::string &node_name,
    std::shared_ptr<Interface<InterfaceMessageType>> interface)
    // InterfaceServiceType &service_msg)
    : rclcpp::Node(node_name), interface_(interface) {
  // , service_msg_{service_msg} {

  RCLCPP_INFO(get_logger(), "Initialising [%s]...", get_name());

  write_in_interface_service_ = create_service<InterfaceServiceType>(
      node_name + "_write",
      std::bind(
          &InterfaceNode<InterfaceServiceType,
                         InterfaceMessageType>::write_in_interface_callback,
          this, _1, _2, _3));

  read_interface_service_ = create_service<InterfaceServiceType>(
      node_name + "_read",
      std::bind(&InterfaceNode<InterfaceServiceType,
                               InterfaceMessageType>::read_interface_callback,
                this, _1, _2, _3));

  // msgq_enqueue_service_ = create_service<InterfaceServiceType>(
  //     node_name + "msgq_enqueue",
  //     std::bind(&InterfaceNode<InterfaceServiceType,
  //                              InterfaceMessageType>::msgq_enqueue_callback,
  //               this, _1, _2, _3));

  notify_readers_pub_ = create_publisher<InterfaceMessageType>(
      node_name + "_notify_readers", rclcpp::QoS(100).reliable());

  RCLCPP_INFO(get_logger(), "Initialised [%s]!", get_name());
}

template <class InterfaceServiceType, class InterfaceMessageType>
void InterfaceNode<InterfaceServiceType, InterfaceMessageType>::
    write_in_interface_callback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<typename InterfaceServiceType::Request> request,
        const std::shared_ptr<typename InterfaceServiceType::Response>
            response) {
  (void)request_header;
  RCLCPP_INFO(get_logger(), "Write in interface callback");
  auto interface_pointer =
      std::make_shared<InterfaceMessageType>(request->interface);
  interface_->interface_data_ptr_ = interface_pointer;
  response->success = true;
  response->interface = *(interface_->interface_data_ptr_);

  // Update Readers through the topic
  notify_readers_pub_->publish(response->interface);

  RCLCPP_INFO(get_logger(), "End of write in interface callback");
}

template <class InterfaceServiceType, class InterfaceMessageType>
void InterfaceNode<InterfaceServiceType, InterfaceMessageType>::
    read_interface_callback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<typename InterfaceServiceType::Request> request,
        const std::shared_ptr<typename InterfaceServiceType::Response>
            response) {
  (void)request_header;
  (void)request;
  RCLCPP_INFO(get_logger(), "Reading from interface");
  response->interface = *interface_->interface_data_ptr_;
  response->success = true;
  RCLCPP_INFO(get_logger(), "End of reading from interface");
}

// template <class InterfaceServiceType, class InterfaceMessageType>
// void InterfaceNode<InterfaceServiceType, InterfaceMessageType>::
//     msgq_enqueue_callback(
//         const std::shared_ptr<rmw_request_id_t> request_header,
//         const std::shared_ptr<typename InterfaceServiceType::Request>
//         request, const std::shared_ptr<typename
//         InterfaceServiceType::Response>
//             response) {
//   RCLCPP_INFO(get_logger(), "Enqueuing message...");

//   RCLCPP_INFO(get_logger(), "Enqueued message!");
// }

} // namespace cx

#endif // !CX_UTILS__INTERFACENODE_HPP_