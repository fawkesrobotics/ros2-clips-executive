// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include <memory>

#include "cx_utils/NodeThread.hpp"

namespace cx {

NodeThread::NodeThread(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_interface)
    : node_(node_interface) {

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  thread_ = std::make_unique<std::thread>([&]() {
    executor_->add_node(node_);
    executor_->spin();
    executor_->remove_node(node_);
  });
}

NodeThread::~NodeThread() {
  executor_->cancel();
  thread_->join();
}

} // namespace cx
