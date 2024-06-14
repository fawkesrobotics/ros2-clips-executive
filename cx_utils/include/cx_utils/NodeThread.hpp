// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#ifndef CX_UTILS__NODETHREAD_HPP_
#define CX_UTILS__NODETHREAD_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace cx {

class NodeThread {

public:
  // Takes the base interface of a node and spins it in the single executor
  // thread
  explicit NodeThread(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_interface);
  // Thread to process all node callbacks with a given executor
  explicit NodeThread(
      rclcpp::executors::SingleThreadedExecutor::SharedPtr executor);

  ~NodeThread();

protected:
  std::unique_ptr<std::thread> thread_;
  rclcpp::Executor::SharedPtr executor_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
};

} // namespace cx

#endif // !CX_UTILS__NODETHREAD_HPP_
