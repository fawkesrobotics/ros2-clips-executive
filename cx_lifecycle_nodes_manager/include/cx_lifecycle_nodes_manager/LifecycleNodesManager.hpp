#ifndef CX_LIFECYCLE_NODES_MANAGER__LIFECYCLENODESMANAGER_HPP_
#define CX_LIFECYCLE_NODES_MANAGER__LIFECYCLENODESMANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "cx_lifecycle_nodes_manager/LifecycleNodesClient.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace cx {

class LifecycleNodesManager : public rclcpp::Node {

public:
  LifecycleNodesManager();
  ~LifecycleNodesManager();

  void initialise();
  bool startupScript();

private:
  // Node manipulation
  bool changeNodeState(const std::string &node_name, std::uint8_t transition);
  bool changeAllNodeStates(std::uint8_t transition);
  void disableAllNodes();

private:
  std::map<std::string, std::shared_ptr<cx::LifecycleNodesClient>>
      nodes_to_manage_;
  std::vector<std::string> node_names_to_manage_{"clips_manager",
                                                 "clips_features_manager"};
};

} // namespace cx

#endif // !CX_LIFECYCLE_NODES_MANAGER__LIFECYCLENODESMANAGER_HPP_
