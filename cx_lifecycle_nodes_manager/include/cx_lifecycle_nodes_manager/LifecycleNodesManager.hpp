// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  LifecycleNodesManager.hpp
 *
 *  Created: 28 June 2021
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
