#ifndef CX_EXAMPLE_SKILL_NODES__MOVESKILLNAV2_HPP
#define CX_EXAMPLE_SKILL_NODES__MOVESKILLNAV2_HPP

#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "cx_skill_execution/SkillExecution.hpp"

#include "cx_msgs/msg/skill_action_execinfo.hpp"
#include "cx_msgs/msg/skill_execution.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace cx {
using namespace std::chrono_literals;

class MoveSkillNav2 : public cx::SkillExecution {

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using NavGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using NavFeedback =
      const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

public:

  MoveSkillNav2(const std::string &id, const std::string &action_name,
                const std::chrono::nanoseconds &rate);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  void perform_execution() override;

private:
  double getDistance(const geometry_msgs::msg::Pose &pos1,
                     const geometry_msgs::msg::Pose &pos2);
  void current_pos_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

private:
  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_map_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      navigation_action_client_;
  std::shared_future<NavGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavGoalHandle::SharedPtr navigation_goal_handle_;

  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  double dist_to_move;
};
} // namespace cx

#endif // !CX_EXAMPLE_SKILL_NODES__MOVESKILLNAV2_HPP
