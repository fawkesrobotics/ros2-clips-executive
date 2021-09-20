#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cx_example_skill_nodes/MoveSkillNav2.hpp"

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
using namespace std::placeholders;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using NavGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
using NavFeedback =
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

MoveSkillNav2::MoveSkillNav2(const std::string &id,
                             const std::string &action_name,
                             const std::chrono::nanoseconds &rate)
    : SkillExecution(id, action_name, rate) {
  geometry_msgs::msg::PoseStamped wp;
  wp.header.frame_id = "map";
  wp.header.stamp = now();
  wp.pose.position.x = 0.0;
  wp.pose.position.y = -2.0;
  wp.pose.position.z = 0.0;
  wp.pose.orientation.x = 0.0;
  wp.pose.orientation.y = 0.0;
  wp.pose.orientation.z = 0.0;
  wp.pose.orientation.w = 1.0;
  waypoints_map_["wp1"] = wp;

  wp.pose.position.x = 1.8;
  wp.pose.position.y = 0.0;
  waypoints_map_["wp2"] = wp;

  wp.pose.position.x = 0.0;
  wp.pose.position.y = 2.0;
  waypoints_map_["wp3"] = wp;

  wp.pose.position.x = -0.5;
  wp.pose.position.y = -0.5;
  waypoints_map_["wp4"] = wp;

  wp.pose.position.x = -2.0;
  wp.pose.position.y = -0.4;
  waypoints_map_["wp_init"] = wp;

  pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      std::bind(&MoveSkillNav2::current_pos_callback, this, _1));
}

void MoveSkillNav2::current_pos_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  current_pos_ = msg->pose.pose;
}

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
MoveSkillNav2::on_activate(const rclcpp_lifecycle::State &state) {
  std::cerr << "MoveSkillNav2::on_activate" << std::endl;
  send_feedback(0.0, "Move starting");

  navigation_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          shared_from_this(), "navigate_to_pose");

  bool is_action_server_ready = false;
  RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

  do {
    // RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

    is_action_server_ready = navigation_action_client_->wait_for_action_server(
        std::chrono::seconds(5));
  } while (!is_action_server_ready);

  RCLCPP_INFO(get_logger(), "Navigation action server ready");

  auto wp_to_navigate =
      action_parameters_[3]; // The goal is in the 3rd argument of the action
  RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

  goal_pos_ = waypoints_map_[wp_to_navigate];
  navigation_goal_.pose = goal_pos_;

  dist_to_move = getDistance(goal_pos_.pose, current_pos_);

  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.feedback_callback = [this](NavGoalHandle::SharedPtr,
                                               NavFeedback feedback) {
    send_feedback(
        std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining /
                                           dist_to_move))),
        "Move running");
  };

  send_goal_options.result_callback = [this](auto) {
    finish_execution(true, 1.0, "Move completed");
  };

  future_navigation_goal_handle_ = navigation_action_client_->async_send_goal(
      navigation_goal_, send_goal_options);

  return SkillExecution::on_activate(state);
}

void MoveSkillNav2::perform_execution() {
  RCLCPP_INFO_STREAM(get_logger(), "Executing [" << action_name_ << "]");
  for (const auto &param : action_parameters_) {
    RCLCPP_INFO_STREAM(get_logger(), "\t[" << param << "]");
  }
}

double MoveSkillNav2::getDistance(const geometry_msgs::msg::Pose &pos1,
                                  const geometry_msgs::msg::Pose &pos2) {
  return sqrt((pos1.position.x - pos2.position.x) *
                  (pos1.position.x - pos2.position.x) +
              (pos1.position.y - pos2.position.y) *
                  (pos1.position.y - pos2.position.y));
}

} // namespace cx