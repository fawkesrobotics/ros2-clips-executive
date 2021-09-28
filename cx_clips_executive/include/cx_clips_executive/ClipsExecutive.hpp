#ifndef CX_CLIPS_EXECUTIVE__CLIPSEXECUTIVE_HPP_
#define CX_CLIPS_EXECUTIVE__CLIPSEXECUTIVE_HPP_

#include <clipsmm.h>
#include <map>
#include <memory>
#include <string>
#include <chrono>

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/empty.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "cx_clips/CLIPSEnvManagerClient.hpp"
#include "cx_clips/CLIPSEnvManagerNode.h"

#include "cx_utils/LockSharedPtr.hpp"
#include "cx_utils/map_skill.h"

namespace cx {

class ClipsExecutive : public rclcpp_lifecycle::LifecycleNode {

public:
  ClipsExecutive();
  // ~ClipsExecutive();
  void pre_configure(std::shared_ptr<cx::CLIPSEnvManagerNode> &manager_node);

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
  // CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
  // CallbackReturn on_error(const rclcpp_lifecycle::State &state);

  // TODO: MOVE TO PRIVATE LATER
  std::shared_ptr<cx::CLIPSEnvManagerNode> clips_env_manager_node_;
  LockSharedPtr<CLIPS::Environment> clips_;

private:
  std::string clips_map_skill(std::string action_name,
                              CLIPS::Values param_names,
                              CLIPS::Values param_values);
  void iterateThroughYamlRecuresively(
      const YAML::Node &current_level_node, const std::string &node_to_search,
      const std::string &parent_node_name, const std::string &cfg_prefix,
      std::map<std::string, std::string> &output_map);

private:
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr
      clips_agenda_refresh_pub_;
  rclcpp::TimerBase::SharedPtr agenda_refresh_timer_;

  std::shared_ptr<cx::CLIPSEnvManagerClient> env_manager_client_;
  std::vector<std::string> clips_dirs{};
  std::string clips_executive_share_dir_;
  bool cfg_assert_time_each_cycle_;
  int refresh_rate_;
  std::chrono::nanoseconds publish_rate_;
  std::shared_ptr<cx::ActionSkillMapping> action_skill_mapping_;
};
} // namespace cx
#endif // !CX_CLIPS_EXECUTIVE__CLIPSEXECUTIVE_HPP_
