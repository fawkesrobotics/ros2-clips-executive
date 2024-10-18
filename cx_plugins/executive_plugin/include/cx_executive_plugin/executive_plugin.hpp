// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#ifndef CX_PLUGINS__EXECUTIVEPLUGIN_HPP_
#define CX_PLUGINS__EXECUTIVEPLUGIN_HPP_

#include <string>

#include "cx_plugin/clips_plugin.hpp"
#include "cx_utils/LockSharedPtr.hpp"

#include <std_msgs/msg/empty.hpp>

namespace cx {

class ExecutivePlugin : public ClipsPlugin {
public:
  ExecutivePlugin();
  ~ExecutivePlugin();

  void initialize() override;

  bool clips_env_init(LockSharedPtr<clips::Environment> &clips) override;
  bool clips_env_destroyed(LockSharedPtr<clips::Environment> &clips) override;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node();

private:
  int refresh_rate_;
  bool assert_time_;
  bool publish_on_refresh_;

  std::string plugin_path_;

  rclcpp::TimerBase::SharedPtr agenda_refresh_timer_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr
      clips_agenda_refresh_pub_;

  std::chrono::nanoseconds publish_rate_;

  std::unique_ptr<rclcpp::Logger> logger_;

  std::vector<LockSharedPtr<clips::Environment>> managed_envs;
  std::mutex envs_mutex_;
};
} // namespace cx

#endif // !CX_PLUGINS__EXECUTIVEPLUGIN_HPP_
