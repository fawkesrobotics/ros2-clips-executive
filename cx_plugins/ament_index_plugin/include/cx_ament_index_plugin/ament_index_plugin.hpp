// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#ifndef CX_PLUGINS__AMENT_INDEX_PLUGIN_HPP_
#define CX_PLUGINS__AMENT_INDEX_PLUGIN_HPP_

#include <string>

#include "cx_plugin/clips_plugin.hpp"
#include "cx_utils/LockSharedPtr.hpp"

namespace cx {

class AmentIndexPlugin : public ClipsPlugin {
public:
  AmentIndexPlugin();
  ~AmentIndexPlugin();

  void initialize();

  bool clips_env_init(LockSharedPtr<clips::Environment> &env) override;
  bool clips_env_destroyed(LockSharedPtr<clips::Environment> &env) override;

private:
  std::unique_ptr<rclcpp::Logger> logger_;
  std::unordered_set<std::string> function_names_;
};
} // namespace cx

#endif // !CX_PLUGINS__AMENT_INDEX_PLUGIN_HPP_
