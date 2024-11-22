// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#ifndef CX_PLUGINS__FILELOADPLUGIN_HPP_
#define CX_PLUGINS__FILELOADPLUGIN_HPP_

#include <string>

#include "cx_plugin/clips_plugin.hpp"
#include "cx_utils/lock_shared_ptr.hpp"

namespace cx {

class FileLoadPlugin : public ClipsPlugin {
public:
  FileLoadPlugin();
  ~FileLoadPlugin();

  void initialize();

  bool clips_env_init(LockSharedPtr<clips::Environment> &env) override;
  bool clips_env_destroyed(LockSharedPtr<clips::Environment> &env) override;

private:
  std::unique_ptr<rclcpp::Logger> logger_;

  std::vector<std::string> init_files_;
  std::vector<std::string> init_batch_files_;
  std::vector<std::string> cleanup_files_;
};
} // namespace cx

#endif // !CX_PLUGINS__FILELOADPLUGIN_HPP_
