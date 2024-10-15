// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#ifndef CX_FEATURES__FILELOADFEATURE_HPP_
#define CX_FEATURES__FILELOADFEATURE_HPP_

#include <string>

#include "cx_feature/clips_feature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

namespace cx {

class FileLoadFeature : public ClipsFeature {
public:
  FileLoadFeature();
  ~FileLoadFeature();

  void initialize();

  bool clips_env_init(LockSharedPtr<clips::Environment> &env) override;
  bool clips_env_destroyed(LockSharedPtr<clips::Environment> &env) override;

private:
  std::unique_ptr<rclcpp::Logger> logger_;

  std::vector<std::string> init_files_;
  std::vector<std::string> cleanup_files_;

  void resolve_files(const std::vector<std::string> &files_in,
                     const std::vector<std::string> &share_dirs,
                     std::vector<std::string> &files_out);
};
} // namespace cx

#endif // !CX_FEATURES__FILELOADFEATURE_HPP_
