// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include <string>

#include "cx_file_load_feature/file_load_feature.hpp"
#include <cx_utils/clips_env_context.hpp>
#include <cx_utils/param_utils.hpp>
#include <format>

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;
namespace cx {
FileLoadFeature::FileLoadFeature() {}

FileLoadFeature::~FileLoadFeature() {}

void FileLoadFeature::resolve_files(const std::vector<std::string> &files_in,
                                    const std::vector<std::string> &share_dirs,
                                    std::vector<std::string> &files_out) {
  // Try to resolve files
  for (const auto &file : files_in) {
    bool found = false;
    std::filesystem::path file_path(file);
    // Check if the path is absolute and exists
    if (file_path.is_absolute() && std::filesystem::exists(file_path)) {
      files_out.push_back(file_path);
      found = true;
    } else {
      // Otherwise check in the respective share directories
      for (const auto &package : share_dirs) {
        std::string dir_path =
            ament_index_cpp::get_package_share_directory(package);
        std::filesystem::path share_dir_path(dir_path);

        std::filesystem::path potential_path = share_dir_path / file_path;
        if (std::filesystem::exists(potential_path)) {
          files_out.push_back(potential_path);
          found = true;
          break;
        }
      }
    }
    if (!found) {
      RCLCPP_ERROR(*logger_, "Cannot resolve file %s", file.c_str());
    }
  }
}

void FileLoadFeature::initialize() {
  logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger(feature_name_));
  auto node = parent_.lock();
  if (!node) {
    return;
  }

  cx::cx_utils::declare_parameter_if_not_declared(
      node, feature_name_ + ".pkg_share_dirs",
      rclcpp::ParameterValue(std::vector<std::string>()));
  cx::cx_utils::declare_parameter_if_not_declared(
      node, feature_name_ + ".files",
      rclcpp::ParameterValue(std::vector<std::string>()));
  cx::cx_utils::declare_parameter_if_not_declared(
      node, feature_name_ + ".cleanup_files",
      rclcpp::ParameterValue(std::vector<std::string>()));
  std::vector<std::string> share_dirs;
  std::vector<std::string> files;
  std::vector<std::string> cleanup_files;
  node->get_parameter(feature_name_ + ".pkg_share_dirs", share_dirs);
  node->get_parameter(feature_name_ + ".files", files);
  node->get_parameter(feature_name_ + ".cleanup_files", cleanup_files);
  resolve_files(files, share_dirs, init_files_);
  resolve_files(cleanup_files, share_dirs, cleanup_files_);
}

bool FileLoadFeature::clips_env_init(LockSharedPtr<clips::Environment> &env) {
  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  RCLCPP_INFO(*logger_, "Initializing feature for environment %s",
              context->env_name_.c_str());
  for (const auto &f : init_files_) {

    if (!clips::BatchStar(env.get_obj().get(), f.c_str())) {
      clips::Writeln(
          env.get_obj().get(),
          std::format("Failed to initialize bach file {}", f).c_str());
      RCLCPP_ERROR(*logger_,
                   "Failed to initialize"
                   "batch file '%s' failed!, aborting...",
                   f.c_str());
      return false;
    }
  }
  return true;
}

bool FileLoadFeature::clips_env_destroyed(
    LockSharedPtr<clips::Environment> &env) {
  for (const auto &f : cleanup_files_) {

    if (!clips::BatchStar(env.get_obj().get(), f.c_str())) {
      clips::Writeln(
          env.get_obj().get(),
          std::format("Failed to initialize bach file {}", f).c_str());
      RCLCPP_ERROR(*logger_,
                   "Failed to initialize"
                   "batch file '%s' failed!, aborting...",
                   f.c_str());
      return false;
    }
  }
  return true;
}
} // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::FileLoadFeature, cx::ClipsFeature)
