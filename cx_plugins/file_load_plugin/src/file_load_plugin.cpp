// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include <string>

#include "cx_file_load_plugin/file_load_plugin.hpp"
#include <cx_utils/clips_env_context.hpp>
#include <cx_utils/param_utils.hpp>
#include <format>

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;
namespace cx {
FileLoadPlugin::FileLoadPlugin() {}

FileLoadPlugin::~FileLoadPlugin() {}

void FileLoadPlugin::initialize() {
  logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger(plugin_name_));
  auto node = parent_.lock();
  if (!node) {
    return;
  }

  cx::cx_utils::declare_parameter_if_not_declared(
      node, plugin_name_ + ".pkg_share_dirs",
      rclcpp::ParameterValue(std::vector<std::string>()));
  cx::cx_utils::declare_parameter_if_not_declared(
      node, plugin_name_ + ".load",
      rclcpp::ParameterValue(std::vector<std::string>()));
  cx::cx_utils::declare_parameter_if_not_declared(
      node, plugin_name_ + ".batch",
      rclcpp::ParameterValue(std::vector<std::string>()));
  cx::cx_utils::declare_parameter_if_not_declared(
      node, plugin_name_ + ".cleanup_batch",
      rclcpp::ParameterValue(std::vector<std::string>()));
  std::vector<std::string> share_dirs;
  std::vector<std::string> files;
  std::vector<std::string> batch_files;
  std::vector<std::string> cleanup_files;
  node->get_parameter(plugin_name_ + ".pkg_share_dirs", share_dirs);
  node->get_parameter(plugin_name_ + ".load", files);
  node->get_parameter(plugin_name_ + ".batch", batch_files);
  node->get_parameter(plugin_name_ + ".cleanup_batch", cleanup_files);
  try {
    cx_utils::resolve_files(files, share_dirs, init_files_);
    cx_utils::resolve_files(batch_files, share_dirs, init_batch_files_);
    cx_utils::resolve_files(cleanup_files, share_dirs, cleanup_files_);
  } catch (std::exception &e) {
    RCLCPP_ERROR(*logger_, "%s", e.what());
  }
}

bool FileLoadPlugin::clips_env_init(LockSharedPtr<clips::Environment> &env) {
  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  RCLCPP_INFO(*logger_, "Initializing plugin for environment %s",
              context->env_name_.c_str());
  for (const auto &f : init_files_) {

    if (clips::EE_NO_ERROR != clips::Eval(env.get_obj().get(),
                                          std::format("(load* {})", f).c_str(),
                                          NULL)) {
      clips::Writeln(env.get_obj().get(),
                     std::format("Failed to load file {}", f).c_str());
      RCLCPP_ERROR(*logger_, "Failed to load file '%s' failed!, aborting...",
                   f.c_str());
      return false;
    }
  }
  for (const auto &f : init_batch_files_) {

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

bool FileLoadPlugin::clips_env_destroyed(
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

PLUGINLIB_EXPORT_CLASS(cx::FileLoadPlugin, cx::ClipsPlugin)
