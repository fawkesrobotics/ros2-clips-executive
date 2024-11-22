// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#ifndef _CX_UTILS_CLIPS_ENV_CONTEXT_HPP
#define _CX_UTILS_CLIPS_ENV_CONTEXT_HPP

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include "cx_utils/lock_shared_ptr.hpp"
#include <clips_ns/clips.h>

namespace cx {
class CLIPSLogger {
public:
  explicit CLIPSLogger(const char *component, bool log_to_file,
                       bool stdout_to_debug);
  ~CLIPSLogger();
  void log(const char *logical_name, const char *str);

private:
  char *component_;
  const rclcpp::Logger logger_;
  const bool stdout_to_debug_;
  std::shared_ptr<spdlog::logger> clips_logger_;
  std::string buffer_;
  std::string terminal_buffer_;
};

class CLIPSEnvContext {
public:
  std::string env_name_;
  LockSharedPtr<clips::Environment> env_lock_ptr_;
  CLIPSLogger logger_;

  static CLIPSEnvContext *get_context(clips::Environment *env);
};

} // namespace cx

#endif // !_CX_UTILS_CLIPS_ENV_CONTEXT_HPP
