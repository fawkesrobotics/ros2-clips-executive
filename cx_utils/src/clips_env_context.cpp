
// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include "cx_utils/clips_env_context.hpp"

#include "rclcpp/logging.hpp"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace cx {
constexpr const char *RED = "\033[31m";
constexpr const char *GREEN = "\033[32m";
constexpr const char *YELLOW = "\033[33m";
constexpr const char *BLUE = "\033[34m";
constexpr const char *MAGENTA = "\033[35m";
constexpr const char *CYAN = "\033[36m";
constexpr const char *WHITE = "\033[37m";
constexpr const char *BOLD = "\033[1m";
constexpr const char *RESET = "\033[0m";

CLIPSLogger::CLIPSLogger(const char *component, bool log_to_file)
    : component_(strdup(component)),
      logger_(rclcpp::get_logger(std::string(component_))) {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);

  std::ostringstream oss;
  oss << std::put_time(std::localtime(&now_time), "%Y-%m-%d-%H-%M-%S");

  std::string formatted_time = oss.str();
  if (log_to_file) {
    clips_logger_ = spdlog::basic_logger_st(
        (component_ ? (std::string)component_ : "CLIPS"),
        (rclcpp::get_logging_directory().string() + "/" +
         (component_ ? (std::string)component_ : "clips") + "_" +
         formatted_time + ".log"));
  } else {
    // Disable the logger by setting the log level to a level that filters out
    // all messages
    clips_logger_ = spdlog::stdout_color_mt("console");
    clips_logger_->set_level(spdlog::level::off);
  }
}

CLIPSLogger::~CLIPSLogger() {
  if (component_) {
    free(component_);
  }
}

void CLIPSLogger::log(const char *logical_name, const char *str) {
  size_t i = 0;
  while (str[i]) {
    i++;
  }
  if (str[i - 1] == '\n') {
    if (i > 1) {
      buffer_ += str;
    }
    if (strcmp(logical_name, "red") == 0) {
      RCLCPP_INFO(
          this->logger_,
          (std::string(cx::RED) + terminal_buffer_ + cx::RESET).c_str());
    } else if (strcmp(logical_name, "bold") == 0) {
      RCLCPP_INFO(
          this->logger_,
          (std::string(cx::BOLD) + terminal_buffer_ + cx::RESET).c_str());
    } else if (strcmp(logical_name, "green") == 0) {
      RCLCPP_INFO(
          this->logger_,
          (std::string(cx::GREEN) + terminal_buffer_ + cx::RESET).c_str());
    } else if (strcmp(logical_name, "yellow") == 0) {
      RCLCPP_INFO(
          this->logger_,
          (std::string(cx::YELLOW) + terminal_buffer_ + cx::RESET).c_str());
    } else if (strcmp(logical_name, "blue") == 0) {
      RCLCPP_INFO(
          this->logger_,
          (std::string(cx::BLUE) + terminal_buffer_ + cx::RESET).c_str());
    } else if (strcmp(logical_name, "magenta") == 0) {
      RCLCPP_INFO(
          this->logger_,
          (std::string(cx::MAGENTA) + terminal_buffer_ + cx::RESET).c_str());
    } else if (strcmp(logical_name, "cyan") == 0) {
      RCLCPP_INFO(
          this->logger_,
          (std::string(cx::CYAN) + terminal_buffer_ + cx::RESET).c_str());
    } else if (strcmp(logical_name, "white") == 0) {
      RCLCPP_INFO(
          this->logger_,
          (std::string(cx::WHITE) + terminal_buffer_ + cx::RESET).c_str());
    } else if (strcmp(logical_name, "debug") == 0 ||
               strcmp(logical_name, "logdebug") == 0 ||
               strcmp(logical_name, clips::STDOUT) == 0) {
      RCLCPP_DEBUG(this->logger_, terminal_buffer_.c_str());
    } else if (strcmp(logical_name, "warn") == 0 ||
               strcmp(logical_name, "logwarn") == 0 ||
               strcmp(logical_name, clips::STDWRN) == 0) {
      RCLCPP_WARN(this->logger_, terminal_buffer_.c_str());
    } else if (strcmp(logical_name, "error") == 0 ||
               strcmp(logical_name, "logerror") == 0 ||
               strcmp(logical_name, clips::STDERR) == 0) {
      RCLCPP_ERROR(this->logger_, terminal_buffer_.c_str());
    } else if (strcmp(logical_name, clips::STDIN) == 0) {
      // ignored
    } else {
      RCLCPP_INFO(this->logger_, terminal_buffer_.c_str());
    }
    // log any output to a dedicated clips log file
    clips_logger_->info(buffer_.c_str());
    clips_logger_->flush();
    buffer_.clear();
    terminal_buffer_.clear();

  } else {
    if (strcmp(logical_name, "red") == 0) {
      terminal_buffer_ += std::string(cx::RED) + str + cx::RESET;
    } else if (strcmp(logical_name, "bold") == 0) {
      terminal_buffer_ += std::string(cx::BOLD) + str + cx::RESET;
    } else if (strcmp(logical_name, "green") == 0) {
      terminal_buffer_ += std::string(cx::GREEN) + str + cx::RESET;
    } else if (strcmp(logical_name, "yellow") == 0) {
      terminal_buffer_ += std::string(cx::YELLOW) + str + cx::RESET;
    } else if (strcmp(logical_name, "blue") == 0) {
      terminal_buffer_ += std::string(cx::BLUE) + str + cx::RESET;
    } else if (strcmp(logical_name, "magenta") == 0) {
      terminal_buffer_ += std::string(cx::MAGENTA) + str + cx::RESET;
    } else if (strcmp(logical_name, "cyan") == 0) {
      terminal_buffer_ += std::string(cx::CYAN) + str + cx::RESET;
    } else if (strcmp(logical_name, "white") == 0) {
      terminal_buffer_ += std::string(cx::WHITE) + str + cx::RESET;
    } else {
      terminal_buffer_ += str;
    }
    buffer_ += str;
  }
}

CLIPSEnvContext *CLIPSEnvContext::get_context(clips::Environment *env) {
  using namespace clips;
  return (CLIPSEnvContext *)GetEnvironmentData(env, USER_ENVIRONMENT_DATA);
}

} // namespace cx
