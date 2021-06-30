#include <string>

#include "cx_features/RedefineWarningFeature.hpp"

extern "C" {
#include <clips/clips.h>
}

namespace cx {

#define ROUTER_NAME "clips-feature-redefine-warn"

/// @cond INTERNALS
class CLIPSRedefineWarningLogger {
public:
  explicit CLIPSRedefineWarningLogger(const char *component)
      : component_(strdup(component)), warn_filter_{"[CSTRCPSR1] WARNING: "} {}

  ~CLIPSRedefineWarningLogger() { free(component_); }

  bool buffer_warning(const char *str, std::string &buffer_string) {
    if (strcmp(str, "\n") == 0) {
      if (warn_buffer_ == warn_filter_) {
        warn_buffer_.clear();
        return false;
      } else {
        buffer_string = warn_buffer_;
        warn_buffer_.clear();
        return true;
      }
    } else {
      warn_buffer_ += str;
      if (warn_filter_.find(warn_buffer_) == std::string::npos) {
        warn_buffer_.clear();
        buffer_string = str;
        return true;
      } else {
        return false;
      }
    }
  }

  void log(const char *str) {
    if (strcmp(str, "\n") == 0) {
      if (buffer_.compare(0, 11, "Redefining ") == 0) {
        RCLCPP_ERROR(this->logger_, component_);
      }
      buffer_.clear();
    } else {
      buffer_ += str;
    }
  }

private:
  char *component_ = strdup("clips_default_refine_warning_log_component");
  const rclcpp::Logger logger_ = rclcpp::get_logger(std::string(component_));
  std::string buffer_;
  std::string warn_buffer_;
  std::string warn_filter_;
};

static int redefine_warning_router_query(void *env, char *logical_name) {
  if (strcmp(logical_name, WDIALOG) == 0)
    return TRUE;
  if (strcmp(logical_name, WWARNING) == 0)
    return TRUE;
  return FALSE;
}

static int redefine_warning_router_print(void *env, char *logical_name,
                                         char *str) {
  void *rc = GetEnvironmentRouterContext(env);
  CLIPSRedefineWarningLogger *logger =
      static_cast<CLIPSRedefineWarningLogger *>(rc);

  if (strcmp(logical_name, WWARNING) == 0) {
    // check if it's the ill-guided output of PrintWarningID from prntutil.c
    std::string wbuffer;
    ;
    if (logger->buffer_warning(str, wbuffer)) {
      // not the warning we were looking for, forward
      EnvDeactivateRouter(env, (char *)ROUTER_NAME);
      EnvPrintRouter(env, logical_name, (char *)wbuffer.c_str());
      if (strcmp(str, "\n") == 0 && wbuffer != "") {
        EnvPrintRouter(env, logical_name, str);
      }
      EnvActivateRouter(env, (char *)ROUTER_NAME);
    }
  } else {
    logger->log(str);
  }

  return TRUE;
}

static int redefine_warning_router_exit(void *env, int exit_code) {
  return TRUE;
}

/// @endcond

RedefineWarningFeature::RedefineWarningFeature() {}

RedefineWarningFeature::~RedefineWarningFeature() {}

std::string RedefineWarningFeature::getFeatureName() const {
  return clips_feature_name;
}

void RedefineWarningFeature::initialise(const std::string &feature_name) {
  clips_feature_name = feature_name;
}

bool RedefineWarningFeature::clips_context_init(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialising context for feature %s",
              clips_feature_name.c_str());
  std::string name = "RWCLIPS|" + env_name;

  CLIPSRedefineWarningLogger *cl = new CLIPSRedefineWarningLogger(name.c_str());

  EnvAddRouterWithContext(clips->cobj(), (char *)ROUTER_NAME,
                          /* exclusive */ 40, redefine_warning_router_query,
                          redefine_warning_router_print,
                          /* getc */ NULL,
                          /* ungetc */ NULL, redefine_warning_router_exit, cl);
  clips->watch("compilations");
  return true;
}

bool RedefineWarningFeature::clips_context_destroyed(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Destroying clips context for feature %s!",
              clips_feature_name.c_str());

  std::string name = "RWCLIPS|" + env_name;

  CLIPSRedefineWarningLogger *logger = NULL;

  struct routerData *rd = RouterData(clips->cobj());
  struct router *r = rd->ListOfRouters;
  while (r != NULL) {
    if (strcmp(r->name, ROUTER_NAME) == 0) {
      logger = static_cast<CLIPSRedefineWarningLogger *>(r->context);
      break;
    }
    r = r->next;
  }

  EnvDeleteRouter(clips->cobj(), (char *)ROUTER_NAME);
  delete logger;

  return true;
}

} // namespace cx
