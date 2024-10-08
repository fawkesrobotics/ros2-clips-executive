// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  CLIPSEnvManagerNode.cpp
 *
 *  Created: 25 June 2021
 *  Copyright  2021  Ivaylo Doychev
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <chrono>
#include <format>
#include <map>
#include <memory>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <string>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp/logging.hpp"

#include "cx_clips/CLIPSEnvManagerNode.h"

extern "C" {
#include <clips/clips.h>
}

#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

namespace cx {

#define ROUTNER_NAME "cxlog"

/// @cond INTERNALS
class CLIPSLogger {
public:
  explicit CLIPSLogger(const char *component, bool log_to_file)
      : component_(strdup(component)) {
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

  ~CLIPSLogger() {
    if (component_) {
      free(component_);
    }
  }

  void log(const char *logical_name, const char *str) {
    size_t i = 0;
    while (str[i]) {
      i++;
    }
    if (str[i - 1] == '\n') {
      if (i > 1) {
        buffer_ += str;
      }
      if (strcmp(logical_name, "debug") == 0 ||
          strcmp(logical_name, "logdebug") == 0 ||
          strcmp(logical_name, clips::STDOUT) == 0) {
        RCLCPP_DEBUG(this->logger_, buffer_.c_str());
      } else if (strcmp(logical_name, "warn") == 0 ||
                 strcmp(logical_name, "logwarn") == 0 ||
                 strcmp(logical_name, clips::STDWRN) == 0) {
        RCLCPP_WARN(this->logger_, buffer_.c_str());
      } else if (strcmp(logical_name, "error") == 0 ||
                 strcmp(logical_name, "logerror") == 0 ||
                 strcmp(logical_name, clips::STDERR) == 0) {
        RCLCPP_ERROR(this->logger_, buffer_.c_str());
      } else if (strcmp(logical_name, clips::STDIN) == 0) {
        // ignored
      } else {
        RCLCPP_INFO(this->logger_, buffer_.c_str());
      }
      // log any output to a dedicated clips log file
      clips_logger_->info(buffer_.c_str());
      clips_logger_->flush();
      buffer_.clear();

    } else {
      buffer_ += str;
    }
  }

private:
  char *component_ = strdup("clips_default_log_component");
  const rclcpp::Logger logger_ = rclcpp::get_logger(std::string(component_));
  std::shared_ptr<spdlog::logger> clips_logger_;
  std::string buffer_;
};

class CLIPSContextMaintainer {
public:
  explicit CLIPSContextMaintainer(const char *log_component_name,
                                  bool log_to_file)
      : logger(log_component_name, log_to_file) {}

  ~CLIPSContextMaintainer() {}

public:
  CLIPSLogger logger;
};

static bool log_router_query(clips::Environment * /*env*/,
                             const char *logical_name, void * /*context*/) {
  // envs
  if (strcmp(logical_name, "l") == 0)
    return true;
  if (strcmp(logical_name, "info") == 0)
    return true;
  if (strcmp(logical_name, "debug") == 0)
    return true;
  if (strcmp(logical_name, "warn") == 0)
    return true;
  if (strcmp(logical_name, "error") == 0)
    return true;
  if (strcmp(logical_name, "loginfo") == 0)
    return true;
  if (strcmp(logical_name, "logdebug") == 0)
    return true;
  if (strcmp(logical_name, "logwarn") == 0)
    return true;
  if (strcmp(logical_name, "logerror") == 0)
    return true;
  if (strcmp(logical_name, "stdout") == 0)
    return true;
  if (strcmp(logical_name, clips::STDOUT) == 0)
    return true;
  if (strcmp(logical_name, clips::STDWRN) == 0)
    return true;
  if (strcmp(logical_name, clips::STDERR) == 0)
    return true;
  return false;
}

static void log_router_print(clips::Environment * /*env*/,
                             const char *logical_name, const char *str,
                             void *context) {
  CLIPSLogger *logger = static_cast<CLIPSLogger *>(context);
  logger->log(logical_name, str);
}

static void log_router_exit(clips::Environment * /*env*/, int /*exit_code*/,
                            void * /*context*/) {
  // no particular handling of a closed router necessary
}

using namespace std::placeholders;

CLIPSEnvManagerNode::CLIPSEnvManagerNode()
    : rclcpp_lifecycle::LifecycleNode("clips_manager") {
  RCLCPP_INFO(get_logger(), "Initialising [%s]...", get_name());
  /*Register callback group for the destroy context client as it calls the
   * clips feature manager service and waits for a result*/
  callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
CLIPSEnvManagerNode::on_configure(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "Configuring [%s]...", get_name());

  destroy_feature_context_client =
      create_client<cx_msgs::srv::ClipsFeatureContext>(
          "clips_features_manager/destroy_feature_context",
          rclcpp::QoS(rclcpp::ServicesQoS()), callback_group_);

  create_env_service_ = create_service<cx_msgs::srv::CreateClipsEnv>(
      "clips_manager/create_env",
      std::bind(&CLIPSEnvManagerNode::create_env_callback, this, _1, _2, _3));

  destroy_env_service_ = create_service<cx_msgs::srv::DestroyClipsEnv>(
      "clips_manager/destroy_env",
      std::bind(&CLIPSEnvManagerNode::destroy_env_callback, this, _1, _2, _3));

  add_features_service_ = create_service<cx_msgs::srv::AddClipsFeatures>(
      "clips_manager/add_clips_features",
      std::bind(&CLIPSEnvManagerNode::add_clips_features_callback, this, _1, _2,
                _3));

  assert_can_remove_features_service_ =
      create_service<cx_msgs::srv::ClipsRemoveFeatures>(
          "clips_manager/assert_can_remove_features",
          std::bind(&CLIPSEnvManagerNode::assert_can_remove_features_callback,
                    this, _1, _2, _3));

  remove_features_service_ = create_service<cx_msgs::srv::ClipsRemoveFeatures>(
      "clips_manager/remove_features",
      std::bind(&CLIPSEnvManagerNode::remove_features_callback, this, _1, _2,
                _3));

  try {
    clips_dir_ =
        ament_index_cpp::get_package_share_directory("cx_clips") + "/clips/";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Configured [%s]!", get_name());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
CLIPSEnvManagerNode::on_activate(const rclcpp_lifecycle::State &state) {
  // no action on activate for now
  (void)state;
  RCLCPP_INFO(get_logger(), "Activated [%s]...", get_name());
  return CallbackReturn::SUCCESS;
}

void CLIPSEnvManagerNode::create_env_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::CreateClipsEnv::Request> request,
    const std::shared_ptr<cx_msgs::srv::CreateClipsEnv::Response> response) {
  (void)request_header; // the request header is not used in this callback

  if (envs_.find(request->env_name) != envs_.end()) {
    RCLCPP_ERROR(get_logger(),
                 "CLIPS environment '%s' already exists--> Should "
                 "be signaled! (e.g. as exception)",
                 request->env_name.c_str());
    response->success = false;
    response->error = "Enviroment " + request->env_name + " already exists!";
  } else {
    LockSharedPtr<clips::Environment> clips = new_env(request->log_name);

    const std::string &env_name = request->env_name;

    if (clips) {
      envs_[env_name].env = clips;

      // add generic functions
      add_functions(env_name);

      // assert all available features
      assert_features(clips, true);

      guarded_load(env_name, clips_dir_ + "utils.clp");
      guarded_load(env_name, clips_dir_ + "time.clp");
      guarded_load(env_name, clips_dir_ + "path.clp");

      clips::CLIPSValue *ret = NULL;
      clips::Eval(clips.get_obj().get(),
                  ("(path-add \"" + clips_dir_ + "\")").c_str(), ret);
      if (ret) {
        RCLCPP_ERROR(
            get_logger(),
            "Failed to initialise CLIPS environment '%s', path-add failed.",
            request->env_name.c_str());
        response->success = false;
      } else {
        response->success = true;
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to initialise CLIPS environment '%s'",
                   request->env_name.c_str());
      response->success = false;
      response->error = "Failed to initialize environment " + request->env_name;
    }
  }
}

void CLIPSEnvManagerNode::destroy_env_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::DestroyClipsEnv::Request> request,
    const std::shared_ptr<cx_msgs::srv::DestroyClipsEnv::Response> response) {

  (void)request_header; // the request header is not used in this callback
  const std::string &env_name = request->env_name;

  RCLCPP_WARN(get_logger(), "Deleting '%s' --- Clips Environment...",
              env_name.c_str());

  if (envs_.find(env_name) != envs_.end()) {

    clips::Environment *env = envs_[env_name].env.get_obj().get();
    CLIPSContextMaintainer *cm = static_cast<CLIPSContextMaintainer *>(
        clips::GetEnvironmentContext(env));

    clips::DeleteRouter(env, (char *)ROUTNER_NAME);
    clips::SetEnvironmentContext(env, NULL);
    delete cm;

    for (auto &feat : envs_[env_name].req_feat) {

      if (features_set.find(feat) != features_set.end()) {
        RCLCPP_WARN(get_logger(), "Destroying Feature %s", feat.c_str());

        call_feature_context_destroy(env_name, feat);
      }
    }

    envs_.erase(env_name);

    RCLCPP_WARN(get_logger(), "Deleted '%s' --- Clips Environment!",
                env_name.c_str());

    response->success = true;
  } else {
    RCLCPP_WARN(get_logger(), "Didn't find the provided env: '%s'!",
                env_name.c_str());
    response->success = true;
    // THERE IS CURRENTLY NO ERROR HANDLING --------------
  }
}

// Parameters of request: std::vector<std::string>
void CLIPSEnvManagerNode::add_clips_features_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::AddClipsFeatures::Request> request,
    const std::shared_ptr<cx_msgs::srv::AddClipsFeatures::Response> response) {

  (void)request_header; // the request header is not used in this callback
  bool success = true;

  for (const auto &feat : request->features) {

    if (features_set.find(feat) != features_set.end()) {
      RCLCPP_ERROR(get_logger(),
                   "Feature '%s' has already b/een registered--> Should "
                   "throw exception later",
                   feat.c_str());
      response->failed_features.push_back(feat);
      success = false;
      return;
    }

    RCLCPP_INFO(get_logger(), "Adding feature '%s'", feat.c_str());

    features_set.insert(feat);

    // assert feature availability to all registered CLIPS Envs
    for (auto &env : envs_) {
      std::lock_guard<std::mutex> guard(*(env.second.env.get_mutex_instance()));
      assert_features(env.second.env, false);
      clips::AssertString(env.second.env.get_obj().get(),
                          std::format("(ff-feature {})", feat).c_str());
    }
  }
  response->success = success;
}

// Parameters of request: std::vector<std::string>
void CLIPSEnvManagerNode::assert_can_remove_features_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::ClipsRemoveFeatures::Request> request,
    const std::shared_ptr<cx_msgs::srv::ClipsRemoveFeatures::Response>
        response) {

  (void)request_header; // the request header is not used in this callback
  for (const auto &feat : request->features) {

    for (const auto &env : envs_) {

      if (std::binary_search(env.second.req_feat.begin(),
                             env.second.req_feat.end(), feat)) {
        RCLCPP_ERROR(
            get_logger(),
            "Feature '%s' can't be removed: Env %s depends on it--> Should "
            "throw exception later",
            feat.c_str(), env.first.c_str());
        response->success = false;
        response->error = "Feature " + feat + " can't be removed - env " +
                          env.first + " depends on it!";
        return;
      }
    }
  }
  response->success = true;
}

// Parameters of request: std::vector<std::string>
void CLIPSEnvManagerNode::remove_features_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::ClipsRemoveFeatures::Request> request,
    const std::shared_ptr<cx_msgs::srv::ClipsRemoveFeatures::Response>
        response) {

  (void)request_header; // the request header is not used in this callback
  for (const auto &feat : request->features) {
    features_set.erase(feat);
  }
  response->success = true;
}

/** Get map of all environments
 * @return map of environment name to a shared lock ptr
 */
std::map<std::string, LockSharedPtr<clips::Environment>>
CLIPSEnvManagerNode::getEnvironments() const {
  std::map<std::string, LockSharedPtr<clips::Environment>> rv;
  for (const auto &envd : envs_) {
    rv[envd.first] = envd.second.env;
  }
  return rv;
}

LockSharedPtr<clips::Environment>
CLIPSEnvManagerNode::getEnvironmentByName(const std::string &env_name) {
  if (envs_.find(env_name) != envs_.end()) {
    // return the environment
    return envs_[env_name].env;
  } else {
    RCLCPP_ERROR(get_logger(),
                 "CLIPS environment '%s' does not exists--> Should "
                 "be signaled! (e.g. as exception)",
                 env_name.c_str());
    throw std::runtime_error("Wrong access to environment: " + env_name);
  }
}

// --------------- ALL PRIVATE FUNCTION HELPERS ---------------

// Helper function to convert a string to lowercase
std::string to_lowercase(const std::string &str) {
  std::string lower_str = str;
  std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return lower_str;
}

// Function to map string to clips::WatchItem enum (case-insensitive)
clips::WatchItem get_watch_item_from_string(const std::string &watch_str) {
  std::string lower_str = to_lowercase(watch_str);

  if (lower_str == "all")
    return clips::ALL;
  if (lower_str == "facts")
    return clips::FACTS;
  if (lower_str == "instances")
    return clips::INSTANCES;
  if (lower_str == "slots")
    return clips::SLOTS;
  if (lower_str == "rules")
    return clips::RULES;
  if (lower_str == "activations")
    return clips::ACTIVATIONS;
  if (lower_str == "messages")
    return clips::MESSAGES;
  if (lower_str == "message_handlers")
    return clips::MESSAGE_HANDLERS;
  if (lower_str == "generic_functions")
    return clips::GENERIC_FUNCTIONS;
  if (lower_str == "methods")
    return clips::METHODS;
  if (lower_str == "deffunctions")
    return clips::DEFFUNCTIONS;
  if (lower_str == "compilations")
    return clips::COMPILATIONS;
  if (lower_str == "statistics")
    return clips::STATISTICS;
  if (lower_str == "globals")
    return clips::GLOBALS;
  if (lower_str == "focus")
    return clips::FOCUS;

  throw std::invalid_argument("Unknown watch item: " + watch_str);
}

LockSharedPtr<clips::Environment>
CLIPSEnvManagerNode::new_env(const std::string &log_component_name) {
  RCLCPP_INFO(get_logger(), "Initialising new CLIPS ENVIRONMENT: %s",
              log_component_name.c_str());

  // CLIPS overwrites the SIGINT hnadler, restore after
  struct sigaction oldact;

  if (sigaction(SIGINT, NULL, &oldact) == 0) {
    std::shared_ptr<clips::Environment> new_env(clips::CreateEnvironment());

    LockSharedPtr<clips::Environment> clips{std::move(new_env)};
    // Only place to init the env mutex
    clips.init_mutex();
    // // silent clips by default
    clips::Unwatch(clips.get_obj().get(), clips::WatchItem::ALL);
    declare_parameter("log_clips_to_file", true);
    declare_parameter("watch", std::vector<std::string>{"facts", "rules"});
    std::vector<std::string> watch_info;
    get_parameter("watch", watch_info);
    for (const auto &w : watch_info) {
      clips::WatchItem watch_item = get_watch_item_from_string(w);
      clips::Watch(clips.get_obj().get(), watch_item);
    }
    bool log_to_file = false;
    get_parameter("log_clips_to_file", log_to_file);

    CLIPSContextMaintainer *cm =
        new CLIPSContextMaintainer(log_component_name.c_str(), log_to_file);

    clips::Environment *env = clips.get_obj().get();

    clips::SetEnvironmentContext(env, cm);

    clips::AddRouter(env, (char *)ROUTNER_NAME, /*router priority*/
                     40, log_router_query, log_router_print, NULL, NULL,
                     log_router_exit, &cm->logger);

    sigaction(SIGINT, &oldact, NULL);

    RCLCPP_INFO(get_logger(), "Initialisied new CLIPS ENVIRONMENT: %s",
                log_component_name.c_str());
    return clips;
  } else {
    RCLCPP_ERROR(
        get_logger(),
        ("CLIPS: Unable to backup --- SIGINT sigaction for restoration."));
    LockSharedPtr<clips::Environment> clips{};
    return clips;
  }
}

void CLIPSEnvManagerNode::assert_features(
    LockSharedPtr<clips::Environment> &clips, bool immediate_assert) {

  // deffact so it survives a reset
  std::string deffacts = "(deffacts ff-features-available";

  for (const auto &feat : features_set) {
    deffacts += " (ff-feature " + feat + ")";
    if (immediate_assert) {
      clips::AssertString(clips.get_obj().get(),
                          std::format("(ff-feature {})", feat).c_str());
    }
  }
  deffacts += ")";

  clips::BuildError res = clips::Build(clips.get_obj().get(), deffacts.c_str());
  if (res != clips::BuildError::BE_NO_ERROR) {
    RCLCPP_WARN(get_logger(),
                "Failed to build deffacts ff-features-available (error %i)",
                static_cast<int>(res));
  }
  RCLCPP_INFO(get_logger(), "Asserted features!");
}

void CLIPSEnvManagerNode::guarded_load(const std::string &env_name,
                                       const std::string &filename) {
  if (envs_.find(env_name) == envs_.end()) {
    RCLCPP_ERROR(get_logger(),
                 "guarded_load: env %s has not been registered --> Should "
                 "be signaled! (e.g. as exception)",
                 env_name.c_str());
  }

  LockSharedPtr<clips::Environment> &clips = envs_[env_name].env;

  clips::LoadError res = clips::Load(clips.get_obj().get(), filename.c_str());
  if (res != clips::LoadError::LE_NO_ERROR) {
    // destroy_env(env_name);
    RCLCPP_ERROR(get_logger(),
                 "guarded_load: %s can't find %s --> Should "
                 "throw exception later",
                 env_name.c_str(), filename.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "guarded_load: Loaded file %s for environment %s",
                filename.c_str(), env_name.c_str());
  }
}

void CLIPSEnvManagerNode::add_functions(const std::string &env_name) {
  clips::Environment *env = getEnvironmentByName(env_name).get_obj().get();
  clips::AddUDF(
      env, "now", "d", 0, 0, NULL,
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        CLIPSEnvManagerNode *instance =
            static_cast<CLIPSEnvManagerNode *>(udfc->context);
        double currentTime = instance->get_clock()->now().seconds();
        out->floatValue = clips::CreateFloat(env, currentTime);
      },
      "clips_now", this);

  clips::AddUDF(
      env, "now-systime", "d", 0, 0, NULL,
      [](clips::Environment *env, clips::UDFContext * /*udfc*/,
         clips::UDFValue *out) {
        using namespace std::chrono;
        // get system seconds
        auto now = time_point_cast<seconds>(system_clock::now());

        out->integerValue =
            clips::CreateInteger(env, now.time_since_epoch().count());
      },
      "clips_now_systime", NULL);
}

void CLIPSEnvManagerNode::call_feature_context_destroy(
    const std::string &env_name, const std::string &feature_name) {

  RCLCPP_INFO(get_logger(), "IN CALL FEATURE CONTEXT DESTROY!");

  while (!destroy_feature_context_client->wait_for_service(5s)) {
    if (!rclcpp::ok()) {

      RCLCPP_ERROR(get_logger(),
                   "%s: timed out waiting for service availability",
                   destroy_feature_context_client->get_service_name());
      return;
    }
    RCLCPP_WARN(get_logger(), "%s: still waiting for service...",
                destroy_feature_context_client->get_service_name());
  }

  auto req = std::make_shared<cx_msgs::srv::ClipsFeatureContext::Request>();
  req->env_name = env_name;
  req->feature_name = feature_name;

  auto future_res = destroy_feature_context_client->async_send_request(req);

  auto status = future_res.wait_for(10s);

  if (status == std::future_status::ready) {
    if (!future_res.get()->success) {
      RCLCPP_ERROR(get_logger(), "ERROR --- %s",
                   future_res.get()->error.c_str());
    }
  } else {
    RCLCPP_ERROR(get_logger(), "%s: timed out waiting for response!",
                 destroy_feature_context_client->get_service_name());
  }

  // int finishedWithCode = -1;
  // auto inner_client_callback =
  //     [&,
  //     this](rclcpp::Client<cx_msgs::srv::ClipsFeatureContext>::SharedFuture
  //                   features_manager_call) {
  //       auto result = features_manager_call.get();
  //       finishedWithCode = result->success;
  //       RCLCPP_INFO(get_logger(), " callback was executed");
  //     };

  // auto future_res = destroy_feature_context_client->async_send_request(
  //     req, inner_client_callback);

  // // Wait for the result of the features manager, as it needs to be
  // synchronous,
  // // otherwise env can be deleted without removing the features!.
  // while (finishedWithCode < 0 && rclcpp::ok()) {
  //   std::this_thread::sleep_for(10ms);
  // }
}

} // namespace cx
