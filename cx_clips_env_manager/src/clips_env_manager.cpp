// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include <chrono>
#include <format>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/logging.hpp"

#include "cx_clips_env_manager/clips_env_manager.h"

#include "cx_utils/clips_env_context.hpp"
#include "cx_utils/param_utils.hpp"

#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

namespace cx {
constexpr const char *ROUTER_NAME = "cxlog";

static bool log_router_query(clips::Environment * /*env*/,
                             const char *logical_name, void * /*context*/) {
  // envs
  if (strcmp(logical_name, "l") == 0)
    return true;
  if (strcmp(logical_name, "red") == 0)
    return true;
  if (strcmp(logical_name, "green") == 0)
    return true;
  if (strcmp(logical_name, "blue") == 0)
    return true;
  if (strcmp(logical_name, "yellow") == 0)
    return true;
  if (strcmp(logical_name, "magenta") == 0)
    return true;
  if (strcmp(logical_name, "cyan") == 0)
    return true;
  if (strcmp(logical_name, "white") == 0)
    return true;
  if (strcmp(logical_name, "bold") == 0)
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

CLIPSEnvManager::CLIPSEnvManager()
    : rclcpp_lifecycle::LifecycleNode("clips_manager") {
  envs_.init_mutex();
  RCLCPP_INFO(get_logger(), "Initialising [%s]...", get_name());
}

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn CLIPSEnvManager::on_configure(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "Configuring [%s]...", get_name());

  create_env_service_ = create_service<cx_msgs::srv::CreateClipsEnv>(
      "clips_manager/create_env",
      std::bind(&CLIPSEnvManager::create_env_callback, this, _1, _2, _3));

  destroy_env_service_ = create_service<cx_msgs::srv::DestroyClipsEnv>(
      "clips_manager/destroy_env",
      std::bind(&CLIPSEnvManager::destroy_env_callback, this, _1, _2, _3));

  std::shared_ptr<EnvsMap> envs = std::make_shared<EnvsMap>();

  auto node = shared_from_this();
  std::vector<std::string> config_envs;
  cx::cx_utils::declare_parameter_if_not_declared(
      node, "environments", rclcpp::ParameterValue(config_envs));
  get_parameter("environments", config_envs);
  for (const auto &env_name : config_envs) {
    envs->insert({env_name, new_env(env_name)});
  }

  envs_.set_obj(envs);

  feature_manager_.configure(node, "ClipsFeatureManager", envs_);

  RCLCPP_INFO(get_logger(), "Configured [%s]!", get_name());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
CLIPSEnvManager::on_activate(const rclcpp_lifecycle::State &state) {
  // no action on activate for now
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating [%s]...", get_name());
  feature_manager_.activate();
  std::scoped_lock envs_lock(*(envs_.get_mutex_instance()));
  for (auto &env : *(envs_.get_obj())) {
    std::scoped_lock env_lock(*(env.second.get_mutex_instance()));
    clips::Reset(env.second.get_obj().get());
    clips::RefreshAllAgendas(env.second.get_obj().get());
    clips::Run(env.second.get_obj().get(), -1);
  }
  RCLCPP_INFO(get_logger(), "Activated [%s]...", get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn
CLIPSEnvManager::on_deactivate(const rclcpp_lifecycle::State &state) {
  // no action on activate for now
  (void)state;
  {
    std::scoped_lock envs_lock(*(envs_.get_mutex_instance()));
    for (auto &env : *(envs_.get_obj())) {
      std::scoped_lock env_lock(*(env.second.get_mutex_instance()));
      clips::AssertString(env.second.get_obj().get(), "(executive-finalize)");

      clips::RefreshAllAgendas(env.second.get_obj().get());
      clips::Run(env.second.get_obj().get(), -1);
    }
  }
  feature_manager_.deactivate();
  create_env_service_.reset();
  destroy_env_service_.reset();
  RCLCPP_INFO(get_logger(), "Deactivated [%s]...", get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn
CLIPSEnvManager::on_shutdown(const rclcpp_lifecycle::State &state) {
  // no action on activate for now
  (void)state;
  RCLCPP_INFO(get_logger(), "Shut down [%s]...", get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn CLIPSEnvManager::on_error(const rclcpp_lifecycle::State &state) {
  // no action on activate for now
  (void)state;
  RCLCPP_INFO(get_logger(), "Error [%s]...", get_name());
  return CallbackReturn::SUCCESS;
}

void CLIPSEnvManager::create_env_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::CreateClipsEnv::Request> request,
    const std::shared_ptr<cx_msgs::srv::CreateClipsEnv::Response> response) {
  (void)request_header; // the request header is not used in this callback

  std::scoped_lock lock(*envs_.get_mutex_instance());
  if (envs_->find(request->env_name) != envs_->end()) {
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
      envs_->insert({env_name, clips});
      feature_manager_.activate_env(env_name, clips);
      response->success = true;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to initialize CLIPS environment '%s'",
                   request->env_name.c_str());
      response->success = false;
      response->error = "Failed to initialize environment " + request->env_name;
    }
  }
}

void CLIPSEnvManager::destroy_env_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::DestroyClipsEnv::Request> request,
    const std::shared_ptr<cx_msgs::srv::DestroyClipsEnv::Response> response) {

  (void)request_header; // the request header is not used in this callback
  std::scoped_lock lock(*envs_.get_mutex_instance());
  const std::string &env_name = request->env_name;
  response->success = delete_env(env_name);
  if (!response->success) {
    response->error = "unknown Environment name";
  }
}
// /** Get map of all environments
//  * @return map of environment name to a shared lock ptr
//  */
// std::map<std::string, LockSharedPtr<clips::Environment>>
// CLIPSEnvManager::getEnvironments() const {
//   std::map<std::string, LockSharedPtr<clips::Environment>> rv;
//   for (const auto &envd : envs_) {
//     rv[envd.first] = envd.second.env;
//   }
//   return rv;
// }
//
// LockSharedPtr<clips::Environment>
// CLIPSEnvManager::getEnvironmentByName(const std::string &env_name) {
//   if (envs_.find(env_name) != envs_.end()) {
//     // return the environment
//     return envs_[env_name].env;
//   } else {
//     RCLCPP_ERROR(get_logger(),
//                  "CLIPS environment '%s' does not exists--> Should "
//                  "be signaled! (e.g. as exception)",
//                  env_name.c_str());
//     throw std::runtime_error("Wrong access to environment: " + env_name);
//   }
// }

// --------------- ALL PRIVATE FUNCTION HELPERS ---------------

bool CLIPSEnvManager::delete_env(const std::string &env_name) {
  RCLCPP_WARN(get_logger(), "Deleting '%s' --- Clips Environment...",
              env_name.c_str());

  if (envs_->find(env_name) != envs_->end()) {

    clips::Environment *env = envs_->at(env_name).get_obj().get();
    feature_manager_.deactivate_env(env_name, envs_->at(env_name));

    clips::DeleteRouter(env, (char *)ROUTER_NAME);
    clips::DestroyEnvironment(env);

    envs_->erase(env_name);

    RCLCPP_WARN(get_logger(), "Deleted '%s' --- Clips Environment!",
                env_name.c_str());

    return true;
  } else {
    RCLCPP_WARN(get_logger(), "Didn't find the provided env: '%s'!",
                env_name.c_str());
    return false;
  }
}

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

cx::LockSharedPtr<clips::Environment>
CLIPSEnvManager::new_env(const std::string &env_name) {

  std::shared_ptr<clips::Environment> new_env(clips::CreateEnvironment());

  LockSharedPtr<clips::Environment> clips(std::move(new_env));
  clips::Environment *env = clips.get_obj().get();
  // Only place to init the env mutex
  clips.init_mutex();
  // // silent clips by default
  clips::Unwatch(env, clips::WatchItem::ALL);
  if (!clips::AllocateEnvironmentData(env, USER_ENVIRONMENT_DATA,
                                      sizeof(CLIPSEnvContext), NULL)) {
    RCLCPP_ERROR(get_logger(), "Error allocating environment data for %s",
                 env_name.c_str());
    clips::Writeln(env, "Error allocating environment data");
    clips::ExitRouter(env, EXIT_FAILURE);
    return clips;
  }
  cx::cx_utils::declare_parameter_if_not_declared(
      this, env_name + ".log_clips_to_file", rclcpp::ParameterValue(true));
  cx::cx_utils::declare_parameter_if_not_declared(
      this, env_name + ".watch",
      rclcpp::ParameterValue(std::vector<std::string>{"facts", "rules"}));
  std::vector<std::string> watch_info;
  get_parameter(env_name + ".watch", watch_info);
  for (const auto &w : watch_info) {
    clips::WatchItem watch_item = get_watch_item_from_string(w);
    clips::Watch(clips.get_obj().get(), watch_item);
  }
  bool log_to_file = false;
  get_parameter(env_name + ".log_clips_to_file", log_to_file);

  auto context = CLIPSEnvContext::get_context(env);
  context->env_name_ = env_name;
  context->env_lock_ptr_ = clips;
  // mem allocated already, so construct object in-place
  new (&context->logger_) CLIPSLogger(env_name.c_str(), log_to_file);

  clips::AddRouter(env, (char *)ROUTER_NAME, /*router priority*/
                   40, log_router_query, log_router_print, NULL, NULL,
                   log_router_exit, &context->logger_);

  RCLCPP_INFO(get_logger(), "Initialisied new CLIPS environment: %s",
              env_name.c_str());
  return clips;
}

} // namespace cx
