#include <chrono>
#include <map>
#include <memory>
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
  explicit CLIPSLogger(const char *component) : component_(strdup(component)) {}

  ~CLIPSLogger() {
    if (component_) {
      free(component_);
    }
  }

  void log(const char *logical_name, const char *str) {
    if (strcmp(str, "\n") == 0) {
      if (strcmp(logical_name, "debug") == 0 ||
          strcmp(logical_name, "logdebug") == 0 ||
          strcmp(logical_name, WTRACE) == 0) {
        // LATER DEBUG
        RCLCPP_INFO(this->logger_, component_ ? "CLIPS", "%s",
                     buffer_.c_str() : component_);
      } else if (strcmp(logical_name, "warn") == 0 ||
                 strcmp(logical_name, "logwarn") == 0 ||
                 strcmp(logical_name, WWARNING) == 0) {
        RCLCPP_WARN(this->logger_, component_ ? "CLIPS", "%s",
                    buffer_.c_str() : component_);
      } else if (strcmp(logical_name, "error") == 0 ||
                 strcmp(logical_name, "logerror") == 0 ||
                 strcmp(logical_name, WERROR) == 0) {
        RCLCPP_ERROR(this->logger_, component_ ? "CLIPS", "%s",
                     buffer_.c_str() : component_);
      } else if (strcmp(logical_name, WDIALOG) == 0) {
        // ignored
      } else {
        RCLCPP_INFO(this->logger_, component_ ? "CLIPS", "%s",
                    buffer_.c_str() : component_);
      }

      buffer_.clear();
    } else {
      buffer_ += str;
    }
  }

private:
  char *component_ = strdup("clips_default_log_component");
  const rclcpp::Logger logger_ = rclcpp::get_logger(std::string(component_));
  std::string buffer_;
};

class CLIPSContextMaintainer {
public:
  explicit CLIPSContextMaintainer(const char *log_component_name)
      : logger(log_component_name) {}

  ~CLIPSContextMaintainer() {}

public:
  CLIPSLogger logger;
};

static int log_router_query(void *env, char *logical_name) {
  if (strcmp(logical_name, "l") == 0)
    return TRUE;
  if (strcmp(logical_name, "info") == 0)
    return TRUE;
  if (strcmp(logical_name, "debug") == 0)
    return TRUE;
  if (strcmp(logical_name, "warn") == 0)
    return TRUE;
  if (strcmp(logical_name, "error") == 0)
    return TRUE;
  if (strcmp(logical_name, "loginfo") == 0)
    return TRUE;
  if (strcmp(logical_name, "logdebug") == 0)
    return TRUE;
  if (strcmp(logical_name, "logwarn") == 0)
    return TRUE;
  if (strcmp(logical_name, "logerror") == 0)
    return TRUE;
  if (strcmp(logical_name, "stdout") == 0)
    return TRUE;
  if (strcmp(logical_name, WTRACE) == 0)
    return TRUE;
  if (strcmp(logical_name, WDIALOG) == 0)
    return TRUE;
  if (strcmp(logical_name, WWARNING) == 0)
    return TRUE;
  if (strcmp(logical_name, WERROR) == 0)
    return TRUE;
  if (strcmp(logical_name, WDISPLAY) == 0)
    return TRUE;
  return FALSE;
}

static int log_router_print(void *env, char *logical_name, char *str) {
  void *rc = GetEnvironmentRouterContext(env);
  CLIPSLogger *logger = static_cast<CLIPSLogger *>(rc);
  logger->log(logical_name, str);
  return TRUE;
}

static int log_router_exit(void *env, int exit_code) { return TRUE; }

using namespace std::placeholders;

CLIPSEnvManagerNode::CLIPSEnvManagerNode()
    : rclcpp_lifecycle::LifecycleNode("clips_manager") {
  RCLCPP_INFO(get_logger(), "Initialising [%s]...", get_name());
  /*Register callback group for the destroy context client as it calls the
   * clips feature manager service and waits for a result*/
  callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  declare_parameter("clips_dirs", clips_dir_);
  declare_parameter("retract_early", FALSE);
}

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
CLIPSEnvManagerNode::on_configure(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "Configuring [%s]...", get_name());

  init_feature_context_client =
      create_client<cx_msgs::srv::ClipsFeatureContext>(
          "clips_features_manager/init_feature_context");

  destroy_feature_context_client =
      create_client<cx_msgs::srv::ClipsFeatureContext>(
          "clips_features_manager/destroy_feature_context",
          rmw_qos_profile_services_default, callback_group_);

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
    clips_dir_ = std::move(
        ament_index_cpp::get_package_share_directory("cx_clips") + "/clips/");
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return CallbackReturn::FAILURE;
  }

  // Initialise CLIPS
  RCLCPP_INFO(get_logger(), "Initialising CLIPS!");

  CLIPS::init();

  RCLCPP_INFO(get_logger(), "Configured [%s]!", get_name());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
CLIPSEnvManagerNode::on_activate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating [%s]...", get_name());
  RCLCPP_INFO(get_logger(), "Activated [%s]...", get_name());
  return CallbackReturn::SUCCESS;
}

void CLIPSEnvManagerNode::create_env_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::CreateClipsEnv::Request> request,
    const std::shared_ptr<cx_msgs::srv::CreateClipsEnv::Response> response) {

  if (envs_.find(request->env_name) != envs_.end()) {
    RCLCPP_ERROR(get_logger(),
                 "CLIPS environment '%s' already exists--> Should "
                 "be signaled! (e.g. as exception)",
                 request->env_name.c_str());
    response->success = FALSE;
    response->error = "Enviroment " + request->env_name + " already exists!";

  } else {
    LockSharedPtr<CLIPS::Environment> clips = new_env(request->log_name);

    const std::string &env_name = request->env_name;

    if (clips) {
      envs_[env_name].env = clips;

      // add generic functions
      add_functions(env_name, clips);

      // assert all available features
      assert_features(clips, true);

      guarded_load(env_name, clips_dir_ + "utils.clp");
      guarded_load(env_name, clips_dir_ + "time.clp");
      guarded_load(env_name, clips_dir_ + "path.clp");

      clips->evaluate("(path-add \"" + clips_dir_ + "\")");
      response->success = true;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to initialise CLIPS environment '%s'",
                   request->env_name.c_str());
      response->success = FALSE;
      response->error = "Failed to initialize environment " + request->env_name;
    }
  }
}

void CLIPSEnvManagerNode::destroy_env_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::DestroyClipsEnv::Request> request,
    const std::shared_ptr<cx_msgs::srv::DestroyClipsEnv::Response> response) {

  const std::string &env_name = request->env_name;

  RCLCPP_WARN(get_logger(), "Deleting '%s' --- Clips Environment...",
              env_name.c_str());

  if (envs_.find(env_name) != envs_.end()) {

    void *env = envs_[env_name].env->cobj();
    CLIPSContextMaintainer *cm =
        static_cast<CLIPSContextMaintainer *>(GetEnvironmentContext(env));

    EnvDeleteRouter(env, (char *)ROUTNER_NAME);
    SetEnvironmentContext(env, NULL);
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

    response->success = TRUE;
  } else {
    RCLCPP_WARN(get_logger(), "Didn't find the provided env: '%s'!",
                env_name.c_str());
    response->success = TRUE;
    // THERE IS CURRENTLY NO ERROR HANDLING --------------
  }
}

// Parameters of request: std::vector<std::string>
void CLIPSEnvManagerNode::add_clips_features_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::AddClipsFeatures::Request> request,
    const std::shared_ptr<cx_msgs::srv::AddClipsFeatures::Response> response) {

  bool success = true;

  for (const auto &feat : request->features) {

    if (features_set.find(feat) != features_set.end()) {
      RCLCPP_ERROR(get_logger(),
                   "Feature '%s' has already been registered--> Should "
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
      env.second.env.scopedLock();
      assert_features(env.second.env, false);
      env.second.env->assert_fact_f("(ff-feature %s)", feat.c_str());
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
        response->error = "Minimal feature " + feat +
                          " can't be removed - env " + env.first +
                          " depends on it!";
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

  for (const auto &feat : request->features) {
    features_set.erase(feat);
  }
  response->success = true;
}

/** Get map of all environments
 * @return map of environment name to a shared lock ptr
 */
std::map<std::string, LockSharedPtr<CLIPS::Environment>>
CLIPSEnvManagerNode::getEnvironments() const {
  std::map<std::string, LockSharedPtr<CLIPS::Environment>> rv;
  for (const auto &envd : envs_) {
    rv[envd.first] = envd.second.env;
  }
  return rv;
}

LockSharedPtr<CLIPS::Environment>
CLIPSEnvManagerNode::getEnvironmentByName(const std::string &env_name) {
  if (envs_.find(env_name) != envs_.end()) {
    // return the environment
    return envs_[env_name].env;
  } else {
    RCLCPP_ERROR(get_logger(),
                 "CLIPS environment '%s' does not exists--> Should "
                 "be signaled! (e.g. as exception)",
                 env_name.c_str());
  }
}

// --------------- ALL PRIVATE FUNCTION HELPERS ---------------

LockSharedPtr<CLIPS::Environment>
CLIPSEnvManagerNode::new_env(const std::string &log_component_name) {
  RCLCPP_INFO(get_logger(), "Initialising new CLIPS ENVIRONMENT: %s",
              log_component_name.c_str());

  // CLIPS overwrites the SIGINT hnadler, restore after
  struct sigaction oldact;

  if (sigaction(SIGINT, NULL, &oldact) == 0) {
    std::shared_ptr<CLIPS::Environment> new_env =
        std::make_shared<CLIPS::Environment>();

    LockSharedPtr<CLIPS::Environment> clips{std::move(new_env)};
    // // silent clips by default
    clips->unwatch("all");

    CLIPSContextMaintainer *cm =
        new CLIPSContextMaintainer(log_component_name.c_str());

    void *env = clips->cobj();

    SetEnvironmentContext(env, cm);

    EnvAddRouterWithContext(env, (char *)ROUTNER_NAME, /*router priority*/
                            30, log_router_query, log_router_print, NULL, NULL,
                            log_router_exit, &cm->logger);

    sigaction(SIGINT, &oldact, NULL);

    RCLCPP_INFO(get_logger(), "Initialisied new CLIPS ENVIRONMENT: %s",
              log_component_name.c_str());
    return clips;
  } else {
    RCLCPP_ERROR(
        get_logger(),
        ("CLIPS: Unable to backup --- SIGINT sigaction for restoration."));
    LockSharedPtr<CLIPS::Environment> clips{};
    return clips;
  }
}

void CLIPSEnvManagerNode::assert_features(
    LockSharedPtr<CLIPS::Environment> &clips, bool immediate_assert) {

  RCLCPP_WARN(get_logger(), "Asserting features!");

  // deffact so it survives a reset
  std::string deffacts = "(deffacts ff-features-available";

  for (const auto &feat : features_set) {
    deffacts += " (ff-feature " + feat + ")";
    if (immediate_assert) {
      clips->assert_fact_f("(ff-feature %s)", feat.c_str());
    }
  }
  deffacts += ")";

  if (!clips->build(deffacts)) {
    RCLCPP_WARN(get_logger(), "Failed to build deffacts ff-features-available");
  }
}

void CLIPSEnvManagerNode::guarded_load(const std::string &env_name,
                                       const std::string &filename) {
  if (envs_.find(env_name) == envs_.end()) {
    RCLCPP_ERROR(get_logger(),
                 "guarded_load: env %s has not been registered --> Should "
                 "be signaled! (e.g. as exception)",
                 env_name.c_str());
  }

  LockSharedPtr<CLIPS::Environment> &clips = envs_[env_name].env;

  int load_rv = 0;
  if ((load_rv = clips->load(filename)) != 1) {
    if (load_rv == 0) {
      // destroy_env(env_name);
      RCLCPP_ERROR(get_logger(),
                   "guarded_load: %s can't find %s --> Should "
                   "throw exception later",
                   env_name.c_str(), filename.c_str());
    } else {
      // destroy_env(env_name);
      RCLCPP_ERROR(get_logger(),
                   "guarded_load: %s: CLIPS code error in %s --> Should "
                   "throw exception later",
                   env_name.c_str(), filename.c_str());
    }
  } else {
    RCLCPP_INFO(get_logger(), "guarded_load: Loaded file %s for environment %s",
                filename.c_str(), env_name.c_str());
  }
}

void CLIPSEnvManagerNode::add_functions(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {

  clips->add_function(
      "ff-feature-request",
      sigc::slot<CLIPS::Value, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this, &CLIPSEnvManagerNode::clips_request_feature),
          env_name)));
  clips->add_function("now", sigc::slot<CLIPS::Values>(sigc::mem_fun(
                                 *this, &CLIPSEnvManagerNode::clips_now)));
  clips->add_function("now-systime",
                      sigc::slot<CLIPS::Values>(sigc::mem_fun(
                          *this, &CLIPSEnvManagerNode::clips_now_systime)));
  // TODO IMPLEMENT MORE!
}

CLIPS::Value
CLIPSEnvManagerNode::clips_request_feature(const std::string &env_name,
                                           const std::string &feature_name) {
  bool rv = true;

  RCLCPP_INFO(get_logger(), "Environment %s requests feature %s",
              env_name.c_str(), feature_name.c_str());
  // Check if ENv exists
  if (envs_.find(env_name) == envs_.end()) {
    RCLCPP_WARN(get_logger(),
                "Feature %s request from non-existent environment %s",
                feature_name.c_str(), env_name.c_str());
    return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
  }
  // Check if the feature is available
  if (features_set.find(feature_name) == features_set.end()) {
    RCLCPP_WARN(get_logger(), "Environment %s requested unavailable feature %s",
                env_name.c_str(), feature_name.c_str());
    return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
  }

  ClipsEnvData &envd = envs_[env_name];
  // Check if features was already requested
  if (std::binary_search(envd.req_feat.begin(), envd.req_feat.end(),
                         feature_name)) {
    RCLCPP_WARN(get_logger(), "Environment %s requested feature %s *again*",
                env_name.c_str(), feature_name.c_str());
    return CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL);
  }

  call_feature_context_initialisation(env_name, feature_name);
  envd.req_feat.push_back(feature_name);
  envd.req_feat.sort();

  // deffacts to sruvive reset
  std::string deffacts = "(deffacts ff-features-loaded";

  for (const auto &feat : envd.req_feat) {
    deffacts += " (ff-feature-loaded " + feat + ")";
  }

  deffacts += ")";

  envd.env->assert_fact_f("(ff-feature-loaded %s)", feature_name.c_str());

  if (!envd.env->build(deffacts)) {
    RCLCPP_WARN(get_logger(),
                "Failed to build deffacts ff-features-loaded for %s",
                env_name.c_str());
    rv = FALSE;
  }

  return CLIPS::Value(rv ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL);
}

void CLIPSEnvManagerNode::call_feature_context_initialisation(
    const std::string &env_name, const std::string &feature_name) {

  while (!init_feature_context_client->wait_for_service(5s)) {
    if (!rclcpp::ok()) {

      RCLCPP_ERROR(get_logger(),
                   "%s: timed out waiting for service availability",
                   init_feature_context_client->get_service_name());
      return;
    }
    RCLCPP_WARN(get_logger(), "%s: still waiting for service...",
                init_feature_context_client->get_service_name());
  }

  auto req = std::make_shared<cx_msgs::srv::ClipsFeatureContext::Request>();
  req->env_name = env_name;
  req->feature_name = feature_name;

  auto future_res = init_feature_context_client->async_send_request(req);

  // WAIT FOR THE FUTURE AS SPINNING WOULD NEED TO BE HAVE OWN THREAD
  if (future_res.wait_for(10s) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "%s: timed out waiting for response!",
                 init_feature_context_client->get_service_name());
    return;
  }

  if (!future_res.get()->success) {
    RCLCPP_ERROR(get_logger(), "ERROR --- %s", future_res.get()->error.c_str());
  }
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
                 init_feature_context_client->get_service_name());
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

CLIPS::Values CLIPSEnvManagerNode::clips_now() {
  CLIPS::Values rv;
  rv.push_back(get_clock()->now().seconds());
  return rv;
}

CLIPS::Values CLIPSEnvManagerNode::clips_now_systime() {
  CLIPS::Values rv;
  using namespace std::chrono;
  // get system seconds
  auto now = time_point_cast<seconds>(system_clock::now());
  rv.push_back(now.time_since_epoch().count());
  return rv;
}

} // namespace cx
