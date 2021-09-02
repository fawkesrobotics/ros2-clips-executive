#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "cx_clips/CLIPSEnvManagerClient.hpp"
#include "cx_clips_executive/ClipsExecutive.hpp"
#include "cx_utils/params_utils.hpp"

using namespace std::chrono_literals;

namespace cx {

ClipsExecutive::ClipsExecutive()
    : rclcpp_lifecycle::LifecycleNode("clips_executive") {

  RCLCPP_INFO(get_logger(), "Initialising [%s]...", get_name());

  declare_parameter("clips-dirs", clips_dirs);
  declare_parameter("spec", "");
  declare_parameter<bool>("assert-time-each-loop", false);
}

// ClipsExecutive::~ClipsExecutive() {}
void ClipsExecutive::pre_configure(
    std::shared_ptr<cx::CLIPSEnvManagerNode> &manager_node) {
  clips_env_manager_node_ = manager_node;
}

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
ClipsExecutive::on_configure(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Configuring [%s]...", get_name());

  env_manager_client_ = std::make_shared<cx::CLIPSEnvManagerClient>();
  clips_agenda_refresh_pub_ = create_publisher<std_msgs::msg::Empty>(
      "clips_executive/refresh_agenda", rclcpp::QoS(100).reliable());

  std::string cx_bringup_dir;
  try {
    cx_bringup_dir =
        std::move(ament_index_cpp::get_package_share_directory("cx_bringup"));
    clips_executive_share_dir = std::move(
        ament_index_cpp::get_package_share_directory("cx_clips_executive"));
  } catch (const std::exception &e) {
    RCLCPP_ERROR(
        get_logger(),
        "Exception when getting bringup/clips_executive shared directory!");
    std::cerr << e.what() << '\n';
    return CallbackReturn::FAILURE;
  }

  if (!(get_parameter("clips-dirs", clips_dirs))) {
    RCLCPP_ERROR(
        get_logger(),
        "Couldnt get parameter /clips_executive/clips-dirs, aborting...");
    return CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i < clips_dirs.size(); ++i) {
    if (clips_dirs[i][clips_dirs[i].size() - 1] != '/') {
      clips_dirs[i] += "/";
    }
    RCLCPP_INFO(get_logger(), "CLIPS_DIR: %s", clips_dirs[i].c_str());
  }

  clips_dirs.insert(clips_dirs.begin(), clips_executive_share_dir + "/clips/");

  std::string cfg_spec;
  get_parameter("spec", cfg_spec);
  if (cfg_spec == "") {
    RCLCPP_ERROR(get_logger(),
                 "Couldnt get parameter /clips_executive/spec, aborting...");
    return CallbackReturn::FAILURE;
  }

  std::string action_mapping_cfgpath = "specs." + cfg_spec + ".action-mapping";

  std::map<std::string, std::string> action_mapping{};

  try {
    YAML::Node config = YAML::LoadFile(
        std::move(cx_bringup_dir + "/params/clips_executive.yaml"));
    iterateThroughYamlRecuresively(config["clips_executive"], "action-mapping",
                                   "", cfg_spec, action_mapping);

  } catch (const std::exception &e) {
    RCLCPP_INFO(get_logger(), "Error loading clips_executive config file!");

    std::cerr << e.what() << '\n';
    return CallbackReturn::FAILURE;
  }

  if (action_mapping.begin() == action_mapping.end()) {
    RCLCPP_ERROR(get_logger(), "Couldnt get parameter %s, aborting...",
                 action_mapping_cfgpath.c_str());
    return CallbackReturn::FAILURE;
  }
  // Print all actions mappings
  for (const auto &action : action_mapping) {
    RCLCPP_INFO(get_logger(), "Adding action mapping %s->%s",
                action.first.c_str(), action.second.c_str());
  }
  action_skill_mapping_ =
      std::make_shared<cx::ActionSkillMapping>(action_mapping);

  RCLCPP_INFO(get_logger(), "Configured [%s]!", get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ClipsExecutive::on_activate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating [%s]...", get_name());
  clips_agenda_refresh_pub_->on_activate();
  // Create Clips Executive environment
  if (env_manager_client_->createNewClipsEnvironment("executive",
                                                     "CLIPS (executive)")) {
    clips_ = clips_env_manager_node_->envs_["executive"].env;
    // TODO: REMOVE LATER!
    clips_->watch("all");
  } else {
    RCLCPP_ERROR(get_logger(),
                 "Clips Executive Environment creation failed, aborting...");
    return CallbackReturn::FAILURE;
  }
  clips_.scopedLock();
  std::string cx_bringup_dir;
  try {
    cx_bringup_dir =
        std::move(ament_index_cpp::get_package_share_directory("cx_bringup"));

    clips_->evaluate(std::string("(path-add-subst \"@CONFDIR@\" \"") +
                     cx_bringup_dir + "\")");

  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i < clips_dirs.size(); ++i) {
    clips_->evaluate("(path-add \"" + clips_dirs[i] + "\")");
  }

  clips_->evaluate("(ff-feature-request \"config_feature\")");

  clips_->add_function(
      "map-action-skill",
      sigc::slot<std::string, std::string, CLIPS::Values, CLIPS::Values>(
          sigc::mem_fun(*this, &ClipsExecutive::clips_map_skill)));

  clips_->evaluate("(ff-feature-request \"redefine_warning_feature\")");

  std::vector<std::string> files{clips_executive_share_dir +
                                     "/clips/saliences.clp",
                                 clips_executive_share_dir + "/clips/init.clp"};
  for (const auto &f : files) {
    if (!clips_->batch_evaluate(f)) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to initialize CLIPS environment, "
                   "batch file '%s' failed!, aborting...",
                   f.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  clips_->assert_fact("(executive-init)");
  clips_->refresh_agenda();
  clips_->run();

  // Verify that initialization did not fail (yet)
  {
    CLIPS::Fact::pointer fact = clips_->get_facts();
    while (fact) {
      CLIPS::Template::pointer tmpl = fact->get_template();
      if (tmpl->name() == "executive-init-stage") {
        CLIPS::Values v = fact->slot_value("");
        if (v.size() > 0 && v[0].as_string() == "FAILED") {
          RCLCPP_ERROR(get_logger(), "CLIPS Executive initialization failed");
          return CallbackReturn::FAILURE;
        }
      }
      fact = fact->next();
    }
  }
  RCLCPP_INFO(get_logger(), "CLIPS Executive was inistialised!");

  agenda_refresh_timer_ = create_wall_timer(33ms, [this]() {
    clips_.scopedLock();
    if (cfg_assert_time_each_cycle_) {

      // clips_->assert_fact("(time (now))");
    }
    clips_->refresh_agenda();
    clips_->run();
    clips_agenda_refresh_pub_->publish(std_msgs::msg::Empty());
  });
  RCLCPP_INFO(get_logger(), "Activared [%s]!", get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ClipsExecutive::on_deactivate(const rclcpp_lifecycle::State &state) {

  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  clips_agenda_refresh_pub_->on_deactivate();
  clips_->assert_fact("(executive-finalize)");
  clips_->refresh_agenda();
  clips_->run();
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());

  return CallbackReturn::SUCCESS;
}

std::string ClipsExecutive::clips_map_skill(std::string action_name,
                                            CLIPS::Values param_names,
                                            CLIPS::Values param_values) {
  if (!action_skill_mapping_) {
    RCLCPP_ERROR(get_logger(), "No action mapping has been loaded");
    return "";
  }
  if (action_name == "") {
    RCLCPP_WARN(get_logger(), "Failed to map, action name is empty");
    return "";
  }
  if (!action_skill_mapping_->has_mapping(action_name)) {
    RCLCPP_WARN(get_logger(), "No mapping for action '%s' known",
                action_name.c_str());
    return "";
  }
  if (param_names.size() != param_values.size()) {
    RCLCPP_WARN(get_logger(),
                "Number of parameter names and values "
                "do not match for action '%s'",
                action_name.c_str());
    return "";
  }
  std::map<std::string, std::string> param_map;
  for (size_t i = 0; i < param_names.size(); ++i) {
    if (param_names[i].type() != CLIPS::TYPE_SYMBOL &&
        param_names[i].type() != CLIPS::TYPE_STRING) {
      RCLCPP_ERROR(get_logger(), "Param for '%s' is not a string or symbol",
                   action_name.c_str());
      return "";
    }
    switch (param_values[i].type()) {
    case CLIPS::TYPE_FLOAT:
      param_map[param_names[i].as_string()] =
          std::to_string(param_values[i].as_float());
      break;
    case CLIPS::TYPE_INTEGER:
      param_map[param_names[i].as_string()] =
          std::to_string(param_values[i].as_integer());
      break;
    case CLIPS::TYPE_SYMBOL:
    case CLIPS::TYPE_STRING:
      param_map[param_names[i].as_string()] = param_values[i].as_string();
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Param '%s' for action '%s' of invalid type",
                   param_names[i].as_string().c_str(), action_name.c_str());
      break;
    }
  }

  std::multimap<std::string, std::string> messages;
  std::string rv =
      action_skill_mapping_->map_skill(action_name, param_map, messages);
  for (auto &m : messages) {
    if (m.first == "WARN") {
      RCLCPP_WARN(get_logger(), "%s", m.second.c_str());
    } else if (m.first == "ERROR") {
      RCLCPP_ERROR(get_logger(), "%s", m.second.c_str());
    } else if (m.first == "DEBUG") {
      RCLCPP_DEBUG(get_logger(), "%s", m.second.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "%s", m.second.c_str());
    }
  }
  return rv;
}

void ClipsExecutive::iterateThroughYamlRecuresively(
    const YAML::Node &current_level_node, const std::string &node_to_search,
    const std::string &parent_node_name, const std::string &cfg_prefix,
    std::map<std::string, std::string> &output_map) {
  for (const auto &item : current_level_node) {
    switch (item.second.Type()) {

    case YAML::NodeType::Undefined: {
      RCLCPP_ERROR(get_logger(), "Undefined YAML KEY");
      break;
    }
    case YAML::NodeType::Null: {
      RCLCPP_ERROR(get_logger(), "NULL YAML KEY");
      break;
    }
    case YAML::NodeType::Scalar: {
      // skip
      break;
    }
    case YAML::NodeType::Sequence: {
      // skip
      break;
    }
    // If it is a MapNode
    case YAML::NodeType::Map: {
      if (item.first.as<std::string>() == node_to_search &&
          parent_node_name == cfg_prefix) {

        for (const auto &map_values : current_level_node[item.first]) {
          output_map.insert({map_values.first.as<std::string>(),
                             map_values.second.as<std::string>()});
          RCLCPP_INFO(get_logger(), "Key %s <------> Value: %s",
                      map_values.first.as<std::string>().c_str(),
                      output_map[map_values.first.as<std::string>()].c_str());
        }
      } else {
        iterateThroughYamlRecuresively(
            current_level_node[item.first], node_to_search,
            item.first.as<std::string>(), cfg_prefix, output_map);
      }
      break;
    }
    }
  }
}

} // namespace cx