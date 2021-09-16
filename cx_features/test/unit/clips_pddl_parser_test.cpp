#include "ament_index_cpp/get_package_share_directory.hpp"

#include <rclcpp/rclcpp.hpp>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/param.h"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_pddl_parser/Utils.h"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"

#include "cx_clips/CLIPSEnvManagerClient.hpp"
#include "cx_clips/CLIPSEnvManagerNode.h"
#include "cx_clips_executive/ClipsExecutive.hpp"
#include "cx_features/ClipsFeaturesManager.hpp"

#include "gtest/gtest.h"

TEST(clips_pddl_parser_test, clips_generate_plan) {
  auto test_node = rclcpp::Node::make_shared("test_node");
  // PSYS2
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  // CX
  auto clips_env_manager_node = std::make_shared<cx::CLIPSEnvManagerNode>();
  auto manager_client =
      std::make_shared<cx::CLIPSEnvManagerClient>("clips_manager_client");
  auto features_manager = std::make_shared<cx::ClipsFeaturesManager>();
  auto clips_executive_node = std::make_shared<cx::ClipsExecutive>();

  std::vector<std::string> allFeatures = {"clips_pddl_parser", "plansys2"};
  features_manager->set_parameter(
      rclcpp::Parameter("clips_features", allFeatures));
  features_manager->declare_parameter("plansys2.plugin");
  features_manager->declare_parameter("clips_pddl_parser.plugin");
  features_manager->set_parameter(rclcpp::Parameter(
      "clips_pddl_parser.plugin", "cx::ClipsPddlParserFeature"));
  features_manager->set_parameter(
      rclcpp::Parameter("plansys2.plugin", "cx::Plansys2Feature"));

  features_manager->declare_parameter("spec");
  clips_executive_node->set_parameter(rclcpp::Parameter("spec", "test"));
  features_manager->pre_configure(clips_env_manager_node);
  clips_executive_node->pre_configure(clips_env_manager_node);

  std::string pkgpath =
      ament_index_cpp::get_package_share_directory("cx_clips_executive");

  domain_node->set_parameter(
      {"model_file", pkgpath + "/pddl/domain_simple.pddl"});
  problem_node->set_parameter(
      {"model_file", pkgpath + "/pddl/domain_simple.pddl"});

  rclcpp::executors::SingleThreadedExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  exe.add_node(clips_env_manager_node->get_node_base_interface());
  exe.add_node(features_manager->get_node_base_interface());
  exe.add_node(clips_executive_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
    while (!finish) {
      exe.spin();
    }
  });

  domain_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  clips_env_manager_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  features_manager->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  clips_executive_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  clips_env_manager_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  features_manager->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  clips_executive_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  const std::string env_name = "executive";
  const std::string log_name = "(clips-executive)";
  RCLCPP_INFO(test_node->get_logger(), "TEST-NODE-START");

  auto pddl_file = std::move(pkgpath + "/pddl/domain.pddl");
  RCLCPP_INFO(test_node->get_logger(), "Pddl Dir: %s", pddl_file.c_str());
  auto clips = clips_env_manager_node->getEnvironmentByName(env_name);

//   std::lock_guard<std::recursive_mutex> guard(*(clips.get_mutex_instance()));
  //   clips->evaluate("(parse-pddl-domain \"" + pddl_file + "\")");

  clips->evaluate("(path-load \"parser.clp\")");
  clips->run();
  //   clips->evaluate("(psys2-clear-knowledge)");

  finish = true;
  t.join();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
