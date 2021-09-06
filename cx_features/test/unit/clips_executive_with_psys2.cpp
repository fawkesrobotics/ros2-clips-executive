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
// DOEST NOT TEST ANYTHING RIGHT NOW
TEST(cx_with_psys2, clips_generate_plan) {
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
  auto manager_client = std::make_shared<cx::CLIPSEnvManagerClient>();
  auto features_manager = std::make_shared<cx::ClipsFeaturesManager>();
  auto clips_executive_node = std::make_shared<cx::ClipsExecutive>();

  std::vector<std::string> allFeatures = {"plansys2"};
  features_manager->set_parameter(
      rclcpp::Parameter("clips_features", allFeatures));
  features_manager->declare_parameter("plansys2.plugin");
  features_manager->set_parameter(
      rclcpp::Parameter("plansys2.plugin", "cx::Plansys2Feature"));
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

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

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

  auto clips = clips_env_manager_node->getEnvironmentByName(env_name);
  clips.scopedLock();
  // clips->evaluate("(psys2-add-domain-instance \"leia\" \"robot\")");
  // clips->evaluate("(psys2-add-domain-instance \"francisco\" \"person\")");
  // clips->evaluate("(psys2-add-domain-instance \"message1\" \"message\")");
  // clips->evaluate("(psys2-add-domain-instance \"bedroom\" \"room\")");
  // clips->evaluate("(psys2-add-domain-instance \"kitchen\" \"room\")");
  // clips->evaluate("(psys2-add-domain-instance \"corridor\" \"room\")");

  // clips->evaluate(
  //     "(psys2-add-domain-predicate \"(robot_at\" \"leia kitchen)\")");
  // clips->evaluate(
  //     "(psys2-add-domain-predicate \"(person_at\" \"francisco bedroom)\")");

  clips->assert_fact_f("(domain-object (name leia) (type robot))");
  clips->assert_fact_f("(domain-object (name reia) (type robot))");
  clips->assert_fact_f("(domain-object (name francisco) (type person))");
  clips->assert_fact_f("(domain-object (name message1) (type message))");
  clips->assert_fact_f("(domain-object (name message2) (type message))");
  clips->assert_fact_f("(domain-object (name bedroom) (type room))");
  clips->assert_fact_f("(domain-object (name kitchen) (type room))");
  clips->assert_fact_f("(domain-object (name corridor) (type room))");

  clips->assert_fact_f(
      "(domain-fact (name robot_at) (param-values leia kitchen))");
  clips->assert_fact_f(
      "(domain-fact (name person_at) (param-values francisco bedroom))");
  clips->evaluate("(pddl-request-plan \"TEST-PSYS\" \"(and (robot_talk leia message1 francisco))\")");
  clips->evaluate("(pddl-request-plan \"TEST-PSYS\" \"(and (robot_talk reia message2 francisco))\")");
  
  // clips->evaluate("(psys2-clear-knowledge)");

  finish = true;
  t.join();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
