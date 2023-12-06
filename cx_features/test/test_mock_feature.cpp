#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "cx_clips/CLIPSEnvManagerClient.hpp"
#include "cx_clips/CLIPSEnvManagerNode.h"


#include "cx_features/ClipsFeaturesManager.hpp"
#include "cx_features/MockFeature.hpp"

// TEST(test_mock_feature, test_context_init) {
//   auto testing_node = rclcpp::Node::make_shared("testing_node");
//   auto manager_node = std::make_shared<cx::CLIPSEnvManagerNode>();
//   auto manager_client = std::make_shared<cx::CLIPSEnvManagerClient>("clips_manager_client");
//   auto features_manager = std::make_shared<cx::ClipsFeaturesManager>();

//   std::vector<std::string> allFeatures = {"mock_feature"};
//   features_manager->set_parameter(
//       rclcpp::Parameter("clips_features", allFeatures));
//   features_manager->declare_parameter("mock_feature.plugin");
//   features_manager->set_parameter(
//       rclcpp::Parameter("mock_feature.plugin", "cx::MockFeature"));
//   // features_manager->set_parameter(
//   //     rclcpp::Parameter("mock_feature.plugin", "cx::MockFeature"));

//   features_manager->pre_configure(manager_node);

//   std::vector<std::string> flist;

//   const std::string env_name = "executive";
//   const std::string log_name = "(clips-executive)";
//   // std::string pkgPath =
//   // ament_index_cpp::get_package_share_directory("cx_clips");

//   rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 2);

//   exe.add_node(manager_node->get_node_base_interface());
//   exe.add_node(features_manager->get_node_base_interface());

//   bool finish_exec = false;
//   std::thread t([&]() {
//     while (!finish_exec) {
//       exe.spin();
//     }
//   });

//   using lc_tr = lifecycle_msgs::msg::Transition;

//   manager_node->trigger_transition(lc_tr::TRANSITION_CONFIGURE);
//   features_manager->trigger_transition(lc_tr::TRANSITION_CONFIGURE);

//   {
//     rclcpp::Rate rate(10);
//     auto start_time = testing_node->now();
//     while ((testing_node->now() - start_time).seconds() < 0.5) {
//       rate.sleep();
//     }
//   }
//   // populate the vector with feature names for the req
//   for (auto feat : features_manager->features_) {
//     const std::string &feat_name = feat.second->getFeatureName();
//     flist.push_back(feat_name);
//   }

//   manager_node->trigger_transition(lc_tr::TRANSITION_ACTIVATE);
//   features_manager->trigger_transition(lc_tr::TRANSITION_ACTIVATE);

//   {
//     rclcpp::Rate rate(10);
//     auto start_time = testing_node->now();
//     while ((testing_node->now() - start_time).seconds() < 0.5) {
//       rate.sleep();
//     }
//   }

//   RCLCPP_INFO(testing_node->get_logger(), "TEST-NODE-START");

//   // auto created_env = manager_node->getEnvironmentByName(env_name);
//   try {

//     // ASSERT_TRUE(manager_client->addFeatures(flist));

//     ASSERT_FALSE(manager_client->addFeatures(flist));

//     ASSERT_TRUE(manager_client->createNewClipsEnvironment(env_name,
//     log_name));

//     auto created_env = manager_node->getEnvironmentByName(env_name);

//     RCLCPP_INFO(testing_node->get_logger(), "Before Features Addition!");

//     features_manager->clips_env_manager_node_->getEnvironmentByName(env_name)
//         ->evaluate("(ff-feature-request \"mock_feature\")");
//     features_manager->clips_env_manager_node_->getEnvironmentByName(env_name)
//         ->evaluate("(ff-feature-request \"redefine_warning_feature\")");
//     features_manager->clips_env_manager_node_->getEnvironmentByName(env_name)
//         ->evaluate("(ff-feature-request \"config_feature\")");
//     features_manager->clips_env_manager_node_->getEnvironmentByName(env_name)
//         ->evaluate("(config-load \"/clips_executive\")");

//     // created_env->evaluate("(ff-feature-request \"mock_feature_2\")");
//     // created_env->evaluate("(ff-feature-request \"mock_feature_3\")");
//     // created_env->evaluate("(ff-feature-request \"mock_feature_4\")");
//     // created_env->evaluate("(ff-feature-request \"mock_feature_10\")");
//     // created_env->evaluate("(ff-feature-request \"mock_feature_1\")");

//     ASSERT_FALSE(manager_client->assertCanRemoveClipsFeatures(flist));
//     // mock_feature->clips_context_init(env_name, created_env);

//     ASSERT_TRUE(manager_client->destroyClipsEnvironment(env_name));

//     ASSERT_TRUE(manager_client->assertCanRemoveClipsFeatures(flist));
//     ASSERT_TRUE(manager_client->removeClipsFeatures(flist));

//   } catch (const std::exception &e) {
//     std::cerr << e.what() << '\n';
//   }

//   RCLCPP_INFO(testing_node->get_logger(), "TEST-NODE-END");

//   finish_exec = true;
//   t.join();
// }

// TEST(test_mock_feature, test_bb_feature) {
//   auto testing_node = rclcpp::Node::make_shared("testing_node");
//   auto manager_node = std::make_shared<cx::CLIPSEnvManagerNode>();
//   auto manager_client = std::make_shared<cx::CLIPSEnvManagerClient>("clips_manager_client");
//   auto bb_node = std::make_shared<cx::Blackboard>();
//   auto features_manager = std::make_shared<cx::ClipsFeaturesManager>();

//   features_manager->pre_configure(manager_node);

//   std::vector<std::string> flist;

//   const std::string env_name = "executive";
//   const std::string log_name = "(clips-executive)";
//   // std::string pkgPath =
//   // ament_index_cpp::get_package_share_directory("cx_clips");

//   rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 3);

//   exe.add_node(manager_node->get_node_base_interface());
//   exe.add_node(bb_node->get_node_base_interface());
//   exe.add_node(features_manager->get_node_base_interface());

//   bool finish_exec = false;
//   std::thread t([&]() {
//     while (!finish_exec) {
//       exe.spin();
//     }
//   });

//   using lc_tr = lifecycle_msgs::msg::Transition;

//   manager_node->trigger_transition(lc_tr::TRANSITION_CONFIGURE);
//   bb_node->trigger_transition(lc_tr::TRANSITION_CONFIGURE);
//   features_manager->trigger_transition(lc_tr::TRANSITION_CONFIGURE);

//   {
//     rclcpp::Rate rate(10);
//     auto start_time = testing_node->now();
//     while ((testing_node->now() - start_time).seconds() < 0.5) {
//       rate.sleep();
//     }
//   }
//   // populate the vector with feature names for the req
//   for (auto feat : features_manager->features_) {
//     const std::string &feat_name = feat.second->getFeatureName();
//     flist.push_back(feat_name);
//   }

//   manager_node->trigger_transition(lc_tr::TRANSITION_ACTIVATE);
//   bb_node->trigger_transition(lc_tr::TRANSITION_ACTIVATE);
//   features_manager->trigger_transition(lc_tr::TRANSITION_ACTIVATE);

//   {
//     rclcpp::Rate rate(10);
//     auto start_time = testing_node->now();
//     while ((testing_node->now() - start_time).seconds() < 0.5) {
//       rate.sleep();
//     }
//   }

//   RCLCPP_INFO(testing_node->get_logger(), "TEST-NODE-START");

//   // auto created_env = manager_node->getEnvironmentByName(env_name);
//   try {

//     ASSERT_TRUE(manager_client->createNewClipsEnvironment(env_name, log_name));

//     auto created_env = manager_node->getEnvironmentByName(env_name);
//     auto bb_feature = features_manager->features_["blackboard_feature"];

//     RCLCPP_INFO(testing_node->get_logger(), "Before Features Addition!");

//     features_manager->clips_env_manager_node_->getEnvironmentByName(env_name)
//         ->evaluate("(ff-feature-request \"redefine_warning_feature\")");
//     features_manager->clips_env_manager_node_->getEnvironmentByName(env_name)
//         ->evaluate("(ff-feature-request \"config_feature\")");
//     features_manager->clips_env_manager_node_->getEnvironmentByName(env_name)
//         ->evaluate("(ff-feature-request \"blackboard_feature\")");
//     features_manager->clips_env_manager_node_->getEnvironmentByName(env_name)
//         ->evaluate("(config-load \"/clips_executive\")");

//     created_env->evaluate(
//         "(blackboard-open-reading \"PddlGenInterface\" \"pddl_gen\")");

//     created_env->evaluate(
//         "(blackboard-create-msg \"PddlGenInterface::pddl_gen\" "
//         "\"GenerateMessage\")");

//     ASSERT_FALSE(manager_client->assertCanRemoveClipsFeatures(flist));
//     ASSERT_TRUE(manager_client->destroyClipsEnvironment(env_name));
//     ASSERT_TRUE(manager_client->assertCanRemoveClipsFeatures(flist));
//     ASSERT_TRUE(manager_client->removeClipsFeatures(flist));

//   } catch (const std::exception &e) {
//     std::cerr << e.what() << '\n';
//   }

//   RCLCPP_INFO(testing_node->get_logger(), "TEST-NODE-END");

//   finish_exec = true;
//   t.join();
// }

// int main(int argc, char **argv) {
//   // testing::InitGoogleTest(&argc, argv);
//   // rclcpp::init(argc, argv);
//   // return RUN_ALL_TESTS();
// }
