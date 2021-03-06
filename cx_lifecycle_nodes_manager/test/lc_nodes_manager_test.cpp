/***************************************************************************
 *  lc_nodes_manager_test.cpp
 *
 *  Created: 14 July 2021
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

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"

#include "cx_clips/CLIPSEnvManagerClient.hpp"
#include "cx_clips/CLIPSEnvManagerNode.h"
#include "cx_features/ClipsFeaturesManager.hpp"
#include "cx_features/MockFeature.hpp"
#include "cx_lifecycle_nodes_manager/LifecycleNodesManager.hpp"
namespace cx {

TEST(lc_nodes_manager_test, test_context_init) {
  auto testing_node = rclcpp::Node::make_shared("testing_node");
  auto manager_node = std::make_shared<cx::CLIPSEnvManagerNode>();
  auto manager_client = std::make_shared<cx::CLIPSEnvManagerClient>("clips_manager_client");
  auto features_manager = std::make_shared<cx::ClipsFeaturesManager>();
  auto lc_manager = std::make_shared<cx::LifecycleNodesManager>();

  std::vector<std::string> allFeatures = {"mock_feature"};
  features_manager->set_parameter(
      rclcpp::Parameter("clips_features", allFeatures));
  features_manager->declare_parameter("mock_feature.plugin");
  features_manager->set_parameter(
      rclcpp::Parameter("mock_feature.plugin", "cx::MockFeature"));
  // features_manager->set_parameter(
  //     rclcpp::Parameter("mock_feature.plugin", "cx::MockFeature"));

  features_manager->pre_configure(manager_node);

  std::vector<std::string> flist;

  const std::string env_name = "executive";
  const std::string log_name = "(clips-executive)";
  // std::string pkgPath =
  // ament_index_cpp::get_package_share_directory("cx_clips");

  rclcpp::executors::SingleThreadedExecutor exe;

  exe.add_node(lc_manager->get_node_base_interface());
  exe.add_node(manager_node->get_node_base_interface());
  exe.add_node(features_manager->get_node_base_interface());

  bool finish_exec = false;
  std::thread t([&]() {
    while (!finish_exec) {
      exe.spin();
    }
  });

  using lc_tr = lifecycle_msgs::msg::Transition;
  {
    rclcpp::Rate rate(10);
    auto start_time = testing_node->now();
    while ((testing_node->now() - start_time).seconds() < 0.5) {
      rate.sleep();
    }
  }
  // populate the vector with feature names for the req
  for (auto feat : features_manager->features_) {
    const std::string &feat_name = feat.second->getFeatureName();
    flist.push_back(feat_name);
  }

  manager_node->trigger_transition(lc_tr::TRANSITION_ACTIVATE);
  features_manager->trigger_transition(lc_tr::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start_time = testing_node->now();
    while ((testing_node->now() - start_time).seconds() < 0.5) {
      rate.sleep();
    }
  }

  // lc_manager->initialise();

  RCLCPP_INFO(testing_node->get_logger(), "TEST-NODE-START");

  // auto created_env = manager_node->getEnvironmentByName(env_name);
  try {

    ASSERT_TRUE(manager_client->createNewClipsEnvironment(env_name, log_name));

    auto created_env = manager_node->getEnvironmentByName(env_name);

    RCLCPP_INFO(testing_node->get_logger(), "Before Features Addition!");

    features_manager->clips_env_manager_node_->getEnvironmentByName(env_name)
        ->evaluate("(ff-feature-request \"mock_feature\")");

    // created_env->evaluate("(ff-feature-request \"mock_feature_2\")");
    // created_env->evaluate("(ff-feature-request \"mock_feature_3\")");
    // created_env->evaluate("(ff-feature-request \"mock_feature_4\")");
    // created_env->evaluate("(ff-feature-request \"mock_feature_10\")");
    // created_env->evaluate("(ff-feature-request \"mock_feature_1\")");

    ASSERT_FALSE(manager_client->assertCanRemoveClipsFeatures(flist));
    // mock_feature->clips_context_init(env_name, created_env);

    ASSERT_TRUE(manager_client->destroyClipsEnvironment(env_name));

    ASSERT_TRUE(manager_client->assertCanRemoveClipsFeatures(flist));
    ASSERT_TRUE(manager_client->removeClipsFeatures(flist));

  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }

  RCLCPP_INFO(testing_node->get_logger(), "TEST-NODE-END");

  finish_exec = true;
  t.join();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
} // namespace cx
