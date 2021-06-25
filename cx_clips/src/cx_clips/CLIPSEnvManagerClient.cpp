#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "cx_clips/CLIPSEnvManagerClient.hpp"

namespace cx {

using namespace std::chrono_literals;

CLIPSEnvManagerClient::CLIPSEnvManagerClient() {
  node_ = rclcpp::Node::make_shared("clips_manager_client");

  create_env_client_ = node_->create_client<cx_msgs::srv::CreateClipsEnv>(
      "clips_manager/create_env");

  destroy_env_client_ = node_->create_client<cx_msgs::srv::DestroyClipsEnv>(
      "clips_manager/destroy_env");

  add_clips_features_client_ =
      node_->create_client<cx_msgs::srv::AddClipsFeatures>(
          "clips_manager/add_clips_features");

  assert_can_remove_features_client_ =
      node_->create_client<cx_msgs::srv::ClipsRemoveFeatures>(
          "clips_manager/assert_can_remove_features");

  remove_features_client_ =
      node_->create_client<cx_msgs::srv::ClipsRemoveFeatures>(
          "clips_manager/remove_features");
}
// CLIPSEnvManagerClient::~CLIPSEnvManagerClient() {}

bool CLIPSEnvManagerClient::createNewClipsEnvironment(
    const std::string &env_name, const std::string &log_name) {

  while (!create_env_client_->wait_for_service(5s)) {
    if (!rclcpp::ok()) {

      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   create_env_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                create_env_client_->get_service_name());
  }

  auto req = std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
  req->env_name = env_name;
  req->log_name = log_name;

  auto future_res = create_env_client_->async_send_request(req);

  // HARD CODED 20 S -> maybe add as param!
  if (rclcpp::spin_until_future_complete(node_, future_res, 10s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "%s: timed out waiting for response!",
                 create_env_client_->get_service_name());
    return false;
  }

  if (future_res.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "ERROR --- %s",
                 future_res.get()->error.c_str());
    return false;
  }
}

bool CLIPSEnvManagerClient::destroyClipsEnvironment(
    const std::string &env_name) {

  while (!destroy_env_client_->wait_for_service(5s)) {
    if (!rclcpp::ok()) {

      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   destroy_env_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                destroy_env_client_->get_service_name());
  }

  auto req = std::make_shared<cx_msgs::srv::DestroyClipsEnv::Request>();
  req->env_name = env_name;

  auto future_res = destroy_env_client_->async_send_request(req);

  // HARD CODED 20 S -> maybe add as param!
  if (rclcpp::spin_until_future_complete(node_, future_res, 20s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "%s: timed out waiting for response!",
                 destroy_env_client_->get_service_name());
    return false;
  }

  if (future_res.get()->success) {
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "ERROR --- %s",
                 future_res.get()->error.c_str());
    return false;
  }
}

bool CLIPSEnvManagerClient::addFeatures(
    const std::vector<std::string> &features) {

  while (!add_clips_features_client_->wait_for_service(5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   add_clips_features_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                add_clips_features_client_->get_service_name());
  }

  auto req = std::make_shared<cx_msgs::srv::AddClipsFeatures::Request>();
  req->features = features;

  auto future_res = add_clips_features_client_->async_send_request(req);

  // HARD CODED 20 S -> maybe add as param!
  if (rclcpp::spin_until_future_complete(node_, future_res, 10s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "%s: timed out waiting for response!",
                 add_clips_features_client_->get_service_name());
    return false;
  }

  if (future_res.get()->success) {
    RCLCPP_INFO(node_->get_logger(), "Successfully added all features!");
    return true;
  } else {
    const std::vector<std::string> &failed_features =
        future_res.get()->failed_features;

    for (const auto &feat : failed_features) {

      RCLCPP_ERROR(node_->get_logger(), "Couldn not load feature %s",
                   feat.c_str());
    }
    return false;
  }
}

bool CLIPSEnvManagerClient::assertCanRemoveClipsFeatures(
    const std::vector<std::string> &features) {

  while (!assert_can_remove_features_client_->wait_for_service(5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   assert_can_remove_features_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                assert_can_remove_features_client_->get_service_name());
  }

  auto req = std::make_shared<cx_msgs::srv::ClipsRemoveFeatures::Request>();
  req->features = features;

  auto future_res = assert_can_remove_features_client_->async_send_request(req);

  // HARD CODED 20 S -> maybe add as param!
  if (rclcpp::spin_until_future_complete(node_, future_res, 15s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "%s: timed out waiting for response!",
                 assert_can_remove_features_client_->get_service_name());
    return false;
  }

  if (future_res.get()->success) {
    RCLCPP_INFO(node_->get_logger(), "All requested features can be removed!");
    return true;
  }

  return false;
}

bool CLIPSEnvManagerClient::removeClipsFeatures(
    const std::vector<std::string> &features) {

  while (!remove_features_client_->wait_for_service(5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   remove_features_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                remove_features_client_->get_service_name());
  }

  auto req = std::make_shared<cx_msgs::srv::ClipsRemoveFeatures::Request>();
  req->features = features;

  auto future_res = remove_features_client_->async_send_request(req);

  // HARD CODED 20 S -> maybe add as param!
  if (rclcpp::spin_until_future_complete(node_, future_res, 15s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "%s: timed out waiting for response!",
                 remove_features_client_->get_service_name());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "All requested features were removed!");
  return true;
}

} // namespace cx
