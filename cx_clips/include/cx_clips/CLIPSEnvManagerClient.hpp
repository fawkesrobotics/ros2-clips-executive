#ifndef CX_CLIPS__CLIPSENVMANAGERCLIENT_HPP_
#define CX_CLIPS__CLIPSENVMANAGERCLIENT_HPP_

#include <list>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "cx_msgs/srv/add_clips_features.hpp"
#include "cx_msgs/srv/clips_remove_features.hpp"
#include "cx_msgs/srv/create_clips_env.hpp"
#include "cx_msgs/srv/destroy_clips_env.hpp"

namespace cx {

class CLIPSEnvManagerClient {
public:
  CLIPSEnvManagerClient();
  // ~CLIPSEnvManagerClient();

  bool createNewClipsEnvironment(const std::string &env_name,
                                 const std::string &log_name);
  bool destroyClipsEnvironment(const std::string &env_name);
  bool addFeatures(const std::vector<std::string> &features);
  bool assertCanRemoveClipsFeatures(const std::vector<std::string> &features);
  bool removeClipsFeatures(const std::vector<std::string> &features);

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<cx_msgs::srv::AddClipsFeatures>::SharedPtr
      add_clips_features_client_;
  rclcpp::Client<cx_msgs::srv::CreateClipsEnv>::SharedPtr create_env_client_;
  rclcpp::Client<cx_msgs::srv::DestroyClipsEnv>::SharedPtr destroy_env_client_;
  rclcpp::Client<cx_msgs::srv::ClipsRemoveFeatures>::SharedPtr
      assert_can_remove_features_client_;
  rclcpp::Client<cx_msgs::srv::ClipsRemoveFeatures>::SharedPtr
      remove_features_client_;
  // destroy_env_client_;
};

} // namespace cx
#endif // !CX_CLIPS__CLIPSENVMANAGERCLIENT_HPP_
