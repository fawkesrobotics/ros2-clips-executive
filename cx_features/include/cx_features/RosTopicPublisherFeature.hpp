/***************************************************************************
 *  RosTopicPublisherFeature.hpp
 *
 *  Created: November 13th, 2023
 *  Copyright  2023 Daniel Swoboda
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

#ifndef CX_FEATURES__CLIPSPROTOBUFFEATURE_HPP_
#define CX_FEATURES__CLIPSPROTOBUFFEATURE_HPP_

#include <map>
#include <memory>
#include <string>

#include "cx_core/ClipsFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"
#include "cx_utils/NodeThread.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace cx {

class RosTopicPublisherFeature : public ClipsFeature, public rclcpp::Node {
public:
  RosTopicPublisherFeature();
  ~RosTopicPublisherFeature();

  void initialise(const std::string &feature_name) override;

  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips) override;
  bool clips_context_destroyed(const std::string &env_name) override;

  std::string getFeatureName() const;

private:
  std::string msg_name_ = "default";
  std::map<std::string, LockSharedPtr<CLIPS::Environment>> envs_;
  std::thread spin_thread_;
  std::map<std::string,
           std::map<std::string,
                    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr>>
      publishers_;
  std::map<std::string, std::map<std::string, std_msgs::msg::String>> messages_;

  void create_message(const std::string &env_name, const std::string &topic);

  void set_field_publish(const std::string &env_name, const std::string &topic,
                         const std::string &field, CLIPS::Value value);

  void set_array_publish(const std::string &env_name, const std::string &topic,
                         const std::string &field, CLIPS::Values values);

  void publish_to_topic(const std::string &env_name,
                        const std::string &topic_name);

  void creater_publisher(const std::string &env_name,
                         const std::string &topic_name);
};

} // namespace cx
#endif // !CX_FEATURES__CLIPSPROTOBUFFEATURE_HPP_
