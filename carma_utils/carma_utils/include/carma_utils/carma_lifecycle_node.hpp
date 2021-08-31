// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CARMA_UTILS__CARMA_LIFECYCLE_NODE_HPP_
#define CARMA_UTILS__CARMA_LIFECYCLE_NODE_HPP_

#include <memory>
#include <string>
#include <thread>

#include "carma_utils/visibility_control.hpp"
#include "cav_msgs/msg/system_alert.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/node_thread.hpp"

namespace carma_utils
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CarmaLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  CARMA_UTILS_PUBLIC
  explicit CarmaLifecycleNode(const rclcpp::NodeOptions & options);
  virtual ~CarmaLifecycleNode();

  carma_utils::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/) override;
  carma_utils::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/) override;
  carma_utils::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/) override;
  carma_utils::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/) override;

  std::shared_ptr<carma_utils::CarmaLifecycleNode> shared_from_this();

  void publish_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg);
  virtual void on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg);

protected:
  rclcpp::Node::SharedPtr rclcpp_node_;
  std::unique_ptr<ros2_utils::NodeThread> rclcpp_thread_;
  void create_rclcpp_node(const rclcpp::NodeOptions & options);

  const std::string system_alert_topic_{"/system_alert"};
  rclcpp::Subscription<cav_msgs::msg::SystemAlert>::SharedPtr system_alert_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<cav_msgs::msg::SystemAlert>>
  system_alert_pub_;
};

}  // namespace carma_utils

#endif  // CARMA_UTILS__CARMA_LIFECYCLE_NODE_HPP_"
