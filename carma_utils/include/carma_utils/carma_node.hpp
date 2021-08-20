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

#ifndef CARMA_UTILS__CARMA_NODE_HPP_
#define CARMA_UTILS__CARMA_NODE_HPP_

#include <memory>
#include <string>
#include <thread>

#include "bondcpp/bond.hpp"
#include "bond/msg/constants.hpp"
#include "carma_utils/visibility_control.hpp"
#include "cav_msgs/msg/system_alert.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/node_thread.hpp"

namespace carma_utils
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// TODO(@pmusau17): Make this a template that takes either Node or LifecycleNode

// Common node functionality


class CarmaNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  // A composition-capable CarmaNode


  CARMA_UTILS_PUBLIC
  CarmaNode(const rclcpp::NodeOptions & options);

  virtual ~CarmaNode();

  // TODO(mjeronimo): BondPeer
  void create_bond();
  void destroy_bond();

  void publish_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg);
  virtual void on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg);

  std::shared_ptr<carma_utils::CarmaNode> shared_from_this();

  // Spin with try catch block


  void spin();

protected:
  // Machinery to support ROS2 classes that don't yet support lifecycle nodes


  bool use_rclcpp_node_{false};            // Whether or not to create a local rclcpp::Node
  rclcpp::Node::SharedPtr rclcpp_node_;    // The rclcpp node
  std::unique_ptr<ros2_utils::NodeThread> rclcpp_thread_;  // The thread to spin it

  // Bond connection for heartbeat messages
  std::unique_ptr<bond::Bond> bond_;

  // System alert pub/sub
  const std::string system_alert_topic_{"/system_alert"};
  rclcpp::Subscription<cav_msgs::msg::SystemAlert>::SharedPtr system_alert_sub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<cav_msgs::msg::SystemAlert>>
  system_alert_pub_;
};

}  // namespace carma_utils

#endif  // CARMA_UTILS__CARMA_NODE_HPP_
