// Copyright (c) 2019 Intel Corporation
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
#include "cav_msgs/msg/system_alert.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/node_thread.hpp"
#include "carma_utils/visibility_control.h"

namespace carma_utils
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Common node functionality
// TODO: Make this a template that takes either Node or LifecycleNode
class CarmaNode : public rclcpp_lifecycle::LifecycleNode 
{
public:
  // TODO: Make this capable of being used as a ComposableNode
  CarmaNode(
    const std::string & node_name,
    const std::string & ns = "",
    bool use_rclcpp_node = false,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~CarmaNode();

  // Make the CarmaNode Composition capable 
  COMPOSITION_PUBLIC
  CarmaNode(const rclcpp::NodeOptions & options);

  std::shared_ptr<carma_utils::CarmaNode> shared_from_this();

  // TODO: BondPeer
  void create_bond();
  void destroy_bond();

  // TODO: SystemEvent
  void publish_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg);
  void handle_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg);

  // Spin with try catch block
  void spin();

protected:
  // Whether or not to create a local rclcpp::Node which can be used for ROS2 classes that
  // don't yet support lifecycle nodes
  bool use_rclcpp_node_{false};

  // System alert pub/sub
  static std::string system_alert_topic_;
  rclcpp::Subscription<cav_msgs::msg::SystemAlert>::SharedPtr system_alert_sub_;
  rclcpp::Publisher<cav_msgs::msg::SystemAlert>::SharedPtr system_alert_pub_;

  // The local rclcpp node and the thread to spin it
  rclcpp::Node::SharedPtr rclcpp_node_;
  std::unique_ptr<ros2_utils::NodeThread> rclcpp_thread_;

  // Bond connection for heartbeat messages
  std::unique_ptr<bond::Bond> bond_;
};

}  // namespace carma_utils

#endif  // CARMA_UTILS__CARMA_NODE_HPP_
