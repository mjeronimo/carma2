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

#include "carma_utils/carma_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace carma_utils
{

#if 0
CarmaNode::CarmaNode(
  const std::string & node_name,
  const std::string & ns,
  bool use_rclcpp_node,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, ns, options),
  use_rclcpp_node_(use_rclcpp_node)
{
  // TODO(@pmusau17): Creation of pubs and sub should be in on_configure

  // Create a system alert publisher. The subscriber will be made by the child class
  system_alert_pub_ = create_publisher<cav_msgs::msg::SystemAlert>(system_alert_topic_, 10);

  // The server side never times out from lifecycle manager
  declare_parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true);
  set_parameter(rclcpp::Parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true));

  if (use_rclcpp_node_) {
    std::vector<std::string> new_args = options.arguments();
    new_args.push_back("--ros-args");
    new_args.push_back("-r");
    new_args.push_back(std::string("__node:=") + get_name() + "_rclcpp_node");
    new_args.push_back("--");
    rclcpp_node_ = std::make_shared<rclcpp::Node>(
      "_", ns, rclcpp::NodeOptions(options).arguments(new_args));
    rclcpp_thread_ = std::make_unique<ros2_utils::NodeThread>(rclcpp_node_);
  }

  RCLCPP_INFO(
    get_logger(),
    "Lifecycle node launched, waiting on state transition requests");
}
#endif

CarmaNode::CarmaNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("carma_node", "", options)
{
  // TODO(@pmusau17): Creation of pubs and sub should be in on_configure

  // Create a system alert publisher. The subscriber will be made by the child class
  system_alert_pub_ = create_publisher<cav_msgs::msg::SystemAlert>(system_alert_topic_, 10);

  // The server side never times out from lifecycle manager
  declare_parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true);
  set_parameter(rclcpp::Parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true));

  RCLCPP_INFO(get_logger(), "Lifecycle node launched, waiting on state transition requests");
}

CarmaNode::~CarmaNode()
{
  RCLCPP_INFO(get_logger(), "Destroying");

  // In case this lifecycle node wasn't properly shut down, do it here
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(get_current_state());
    on_cleanup(get_current_state());
  }
}

std::shared_ptr<carma_utils::CarmaNode>
CarmaNode::shared_from_this()
{
  return std::static_pointer_cast<carma_utils::CarmaNode>(
    rclcpp_lifecycle::LifecycleNode::shared_from_this());
}

void
CarmaNode::publish_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  system_alert_pub_->publish(*msg);
}

void
CarmaNode::on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received SystemAlert message of type: %u", msg->type);
}

void
CarmaNode::create_bond()
{
  RCLCPP_DEBUG(get_logger(), "Creating bond to lifecycle manager");

  bond_ = std::make_unique<bond::Bond>(std::string("bond"), get_name(), shared_from_this());
  bond_->setHeartbeatPeriod(0.10);
  bond_->setHeartbeatTimeout(2.0);
  bond_->start();
}

void
CarmaNode::destroy_bond()
{
  RCLCPP_DEBUG(get_logger(), "Destroying bond to lifecycle manager.");

  if (bond_) {
    bond_.reset();
  }
}

void
CarmaNode::spin()
{
  try {
    rclcpp::spin(get_node_base_interface());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Handle exception %s, and issue system alert as you wish", e.what());
  }
}

}  // namespace carma_utils

// Register the component with class_loader. This acts as a sort of entry point, allowing
// the component to be discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(carma_utils::CarmaNode)
