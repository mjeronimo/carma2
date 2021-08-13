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

#include "carma_utils/carma_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

namespace carma_utils
{

// taken from ROS1 CARMANodeHandle.h
std::string CarmaNode::system_alert_topic_ = "/system_alert";

CarmaNode::CarmaNode(
  const std::string & node_name,
  const std::string & ns, bool use_rclcpp_node,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, ns, options),
  use_rclcpp_node_(use_rclcpp_node)
{


   // create system alert publisher subscriber will be made by child class
   system_alert_pub_ = this->create_publisher<cav_msgs::msg::SystemAlert> (system_alert_topic_, 0);

  // The server side never times out from lifecycle manager
  this->declare_parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true);
  this->set_parameter(
    rclcpp::Parameter(
      bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true));

  if (use_rclcpp_node_) {
    std::vector<std::string> new_args = options.arguments();
    new_args.push_back("--ros-args");
    new_args.push_back("-r");
    new_args.push_back(std::string("__node:=") + this->get_name() + "_rclcpp_node");
    new_args.push_back("--");
    rclcpp_node_ = std::make_shared<rclcpp::Node>(
      "_", ns, rclcpp::NodeOptions(options).arguments(new_args));
    rclcpp_thread_ = std::make_unique<ros2_utils::NodeThread>(rclcpp_node_);
  }

 

  print_lifecycle_node_notification();


}

CarmaNode::~CarmaNode()
{
  RCLCPP_INFO(get_logger(), "Destroying");
  // In case this lifecycle node wasn't properly shut down, do it here
  if (get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    on_deactivate(get_current_state());
    on_cleanup(get_current_state());
  }
}


// Carma Alert Publisher
void CarmaNode::publishSystemAlert(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  system_alert_pub_->publish(*msg);
}

// Carma Alert Handler
void CarmaNode::systemAlertHandler(const cav_msgs::msg::SystemAlert::SharedPtr msg) 
{
    RCLCPP_INFO(this->get_logger(),"Received SystemAlert message of type: %u",msg->type);
}

void CarmaNode::createBond()
{
  RCLCPP_INFO(get_logger(), "Creating bond to lifecycle manager");

  bond_ = std::make_unique<bond::Bond>(
    std::string("bond"),
    this->get_name(),
    shared_from_this());

  bond_->setHeartbeatPeriod(0.10);
  bond_->setHeartbeatTimeout(4.0);
  bond_->start();
}

void CarmaNode::destroyBond()
{
  RCLCPP_INFO(get_logger(), "Destroying bond (%s) to lifecycle manager.", this->get_name());

  if (bond_) {
    bond_.reset();
  }
}

void CarmaNode::spin()
{
  try
  {
    rclcpp::spin(this->get_node_base_interface());
  }
  catch(const std::exception& e)
  {
     RCLCPP_ERROR(this->get_logger(),"handle exception %s",e.what());
  }

}

void CarmaNode::print_lifecycle_node_notification()
{
  RCLCPP_INFO(get_logger(), "Lifecycle node launched, waiting on external lifecycle transitions to activate");
}

}  // namespace carma_utils
