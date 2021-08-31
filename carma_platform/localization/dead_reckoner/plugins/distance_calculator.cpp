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

#include "dead_reckoner/distance_calculator.hpp"

#include <exception>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace dead_reckoner
{

void
DistanceCalculator::initialize(const carma_utils::CarmaLifecycleNode::SharedPtr node)
{
  node_ = node;
}

void
DistanceCalculator::configure()
{
  RCLCPP_INFO(node_->get_logger(), "Plugin on configure");
  distance_pub_ = node_->create_publisher<std_msgs::msg::Float32>("distance_to_fix", 10);
}

void
DistanceCalculator::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Plugin on activate");
  distance_pub_->on_activate();
}

void
DistanceCalculator::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Plugin on deactivate");
  distance_pub_->on_deactivate();
}

void
DistanceCalculator::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Plugin on cleanup");
  distance_pub_.reset();
}

void
DistanceCalculator::publish_distance()
{
  auto message = std_msgs::msg::Float32();
  message.data = 15.0;
  distance_pub_->publish(message);
}

}  // namespace dead_reckoner

PLUGINLIB_EXPORT_CLASS(dead_reckoner::DistanceCalculator, carma_utils::PluginInterface)
