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

#ifndef DEAD_RECKONER__DISTANCE_CALCULATOR_HPP_
#define DEAD_RECKONER__DISTANCE_CALCULATOR_HPP_

#include <memory>

#include "carma_utils/carma_lifecycle_node.hpp"
#include "carma_utils/plugin_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace dead_reckoner
{

class DistanceCalculator : public carma_utils::PluginInterface
{
public:
  DistanceCalculator() = default;
  void initialize(const carma_utils::CarmaLifecycleNode::SharedPtr node);

  void configure() override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;

protected:
  void publish_distance();
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>> distance_pub_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

}  // namespace dead_reckoner

#endif  //  DEAD_RECKONER__DISTANCE_CALCULATOR_HPP_
