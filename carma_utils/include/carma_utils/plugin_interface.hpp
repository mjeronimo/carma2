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

#ifndef CARMA_UTILS__PLUGIN_INTERFACE_HPP_
#define CARMA_UTILS__PLUGIN_INTERFACE_HPP_

#include "ros2_utils/lifecycle_interface.hpp"
#include "carma_utils/carma_node.hpp"

namespace carma_utils
{

class PluginInterface : ros2_utils::LifecycleInterface
{
public:
  PluginInterface(){};
  virtual ~PluginInterface(){}; //= default;
  virtual void initialize(const carma_utils::CarmaNode::SharedPtr node){node_=node;};
  virtual void configure(){};
  virtual void activate(){};
  virtual void deactivate(){};
  virtual void cleanup(){}; 
protected:
  // node interface
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

}  // namespace carma_utils

#endif  // CARMA_UTILS__PLUGIN_INTERFACE_HPP_