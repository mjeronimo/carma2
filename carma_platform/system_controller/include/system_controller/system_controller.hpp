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

#ifndef SYSTEM_CONTROLLER__SYSTEM_CONTROLLER_HPP_
#define SYSTEM_CONTROLLER__SYSTEM_CONTROLLER_HPP_

#include <memory>

#include "carma_utils/carma_node.hpp"
#include "cav_msgs/msg/system_alert.hpp"
#include "ros2_lifecycle_manager/lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"

namespace system_controller
{

class SystemController : public carma_utils::CarmaNode
{
public:
  SystemController() = delete;
  SystemController(const rclcpp::NodeOptions & options);

protected:
  void on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg) override;

  std::shared_ptr<ros2_lifecycle_manager::LifecycleManager> lifecycle_mgr_;
};

}  // namespace system_controller

#endif  // SYSTEM_CONTROLLER__SYSTEM_CONTROLLER_HPP_
