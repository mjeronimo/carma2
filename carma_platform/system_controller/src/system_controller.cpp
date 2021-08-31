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

#include "system_controller/system_controller.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace system_controller
{

SystemController::SystemController(const rclcpp::NodeOptions & options)
: CarmaNode(options) 
{
  lifecycle_mgr_ = std::make_shared<ros2_lifecycle_manager::LifecycleManager>(
      get_node_base_interface(),
      get_node_parameters_interface(),
      get_node_logging_interface(),
      get_node_timers_interface(),
      get_node_services_interface()
  );
}

void
SystemController::on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(), "Received SystemAlert message of type: %u, msg: %s",
    msg->type, msg->description.c_str());

  switch (msg->type) {
    case cav_msgs::msg::SystemAlert::CAUTION:
      lifecycle_mgr_->pause();
      break;
    case cav_msgs::msg::SystemAlert::SHUTDOWN:
    case cav_msgs::msg::SystemAlert::FATAL:
      lifecycle_mgr_->shutdown();
      break;
  }
}

}  // namespace system_controller
