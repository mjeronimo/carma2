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
  switch (msg->type) {
    case cav_msgs::msg::SystemAlert::SHUTDOWN:
    {
      // First, shut down all of the lifecycle nodes
      lifecycle_mgr_->shutdown();

      // Then tell all of the CARMA nodes to terminate (exit)
      auto event_publisher = create_publisher<cav_msgs::msg::SystemAlert>("/system_alert", 10);
      RCLCPP_INFO(get_logger(), "SystemController publishing TERMINATE");

      auto terminate_msg = cav_msgs::msg::SystemAlert();
      terminate_msg.type = cav_msgs::msg::SystemAlert::TERMINATE;
      terminate_msg.description = "Terminate all nodes in the system";
      event_publisher->publish(terminate_msg);
      break;
    }

    default:
      break;
  }

  CarmaNode::on_system_alert(msg);
}

}  // namespace system_controller
