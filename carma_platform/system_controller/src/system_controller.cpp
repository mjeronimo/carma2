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

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_lifecycle_manager/lifecycle_manager.hpp"
#include "system_controller/system_controller.hpp"

namespace system_controller
{

std::string SystemController::system_alert_topic_ = "/system_alert";

SystemController::SystemController()
{
}
  
SystemController::~SystemController()
{
}

void 
SystemController::subscribe_to_system_alerts()
{
   // Create system alert subscriber and publisher for lifcycle manager
  system_alert_sub_ = create_subscription<cav_msgs::msg::SystemAlert>(system_alert_topic_, 10, 
        std::bind(&SystemController::handle_system_alert, this, std::placeholders::_1));
  system_alert_pub_ = create_publisher<cav_msgs::msg::SystemAlert> (system_alert_topic_, 10);
}

// Carma alert publisher
void 
SystemController::publish_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  system_alert_pub_->publish(*msg);
}

// Carma alert handler
void 
SystemController::handle_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg) 
{
  RCLCPP_INFO(get_logger(),"Received SystemAlert message of type: %u, msg: %s",
              msg->type,msg->description.c_str());

  if (msg->type ==  cav_msgs::msg::SystemAlert::CAUTION) {
    pause();
  } else if ((msg->type ==  cav_msgs::msg::SystemAlert::SHUTDOWN) | (msg->type ==  cav_msgs::msg::SystemAlert::FATAL)) {
    shutdown();
  }
}

// bool
// SystemController::startup()
// {
//   RCLCPP_INFO(get_logger(), "Starting managed nodes bringup");

//   if (!changeStateForAllNodes(Transition::TRANSITION_CONFIGURE) ||
//     !changeStateForAllNodes(Transition::TRANSITION_ACTIVATE))
//   {
//     RCLCPP_ERROR(get_logger(), "Failed to bring up all requested nodes. Aborting bringup.");
//     return false;
//   }
//   RCLCPP_INFO(get_logger(), "Managed nodes are active");

//   if (driver_manager_) {
//     // Example alert message
//     cav_msgs::msg::SystemAlert alert_msg;
//     alert_msg.type = cav_msgs::msg::SystemAlert::DRIVERS_READY;
//     alert_msg.description = "Drivers are ready";
//     system_alert_pub_->publish(alert_msg);
//   }

//   createBondTimer();
//   return true;
// }


}  // namespace system_controller
