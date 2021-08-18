//
// Copyright (C) 2021 LEIDOS.
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may not
// use this file except in compliance with the License. You may obtain a copy of
// the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations under
// the License.
//

#include "ekf_localizer/ekf_localizer.hpp"

namespace ekf_localizer
{

EkfLocalizer::EkfLocalizer()
: CarmaNode("ekf_localizer")
{
}

EkfLocalizer::~EkfLocalizer()
{
}

carma_utils::CallbackReturn
EkfLocalizer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  system_alert_sub_ = create_subscription<cav_msgs::msg::SystemAlert>(system_alert_topic_, 1,
        std::bind(&EkfLocalizer::systemAlertHandler, this, std::placeholders::_1));
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
EkfLocalizer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  system_alert_pub_->on_activate();
  
  // Create bond with the lifecycle manager
  create_bond();
  

  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
EkfLocalizer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  system_alert_pub_->on_deactivate();
  // Destroy the bond with the lifecycle manager
  destroy_bond();

  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
EkfLocalizer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  system_alert_pub_.reset();
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
EkfLocalizer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  system_alert_pub_.reset();
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
EkfLocalizer::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node error");
  return carma_utils::CallbackReturn::SUCCESS;
}


void
EkfLocalizer::systemAlertHandler(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received SystemAlert message of type: %u, msg: %s",
              msg->type, msg->description.c_str());
  RCLCPP_INFO(get_logger(), "Perform EkfLocalizer-specific system event handling");
}

}  // namespace ekf_localizer
