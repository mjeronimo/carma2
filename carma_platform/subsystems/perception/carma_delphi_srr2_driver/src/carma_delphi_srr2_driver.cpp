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

#include "carma_delphi_srr2_driver/carma_delphi_srr2_driver.hpp"

namespace carma_delphi_srr2_driver
{

CarmaDelphiSrr2Driver::CarmaDelphiSrr2Driver()
: CarmaNode("carma_delphi_srr2_driver")
{
    system_alert_sub_ = this->create_subscription<cav_msgs::msg::SystemAlert>(system_alert_topic_, 1, 
        std::bind(&CarmaDelphiSrr2Driver::systemAlertHandler, this, std::placeholders::_1));
}

CarmaDelphiSrr2Driver::~CarmaDelphiSrr2Driver()
{
}

carma_utils::CallbackReturn
CarmaDelphiSrr2Driver::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CarmaDelphiSrr2Driver::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Create bond with the lifecycle manager
  createBond();

  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CarmaDelphiSrr2Driver::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Destroy the bond with the lifecycle manager
  destroyBond();

  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn 
CarmaDelphiSrr2Driver::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn 
CarmaDelphiSrr2Driver::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return carma_utils::CallbackReturn::SUCCESS;
}

void CarmaDelphiSrr2Driver::systemAlertHandler(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),"Received SystemAlert message of type: %u, msg: %s",
              msg->type,msg->description.c_str());
  RCLCPP_INFO(this->get_logger(),"Perform Delphi Specific System Event Handling");
}

}  // namespace carma_delphi_srr2_driver
