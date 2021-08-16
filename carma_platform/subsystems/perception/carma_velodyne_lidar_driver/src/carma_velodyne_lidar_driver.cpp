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

#include "carma_velodyne_lidar_driver/carma_velodyne_lidar_driver.hpp"

namespace carma_velodyne_lidar_driver
{

CarmaVelodyneLidarDriver::CarmaVelodyneLidarDriver()
: CarmaNode("carma_velodyne_lidar_driver")
{
   system_alert_sub_ = create_subscription<cav_msgs::msg::SystemAlert>(system_alert_topic_, 1, 
        std::bind(&CarmaVelodyneLidarDriver::handle_system_alert, this, std::placeholders::_1));
}

CarmaVelodyneLidarDriver::~CarmaVelodyneLidarDriver()
{
}

carma_utils::CallbackReturn
CarmaVelodyneLidarDriver::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CarmaVelodyneLidarDriver::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
 
  // Create bond with the lifecycle manager
  create_bond();

  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CarmaVelodyneLidarDriver::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Destroy the bond with the lifecycle manager
  destroy_bond();

  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn 
CarmaVelodyneLidarDriver::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn 
CarmaVelodyneLidarDriver::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CarmaVelodyneLidarDriver::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node error");
  return carma_utils::CallbackReturn::SUCCESS;
}

void
CarmaVelodyneLidarDriver::handle_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(),"Received SystemAlert message of type: %u, msg: %s",
              msg->type,msg->description.c_str());
  RCLCPP_INFO(get_logger(),"Perform velodyne-specific system event handling");
}

}  // namespace carma_velodyne_lidar_driver
