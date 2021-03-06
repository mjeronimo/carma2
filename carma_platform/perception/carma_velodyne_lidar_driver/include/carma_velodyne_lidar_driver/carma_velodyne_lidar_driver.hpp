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

#ifndef CARMA_VELODYNE_LIDAR_DRIVER__CARMA_VELODYNE_LIDAR_DRIVER_HPP_
#define CARMA_VELODYNE_LIDAR_DRIVER__CARMA_VELODYNE_LIDAR_DRIVER_HPP_

#include "carma_utils/carma_lifecycle_node.hpp"
#include "carma_utils/visibility_control.hpp"
#include "rclcpp/rclcpp.hpp"

namespace carma_velodyne_lidar_driver
{

class CarmaVelodyneLidarDriver : public carma_utils::CarmaLifecycleNode
{
public:
  CarmaVelodyneLidarDriver();

  CARMA_UTILS_PUBLIC
  explicit CarmaVelodyneLidarDriver(const rclcpp::NodeOptions & options);

protected:
  carma_utils::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  void on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg) override;
};

}  // namespace carma_velodyne_lidar_driver

#endif  //  CARMA_VELODYNE_LIDAR_DRIVER__CARMA_VELODYNE_LIDAR_DRIVER_HPP_
