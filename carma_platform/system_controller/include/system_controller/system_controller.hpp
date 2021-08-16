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

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "cav_msgs/msg/system_alert.hpp"
#include "ros2_lifecycle_manager/lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"



namespace system_controller
{


// The System Controller inherits from the lifecycle manager class from ROS2
// It adds system alerts for the CARMA System on top of the existing lifecycle functionality. 

class SystemController : public ros2_lifecycle_manager::LifecycleManager       
{
public:
  SystemController();
  ~SystemController();
  void subscribe_to_system_alerts();

protected:
  // System alerts
  static std::string system_alert_topic_;
  rclcpp::Subscription<cav_msgs::msg::SystemAlert>::SharedPtr system_alert_sub_;
  rclcpp::Publisher<cav_msgs::msg::SystemAlert>::SharedPtr system_alert_pub_;

  // whether or not this is a driver manager (ideally this would be more sophisticated)
  bool driver_manager_{false};
  void publish_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg);
  void handle_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg);
};

}  // namespace system_controller

#endif  // SYSTEM_CONTROLLER__SYSTEM_CONTROLLER_HPP_
