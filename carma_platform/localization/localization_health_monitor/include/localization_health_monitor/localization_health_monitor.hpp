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

#ifndef LOCALIZATION_HEALTH_MONITOR__LOCALIZATION_HEALTH_MONITOR_HPP_
#define LOCALIZATION_HEALTH_MONITOR__LOCALIZATION_HEALTH_MONITOR_HPP_

#include "carma_utils/carma_node.hpp"
#include "cav_msgs/msg/localization_status_report.hpp"
#include "rclcpp/rclcpp.hpp"

namespace localization_health_monitor
{

class LocalizationHealthMonitor : public carma_utils::CarmaNode
{
public:
  LocalizationHealthMonitor();
  ~LocalizationHealthMonitor();

protected:
  carma_utils::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  carma_utils::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;
  cav_msgs::msg::SystemAlert alert_msg;
  rclcpp::Subscription<cav_msgs::msg::LocalizationStatusReport>::SharedPtr localization_status_sub_;
  void handle_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg);
  void handle_localization_status(const cav_msgs::msg::LocalizationStatusReport::SharedPtr msg);
};

}  // namespace localization_health_monitor

#endif  //  LOCALIZATION_HEALTH_MONITOR__LOCALIZATION_HEALTH_MONITOR_HPP_
