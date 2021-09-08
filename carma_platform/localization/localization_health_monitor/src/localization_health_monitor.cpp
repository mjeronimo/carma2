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

#include "localization_health_monitor/localization_health_monitor.hpp"

namespace localization_health_monitor
{

LocalizationHealthMonitor::LocalizationHealthMonitor(const rclcpp::NodeOptions & options)
: CarmaLifecycleNode(options)
{
  declare_parameter("auto_initialization_timeout", rclcpp::ParameterValue(30000));
  declare_parameter("fitness_score_degraded_threshold", rclcpp::ParameterValue(20.0));
  declare_parameter("fitness_score_fault_threshold", rclcpp::ParameterValue(100000.0));
  declare_parameter("gnss_only_operation_timeout", rclcpp::ParameterValue(20000));
  declare_parameter("ndt_frequency_degraded_threshold", rclcpp::ParameterValue(8.0));
  declare_parameter("ndt_frequency_fault_threshold", rclcpp::ParameterValue(0.01));
}

carma_utils::CallbackReturn
LocalizationHealthMonitor::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  CarmaLifecycleNode::on_configure(state);

  localization_status_sub_ = create_subscription<cav_msgs::msg::LocalizationStatusReport>(
    "/localization_status", 1,
    std::bind(
      &LocalizationHealthMonitor::on_localization_status_change, this,
      std::placeholders::_1));

  get_parameter("auto_initialization_timeout", auto_initialization_timeout_);
  get_parameter("fitness_score_degraded_threshold", fitness_score_degraded_threshold_);
  get_parameter("fitness_score_fault_threshold", fitness_score_fault_threshold_);
  get_parameter("gnss_only_operation_timeout", gnss_only_operation_timeout_);
  get_parameter("ndt_frequency_degraded_threshold", ndt_frequency_degraded_threshold_);
  get_parameter("ndt_frequency_fault_threshold", ndt_frequency_fault_threshold_);

  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
LocalizationHealthMonitor::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");
  CarmaLifecycleNode::on_activate(state);
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
LocalizationHealthMonitor::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  CarmaLifecycleNode::on_deactivate(state);
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
LocalizationHealthMonitor::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  CarmaLifecycleNode::on_cleanup(state);
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
LocalizationHealthMonitor::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
LocalizationHealthMonitor::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node error");
  return carma_utils::CallbackReturn::SUCCESS;
}

void
LocalizationHealthMonitor::on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  // This where to implement the logic for the LocalizationHealthMonitor
  RCLCPP_INFO(get_logger(), "Perform Localization-Health-Monitor-specific system event handling");

  CarmaLifecycleNode::on_system_alert(msg);
}

void
LocalizationHealthMonitor::on_localization_status_change(
  const cav_msgs::msg::LocalizationStatusReport::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "pub is activated %d", system_alert_pub_->is_activated());

  cav_msgs::msg::SystemAlert alert_msg;
  switch (msg->status) {
    case cav_msgs::msg::LocalizationStatusReport::INITIALIZING:
      RCLCPP_INFO(get_logger(), "Localization System Initializing");
      break;

    case cav_msgs::msg::LocalizationStatusReport::DEGRADED_NO_LIDAR_FIX:
      alert_msg.type = cav_msgs::msg::SystemAlert::FATAL;
      alert_msg.description = "Localization in Degraded Mode No Fix";
      this->system_alert_pub_->publish(alert_msg);
      break;

    case cav_msgs::msg::LocalizationStatusReport::DEGRADED:
    case cav_msgs::msg::LocalizationStatusReport::AWAIT_MANUAL_INITIALIZATION:
      alert_msg.type = cav_msgs::msg::SystemAlert::CAUTION;
      alert_msg.description = "Localization in Degraded Mode";
      this->system_alert_pub_->publish(alert_msg);
      break;
  }
}

}  // namespace localization_health_monitor
