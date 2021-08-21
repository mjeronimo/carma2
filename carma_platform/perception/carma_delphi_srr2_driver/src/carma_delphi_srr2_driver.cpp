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

#include "carma_delphi_srr2_driver/carma_delphi_srr2_driver.hpp"

namespace carma_delphi_srr2_driver
{

CarmaDelphiSrr2Driver::CarmaDelphiSrr2Driver(const rclcpp::NodeOptions & options)
: CarmaNode(options)
{
}

carma_utils::CallbackReturn
CarmaDelphiSrr2Driver::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  CarmaNode::on_configure(state);
  system_alert_sub_ = create_subscription<cav_msgs::msg::SystemAlert>(
    system_alert_topic_, 1,
    std::bind(&CarmaDelphiSrr2Driver::on_system_alert, this, std::placeholders::_1));
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CarmaDelphiSrr2Driver::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");
  CarmaNode::on_activate(state);
  system_alert_pub_->on_activate();
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CarmaDelphiSrr2Driver::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  CarmaNode::on_deactivate(state);
  system_alert_pub_->on_deactivate();
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CarmaDelphiSrr2Driver::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  CarmaNode::on_cleanup(state);
  system_alert_pub_.reset();
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CarmaDelphiSrr2Driver::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
CarmaDelphiSrr2Driver::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node error");
  return carma_utils::CallbackReturn::SUCCESS;
}

void
CarmaDelphiSrr2Driver::on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(), "Received SystemAlert message of type: %u, msg: %s",
    msg->type, msg->description.c_str());
  RCLCPP_INFO(get_logger(), "Perform DelphiSrr2Driver-specific system event handling");
}

}  // namespace carma_delphi_srr2_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(carma_delphi_srr2_driver::CarmaDelphiSrr2Driver)
