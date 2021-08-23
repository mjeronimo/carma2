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

#include "ekf_localizer/ekf_localizer.hpp"
#include <memory>

using namespace std::chrono_literals;

namespace ekf_localizer
{

EkfLocalizer::EkfLocalizer(const rclcpp::NodeOptions & options)
: CarmaNode(options)
{
}

carma_utils::CallbackReturn
EkfLocalizer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  CarmaNode::on_configure(state);
  system_alert_sub_ = create_subscription<cav_msgs::msg::SystemAlert>(
    system_alert_topic_, 1,
    std::bind(&EkfLocalizer::on_system_alert, this, std::placeholders::_1));

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  timer_ = create_wall_timer(1s, std::bind(&EkfLocalizer::lookup_transform, this));


  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
EkfLocalizer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");
  CarmaNode::on_activate(state);
  system_alert_pub_->on_activate();
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
EkfLocalizer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  CarmaNode::on_deactivate(state);
  system_alert_pub_->on_deactivate();
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
EkfLocalizer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  CarmaNode::on_cleanup(state);
  
  // Reset the listener before the buffer
  tf_listener_.reset();
  tf_.reset();

  system_alert_pub_.reset();
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
EkfLocalizer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
EkfLocalizer::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node error");
  return carma_utils::CallbackReturn::SUCCESS;
}

void
EkfLocalizer::on_system_alert(const cav_msgs::msg::SystemAlert::SharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(), "Received SystemAlert message of type: %u, msg: %s",
    msg->type, msg->description.c_str());
  RCLCPP_INFO(get_logger(), "Perform EkfLocalizer-specific system event handling");
}


void
EkfLocalizer::lookup_transform()
{
  if (tf_->canTransform("odom", "laser", rclcpp::Time(0))) {
    geometry_msgs::msg::TransformStamped odomLaserTransform;
    try {
      odomLaserTransform = tf_->lookupTransform(
        "odom", "laser", tf2::TimePointZero, tf2::durationFromSec(
          0.0));
      RCLCPP_INFO(get_logger(), "Transform Received");
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
  } else {
    RCLCPP_INFO(get_logger(), "can't transform");
  }
}

}  // namespace ekf_localizer
