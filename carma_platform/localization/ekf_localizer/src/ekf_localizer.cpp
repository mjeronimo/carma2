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

#include "tf2_ros/create_timer_ros.h"

using namespace std::chrono_literals;

namespace ekf_localizer
{

EkfLocalizer::EkfLocalizer(const rclcpp::NodeOptions & options)
: CarmaLifecycleNode(options)
{
}

carma_utils::CallbackReturn
EkfLocalizer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  CarmaLifecycleNode::on_configure(state);

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  timer_ = create_wall_timer(1s, std::bind(&EkfLocalizer::lookup_transform, this));

  // Cancel the timer immediately to prevent it running the first time
  timer_->cancel();

  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
EkfLocalizer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");
  CarmaLifecycleNode::on_activate(state);
  timer_->reset();
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
EkfLocalizer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  CarmaLifecycleNode::on_deactivate(state);
  timer_->cancel();
  return carma_utils::CallbackReturn::SUCCESS;
}

carma_utils::CallbackReturn
EkfLocalizer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  CarmaLifecycleNode::on_cleanup(state);
  timer_->cancel();

  // Reset the listener before the buffer
  tf_listener_.reset();
  tf_.reset();

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
  RCLCPP_INFO(get_logger(), "Perform EkfLocalizer-specific system event handling");

  CarmaLifecycleNode::on_system_alert(msg);
}

void
EkfLocalizer::lookup_transform()
{
  if (tf_->canTransform("odom", "camera", rclcpp::Time(0))) {
    geometry_msgs::msg::TransformStamped odomLaserTransform;
    try {
      odomLaserTransform = tf_->lookupTransform(
        "odom", "camera", tf2::TimePointZero, tf2::durationFromSec(
          0.0));
      RCLCPP_INFO(get_logger(), "Transform received");
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
  } else {
    RCLCPP_INFO(get_logger(), "Can't generate transform");
  }
}

}  // namespace ekf_localizer
