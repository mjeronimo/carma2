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

#ifndef ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "bondcpp/bond.hpp"
#include "ros2_utils/lifecycle_service_client.hpp"
#include "ros2_utils/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_lifecycle_manager_msgs/srv/manage_lifecycle_nodes.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "cav_msgs/msg/system_alert.hpp"

namespace ros2_lifecycle_manager
{

using ros2_lifecycle_manager_msgs::srv::ManageLifecycleNodes;

// The LifecycleManager implements the service interface to transition managed nodes.
// It receives a transition request and then uses the managed node's lifecycle 
// interface to change its state.
class LifecycleManager : public rclcpp::Node
{
public:
  LifecycleManager();
  ~LifecycleManager();

protected:
  bool startup();
  bool shutdown();
  bool reset();
  bool pause();
  bool resume();

  cav_msgs::msg::SystemAlert  alert_msg;

  void createLifecycleServiceClients();
  void destroyLifecycleServiceClients();

  void createBondTimer();
  void destroyBondTimer();
  bool createBondConnection(const std::string & node_name);
  void checkBondConnections();

  bool changeStateForNode(const std::string & node_name, std::uint8_t transition);
  bool changeStateForAllNodes(std::uint8_t transition);
  void shutdownAllNodes();

  /**
   * @brief publish the system alert message 
   */
  void publishSystemAlert(const cav_msgs::msg::SystemAlert::SharedPtr msg);

  /**
   * @brief handle the system alert message 
   */
  void systemAlertHandler(const cav_msgs::msg::SystemAlert::SharedPtr msg);

  // Callback group used by services and timers
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unique_ptr<ros2_utils::NodeThread> service_thread_;

  // The services provided by this node
  rclcpp::Service<ManageLifecycleNodes>::SharedPtr manager_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr is_active_srv_;

  void managerCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ManageLifecycleNodes::Request> request,
    std::shared_ptr<ManageLifecycleNodes::Response> response);

  void isActiveCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Timer thread to look at bond connections
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr bond_timer_;
  std::chrono::milliseconds bond_timeout_;

  // A map of all nodes to check bond connection
  std::map<std::string, std::shared_ptr<bond::Bond>> bond_map_;

  // A map of all nodes to be controlled
  std::map<std::string, std::shared_ptr<ros2_utils::LifecycleServiceClient>> node_map_;

  std::map<std::uint8_t, std::string> transition_label_map_;

  // A map of the expected transitions to primary states
  std::unordered_map<std::uint8_t, std::uint8_t> transition_state_map_;

  // Subscribe to other system alerts
  rclcpp::Subscription<cav_msgs::msg::SystemAlert>::SharedPtr  system_alert_sub_;

  // System Alert Topic
  static std::string system_alert_topic_;

  // System Alerts Publisher
  rclcpp::Publisher<cav_msgs::msg::SystemAlert>::SharedPtr  system_alert_pub_;

  // The names of the nodes to be managed, in the order of desired bring-up
  std::vector<std::string> node_names_;

  // Whether to automatically start up the system
  bool autostart_{false};

  bool system_active_{false};
};

}  // namespace ros2_lifecycle_manager

#endif  // ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
