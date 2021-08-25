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

#include "ros2_lifecycle_manager/lifecycle_manager.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using lifecycle_msgs::msg::Transition;
using lifecycle_msgs::msg::State;

namespace ros2_lifecycle_manager
{

LifecycleManager::LifecycleManager()
: Node("lifecycle_manager")
{
  // The list of names is parameterized, allowing this module to be used with a different set
  // of managed nodes. By default the node name list is empty.
  std::vector<std::string> default_node_names;
  declare_parameter("node_names", rclcpp::ParameterValue(default_node_names));
  declare_parameter("autostart", rclcpp::ParameterValue(false));
  declare_parameter("bond_timeout", 2.0);

  node_names_ = get_parameter("node_names").as_string_array();
  get_parameter("autostart", autostart_);
  double bond_timeout_s;
  get_parameter("bond_timeout", bond_timeout_s);
  bond_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(bond_timeout_s));

  manager_srv_ = create_service<ManageLifecycleNodes>(
    get_name() + std::string("/manage_nodes"),
    std::bind(&LifecycleManager::managerCallback, this, _1, _2, _3));

  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "-r", std::string("__node:=") + get_name() + "_service_client", "--"});
  service_client_node_ = std::make_shared<rclcpp::Node>("_", options);

  transition_state_map_[Transition::TRANSITION_CONFIGURE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_CLEANUP] = State::PRIMARY_STATE_UNCONFIGURED;
  transition_state_map_[Transition::TRANSITION_ACTIVATE] = State::PRIMARY_STATE_ACTIVE;
  transition_state_map_[Transition::TRANSITION_DEACTIVATE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
    State::PRIMARY_STATE_FINALIZED;

  transition_label_map_[Transition::TRANSITION_CONFIGURE] = std::string("Configuring ");
  transition_label_map_[Transition::TRANSITION_CLEANUP] = std::string("Cleaning up ");
  transition_label_map_[Transition::TRANSITION_ACTIVATE] = std::string("Activating ");
  transition_label_map_[Transition::TRANSITION_DEACTIVATE] = std::string("Deactivating ");
  transition_label_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
    std::string("Shutting down ");

  // TODO(mjeronimo): Get rid of this timer callback
  // and do it a different way (avoid race condition)
  // Can't use shared_from_this() during construction
  init_timer_ = create_wall_timer(
    // std::chrono::nanoseconds(10),
    std::chrono::seconds(1),
    [this]() -> void {
      init_timer_->cancel();
      createLifecycleServiceClients();
      if (autostart_) {
        startup();
      }
    });
}

LifecycleManager::~LifecycleManager()
{
}

void
LifecycleManager::managerCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ManageLifecycleNodes::Request> request,
  std::shared_ptr<ManageLifecycleNodes::Response> response)
{
  switch (request->command) {
    case ManageLifecycleNodes::Request::STARTUP:
      response->success = startup();
      break;
    case ManageLifecycleNodes::Request::RESET:
      response->success = reset();
      break;
    case ManageLifecycleNodes::Request::SHUTDOWN:
      response->success = shutdown();
      break;
    case ManageLifecycleNodes::Request::PAUSE:
      response->success = pause();
      break;
    case ManageLifecycleNodes::Request::RESUME:
      response->success = resume();
      break;
  }
}

void
LifecycleManager::createLifecycleServiceClients()
{
  RCLCPP_INFO(get_logger(), "Creating and initializing lifecycle service clients");
  for (auto & node_name : node_names_) {
    node_map_[node_name] =
      std::make_shared<ros2_utils::LifecycleServiceClient>(node_name, service_client_node_);
  }
}

void
LifecycleManager::destroyLifecycleServiceClients()
{
  RCLCPP_INFO(get_logger(), "Destroying lifecycle service clients");
  for (auto & kv : node_map_) {
    kv.second.reset();
  }
}

bool
LifecycleManager::createBondConnection(const std::string & node_name)
{
  const double timeout_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(bond_timeout_).count();
  const double timeout_s = timeout_ns / 1e9;

  if (bond_map_.find(node_name) == bond_map_.end() && bond_timeout_.count() > 0.0) {
    bond_map_[node_name] =
      std::make_shared<bond::Bond>("bond", node_name, shared_from_this());
    bond_map_[node_name]->setHeartbeatTimeout(timeout_s);
    bond_map_[node_name]->setHeartbeatPeriod(0.10);
    bond_map_[node_name]->start();
    if (
      !bond_map_[node_name]->waitUntilFormed(
        // rclcpp::Duration(rclcpp::Duration::from_nanoseconds(timeout_ns / 2))))
        // rclcpp::Duration(rclcpp::Duration::from_nanoseconds(timeout_ns))))
        rclcpp::Duration(timeout_ns)))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Server %s was unable to be reached after %0.2fs by bond. "
        "This server may be misconfigured.",
        node_name.c_str(), timeout_s);
      return false;
    }
    RCLCPP_DEBUG(get_logger(), "Server %s connected with bond", node_name.c_str());
  }

  return true;
}

bool
LifecycleManager::changeStateForNode(const std::string & node_name, std::uint8_t transition)
{
  std::string msg = transition_label_map_[transition] + node_name;
  RCLCPP_INFO(get_logger(), msg.c_str());

  try {
    if (!node_map_[node_name]->change_state(transition, 1s) ||
      !(node_map_[node_name]->get_state() == transition_state_map_[transition]))
    {
      RCLCPP_ERROR(get_logger(), "Failed to change lifecycle node state");
      return false;
    }
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(get_logger(), "Failed to change lifecycle node state");
    return false;
  }

#ifdef USE_BOND_CONNECTIONS
  if (transition == Transition::TRANSITION_ACTIVATE) {
    return createBondConnection(node_name);
  } else if (transition == Transition::TRANSITION_DEACTIVATE) {
    bond_map_.erase(node_name);
  }
#endif

  return true;
}

bool
LifecycleManager::changeStateForAllNodes(std::uint8_t transition)
{
  if (transition == Transition::TRANSITION_CONFIGURE ||
    transition == Transition::TRANSITION_ACTIVATE)
  {
    for (auto & node_name : node_names_) {
      if (!changeStateForNode(node_name, transition)) {
        return false;
      }
    }
  } else {
    std::vector<std::string>::reverse_iterator rit;
    for (rit = node_names_.rbegin(); rit != node_names_.rend(); ++rit) {
      if (!changeStateForNode(*rit, transition)) {
        return false;
      }
    }
  }
  return true;
}

void
LifecycleManager::shutdownAllNodes()
{
  RCLCPP_INFO(get_logger(), "Deactivate, cleanup, and shutdown nodes");

  changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE);
  changeStateForAllNodes(Transition::TRANSITION_CLEANUP);
  changeStateForAllNodes(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}

bool
LifecycleManager::startup()
{
  RCLCPP_INFO(get_logger(), "Starting managed nodes bringup");

  if (!changeStateForAllNodes(Transition::TRANSITION_CONFIGURE) ||
    !changeStateForAllNodes(Transition::TRANSITION_ACTIVATE))
  {
    RCLCPP_ERROR(get_logger(), "Failed to bring up all requested nodes. Aborting bringup.");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Managed nodes are active");

#ifdef USE_BOND_CONNECTIONS
  createBondTimer();
#endif
  return true;
}

bool
LifecycleManager::shutdown()
{
#ifdef USE_BOND_CONNECTIONS
  destroyBondTimer();
#endif

  RCLCPP_INFO(get_logger(), "Shutting down managed nodes...");
  shutdownAllNodes();
  destroyLifecycleServiceClients();
  RCLCPP_INFO(get_logger(), "Managed nodes have been shut down");

  return true;
}

bool
LifecycleManager::reset()
{
#ifdef USE_BOND_CONNECTIONS
  destroyBondTimer();
#endif

  // Should transition in reverse order
  RCLCPP_INFO(get_logger(), "Resetting managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE) ||
    !changeStateForAllNodes(Transition::TRANSITION_CLEANUP))
  {
    RCLCPP_ERROR(get_logger(), "Failed to reset nodes: aborting reset");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Managed nodes have been reset");
  return true;
}

bool
LifecycleManager::pause()
{
#ifdef USE_BOND_CONNECTIONS
  destroyBondTimer();
#endif

  RCLCPP_INFO(get_logger(), "Pausing managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE)) {
    RCLCPP_ERROR(get_logger(), "Failed to pause nodes: aborting pause");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Managed nodes have been paused");

  return true;
}

bool
LifecycleManager::resume()
{
  RCLCPP_INFO(get_logger(), "Resuming managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_ACTIVATE)) {
    RCLCPP_ERROR(get_logger(), "Failed to resume nodes: aborting resume");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Managed nodes are active");

#ifdef USE_BOND_CONNECTIONS
  createBondTimer();
#endif
  return true;
}

void
LifecycleManager::createBondTimer()
{
  if (bond_timeout_.count() <= 0) {
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Creating bond timer");
  bond_timer_ = create_wall_timer(200ms, std::bind(&LifecycleManager::checkBondConnections, this));
}

void
LifecycleManager::destroyBondTimer()
{
  if (bond_timer_) {
    RCLCPP_DEBUG(get_logger(), "Terminating bond timer");
    bond_timer_->cancel();
    bond_timer_.reset();
  }
}

void
LifecycleManager::checkBondConnections()
{
  if (bond_map_.empty()) {
    return;
  }

  for (auto & node_name : node_names_) {
    if (!rclcpp::ok()) {
      return;
    }

    if (bond_map_[node_name]->isBroken()) {
      RCLCPP_WARN(get_logger(), "No heartbeat from %s, restarting", node_name.c_str());

      // Remove the current bond from the map;
      // we'll have to create a new one with the
      // new node automatically restarted by the launch system
      bond_map_.erase(node_name);

      // TODO(mjeronimo): make this more robust. For now, just assume that the node
      // has crashed and has been restarted by the launch system. Therefore, we need
      // to configure and activate it
      if (changeStateForNode(node_name, Transition::TRANSITION_CONFIGURE)) {
        if (!changeStateForNode(node_name, Transition::TRANSITION_ACTIVATE)) {
          bond_map_.erase(node_name);
        }
      } else {
        bond_map_.erase(node_name);
      }
    }
  }
}

}  // namespace ros2_lifecycle_manager
