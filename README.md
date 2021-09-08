# Build and run the CARMA2 source code

## Create a colcon workspace

```
export CARMA_WS=~/src/carma2_ws
mkdir -p $CARMA_WS/src
cd $CARMA_WS/src
```

## Download the CARMA2 source code

```
git clone https://github.com/mjeronimo/carma2
vcs import < carma2/carma2.repos
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

## Build CARMA2

```
cd $CARMA_WS
colcon build --symlink-install
```

## Source the CARMA2 workspace

```
source $CARMA_WS/install/setup.bash
```

## Launch the system

```
ros2 launch carma_bringup carma_launch.py
```

# Build and run CARMA2 in a Docker container

## Build the Docker image

Foxy

```
cd $CARMA_WS/src/carma2/docker
./build-image.sh
```

Rolling
```
cd $CARMA_WS/src/carma2/docker
./build-image.sh --rosdistro=rolling
```

## Launch the Docker container

Foxy
```
docker run -it openrobotics/carma2:foxy
```

Rolling
```
docker run -it openrobotics/carma2:rolling
```

# Features/Capabilities

The example code in this repository provides the following features:

* CMake code
    * Can be included by CARMA packages
    * The carma_project() macro serves as a single source for compiler settings, etc.

* CarmaNode and CarmaLifecycleNode
    * Base classes for CARMA nodes in the system
    * CarmaNode derives from rclcpp::Node and CarmaLifecycleNode derives from rclcpp_lifecycle::LifecycleNode
    * System alert capable (adds CARMA specifics)

* Perception Subsystem
    * All nodes are lifecycle nodes
    * All nodes are also composable nodes and are loaded into a single container
        * A stub CARMA Delphi SRR2 Driver
        * A stub CARMA Velodyne LiDAR Driver
        * A sample "camera driver" sending images to a client
        * A sample camera client to receive images
    * No-copy messages communication within the subsystem

* Localization Subsystem
    * All nodes are lifecycle nodes
        * A stub dead_reckoner node
        * A stub ekf_localizer node node
        * Localization Health Monitor
            * With sample parameters
            * Handles LocalizationStatusReport messages
            * Messages sent manually using a sample command line program

* System Controller
    * A Lifecycle Manager that manages the state of the CARMA nodes (lifecycle nodes), initiating state transitions
    * Messages (such as SHUTDOWN) can be sent manually via a command-line program
    * System alert capable

* Sample usage of the rclcpp node, such as using a message filter
* Sample usage from a CARMA node of a transform listener (doesn't need the rclcpp_node)
* Create timers in on_configure and then deactivate (cancel) and reactivate (reset) them, without using our own member variables (like "active").
* Make the show_image variable a node parameter in camera_driver_client, so it can be configured by the launch script
* Parameterize the launch script so that it can optionally display the output window (default to not displaying the output
    window so that it works in a Docker container)
* Environment variable, CARMA_LAUNCH_PREFIX to set the launch prefix for nodes in the launch file
* Using a helper class that itself creates pubs/subs, but accepts a node (not itself a node)
* XML launch file (equivalent to the current Python launch file)
* A plugin to some node (a "helper plugin"; not a node itself, but takes a node to use)
* Separate carma_node and carma_lifecycle_node classes
* ros2_lifecycle_manager is a helper class (that takes a Node) rather than a Node
* Namespaces in launch scripts
* Can set CARMA_LAUNCH_PREFIX in the environment: export CARMA_LAUNCH_PREFIX="xterm -fa 'Monospace' -fs 10 -geometry 120x30 -hold -e"
* Reporting process exit via SystemEvent: launch publishes SHUTDOWN, system_controller shuts down nodes and publishes TERMINATE, nodes exit


# Architecture Questions

* What is the recovery strategy?
    * Notify the system monitor?
    * Monitor and reset a subsystem?

* How does Launch interact with Lifecycle nodes and recovery?
    * Which component owns restarting the nodes?

* If a ComposableNode crashes, does it bring down the container?

# ROS 2 Porting considerations

* Launch
* Lifecycle nodes
* Parameters
* Subsystems
* System Eventing and state machines
* Composable nodes (especially in subsystems)
* Recovery

# Notes

```
Localization system recovery
    LiDAR-based localization fails, transitions output pose to GPS
    Localization Manager
        Monitoring the performance of the system and heartbeat status
        Implements recovery internally

    Separate node from the health monitoring
        Monitor and then decide when to shut down
        Checking heartbeat status of the nodes
```

# Manually controlling the lifecycle state of the system

## Startup (configure, activate)
```
ros2 service call /carma_system_controller/manage_nodes ros2_lifecycle_manager_msgs/ManageLifecycleNodes "{ command: 0 }"
```

## Pause (deactivate):
```
ros2 service call /carma_system_controller/manage_nodes ros2_lifecycle_manager_msgs/ManageLifecycleNodes "{ command: 1 }"
```

## Resume (activate):
```
ros2 service call /carma_system_controller/manage_nodes ros2_lifecycle_manager_msgs/ManageLifecycleNodes "{ command: 2 }"
```

## Reset (deactivate, cleanup)
```
ros2 service call /carma_system_controller/manage_nodes ros2_lifecycle_manager_msgs/ManageLifecycleNodes "{ command: 3 }"
```

## Shutdown (deactivate, cleanup, shutdown)
```
ros2 service call /carma_system_controller/manage_nodes ros2_lifecycle_manager_msgs/ManageLifecycleNodes "{ command: 4 }"
```

# Task List

```
[ ] CameraDriverClient's helper class: add initialize() like plugin?
[ ] Add some example parameters to plugin (to demonstrate how to separate parameters) with two plugins
[ ] When setting the composable node container to respawn, the contained composable nodes aren't reloaded
```
