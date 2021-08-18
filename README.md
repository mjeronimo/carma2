
# Features/Capabilities

The example code in this repository provides the following features:

* CMake code
    * Can be included by CARMA packages
    * The carma_project() macro serves as a single source for compiler settings, etc.

* CarmaNode
    * Derives from rclcpp_lifecycle::LifecycleNode (could also be a template that can use either Node or LifecycleNode)
    * Automatically creates a bond with the lifecycle manager (see: https://github.com/ros/bond_core)
    * System alert capable

* Lifecycle Manager
    * Manages the lifecycle nodes, initiating state transitions
    * Monitors bonds with the managed nodes, can detect lack of heartbeat and initialize restarted nodes
    * System alert capable

* Subsystems
    * Lifecycle management of a subsystem
    * Composable Nodes - likely used per-subsystem

* Recovery
    * What to do here?

# Architecture Questions

* All CARMA nodes as lifecycle nodes? Yes

* All CARMA nodes as Composable nodes? Yes

* Is there a lifecycle manager for each subsystem? No, only one central lifecycle manager to start.
    * With one central lifecycle manager that interacts with the subsystems?

* What is the recovery strategy?
    * Notify the system monitor?
    * Monitor and reset a subsystem?

* How does Launch interact with Lifecycle nodes and recovery?
    * Which component owns restarting the nodes?

* Topic
    * Launch
    * Lifecycle nodes
    * Parameters
    * Subsystems
    * System Eventing and state machines
    * Composable nodes (especially in subsystems)
    * Recovery

Localization system recovery
    LiDAR-based localization fails, transitions output pose to GPS
    Localization Manager
        Monitoring the performance of the system and heartbeat status
        Implements recovery internally

    Separate node from the health monitoring
        Monitor and then decide when to shut down
        Checking heartbeat status of the nodes

# Task List

```
[x] Add composable nodes to CarmaNode
[x]     NodeOptions constructor
[x]     Macro used in the implementation file 
[x]     Build magic
[x] Make sure that Perception subsystem uses composable nodes
[x]     Launch with ComposableNode
[x]     Nice if there was some data passed that demonstrated this

[x] Localization Subsystem
[x]     Rename the WorldModel subsystem to be the Localization subsystem
[x]     Get rid of intermediate controller
[x]     Keep the health monitor
[ ]     Understand the current CARMA health monitor and bring in communication patterns (create a shell)

[x] Perception Subsystem
[x]     Get rid of intermediate controller
[x]     Get rid of the health monitor

[x] CARMA lifecycle manager
[x]     Remove CARMA-specifics from ros2_lifecycle_manager
[x]     Create a system_controller that derives from ros2_lifecycle_manager and adds CARMA-specifics

[x] Launch
[x]     Use only one central lifecycle manager (instead of per subsystem)

```