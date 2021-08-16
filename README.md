
# Features/Capabilities

* CMake macro that can be included by CARMA packages ("carma_project()")
    * A single source for compiler settings, etc.

* CarmaNode
    * Derives from rclcpp_lifecycle::LifecycleNode (could also be a template that can use either Node or LifecycleNode)
    * Automatically creates a bond with the lifecycle manager(see: https://github.com/ros/bond_core)
    * System alert capable

* Lifecycle Manager
    * Manages the lifecycle nodes
    * Monitors bonds with the managed nodes, can detect lack of heartbeat
    * System alert capable

* Subsystems
    * Lifecycle management of a subsystem
    * Composable Nodes - likely used per-subsystem

* Recovery
    * What to do here?

# Architecture Questions

* All CARMA nodes as lifecycle nodes?

* All CARMA nodes as Composable nodes?

* Is there a lifecycle manager for each subsystem?
    * With one central lifecycle manager that interacts with the subsystems?

* How does the CARMA architecture intersect with Docker containers? Per subsystem?

* What is the recovery strategy?
    * Notify the system monitor?
    * Monitor and reset a subsystem?[ ] Add composable nodes to CarmaNode

* How does Launch interact with Lifecycle nodes and recovery?
    * Which component owns restarting the nodes?

Launch
Lifecycle nodes
Parameters
Subsystems
System Eventing and state machines
Composable nodes (especially in subsystems)
Recovery

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
[ ] Add composable nodes to CarmaNode
[ ]     NodeOptions constructor
[ ]     Macro used in the implementation file 
[ ]     Build magic
[ ] Make sure that Perception subsystem uses composable nodes
[ ]     Launch with ComposableNode
[ ]     Nice if there was some data passed that demonstrated this

[ ] Localization Subsystem
[ ]     Rename the WorldModel subsystem to be the Localization subsystem
[ ]     Get rid of intermediate controller
[ ]     Keep the health monitor
[ ]     Understand the current CARMA health monitor and bring in communication patterns (create a shell)

[ ] Perception Subsystem
[ ]     Get rid of intermediate controller
[ ]     Get rid of the health monitor

[ ] CARMA lifecycle manager
[ ]     Remove CARMA-specifics from ros2_lifecycle_manager
[ ]     Create a system_controller that derives from ros2_lifecycle_manager and adds CARMA-specifics

[ ] Launch
[ ]     Use only one central lifecycle manager (instead of per subsystem)

```

