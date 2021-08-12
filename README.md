
# Features/Capabilities

* CMake macro that can be included by CARMA packages ("carma_project()")
    * A single source for compiler settings, etc.
* CarmaNode
    * Derives from rclcpp_lifecycle::LifecycleNode
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

* Is there a lifecycle manager for each subsystem?
    * With one central lifecycle manager that interacts with the subsystems?
* How does the CARMA architecture intersect with Docker containers?  Per subsystem?
* What is the recovery strategy?
    * Notify the system monitor?
    * Monitor and reset a subsystem?
