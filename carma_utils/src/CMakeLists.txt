add_library(${library_name} SHARED
  carma_node.cpp
)

ament_target_dependencies(${library_name}
  bond
  bondcpp
  rclcpp
  rclcpp_lifecycle
  ros2_lifecycle_manager_msgs
  ros2_utils
)

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
