add_library(${library_name} SHARED
  carma_node.cpp
)

add_executable(event_pub
  publish_events.cpp
)
ament_target_dependencies(event_pub ${dependencies})

ament_target_dependencies(${library_name}
  rclcpp
  cav_msgs
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


install(TARGETS
  event_pub
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
