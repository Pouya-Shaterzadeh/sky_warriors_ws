cmake_minimum_required(VERSION 3.8)
project(skyw_swarm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SetFormation.action"
  DEPENDENCIES action_msgs builtin_interfaces
)

# Install Python executables
install(PROGRAMS
  skyw_swarm/formation_server.py
  skyw_swarm/formation_client.py
  skyw_swarm/px4_pose_bridge.py
  skyw_swarm/formation_math.py
  skyw_swarm/swarm_controller.py
  skyw_swarm/px4_offboard_bridge.py
  skyw_swarm/mission_sequencer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install action files
install(DIRECTORY
  action
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
