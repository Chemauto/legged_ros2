cmake_minimum_required(VERSION 3.8)
project(bishe_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  bishe_bridge/dds_state_writer.py
  bishe_bridge/rl_cmd_vel_writer.py
  bishe_bridge/state_bridge_node.py
  bishe_bridge/odom_node.py
  bishe_bridge/box_pose_node.py
  bishe_bridge/scene_objects_node.py
  bishe_bridge/skill_status_node.py
  bishe_bridge/skill_command_node.py
  bishe_bridge/cmd_vel_node.py
  bishe_bridge/goal_pose_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
