find_package(unitree_go REQUIRED) # from unitree_ros2

# ------------------------------------------------------------
# go2 system interface
# ------------------------------------------------------------

add_library(go2_system_interface SHARED
    go2_system_interface.cpp
    go2_lowlevel_node.cpp
    motor_crc.cpp
)
target_compile_features(go2_system_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(go2_system_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
    go2_system_interface
    "rclcpp"
    "hardware_interface"
    "pluginlib"
    "rclcpp_lifecycle"
    "unitree_go"
)
target_link_libraries(go2_system_interface
  legged_system_interface
)

install(
  TARGETS go2_system_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ------------------------------------------------------------
# go2 main loop
# ------------------------------------------------------------

add_executable(go2_main_loop
    go2_main_loop.cpp
    go2_wireless_controller.cpp
    ../../legged_ros2_control.cpp
)
target_compile_features(go2_main_loop PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(go2_main_loop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
    go2_main_loop
    "rclcpp"
    "geometry_msgs"
    "controller_manager"
    "controller_manager_msgs"
    "pluginlib"
    "rosgraph_msgs"
    "unitree_go"
)
target_link_libraries(go2_main_loop
    go2_system_interface
    legged_system_interface
)

install(
    TARGETS go2_main_loop
    DESTINATION lib/${PROJECT_NAME}
)
