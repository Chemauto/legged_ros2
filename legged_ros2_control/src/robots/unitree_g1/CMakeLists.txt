find_package(unitree_hg REQUIRED) # from unitree_ros2

# ------------------------------------------------------------
# g1 system interface
# ------------------------------------------------------------

add_library(g1_system_interface SHARED
    g1_system_interface.cpp
    g1_lowlevel_node.cpp
    motor_crc_hg.cpp
)
target_compile_features(g1_system_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(g1_system_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
    g1_system_interface
    "rclcpp"
    "hardware_interface"
    "pluginlib"
    "rclcpp_lifecycle"
    "unitree_hg"
)
target_link_libraries(g1_system_interface
  legged_system_interface
)

install(
  TARGETS g1_system_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ------------------------------------------------------------
# g1 main loop
# ------------------------------------------------------------

add_executable(g1_main_loop
    g1_main_loop.cpp
    g1_wireless_controller.cpp
    ../../legged_ros2_control.cpp
)
target_compile_features(g1_main_loop PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(g1_main_loop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
    g1_main_loop
    "rclcpp"
    "geometry_msgs"
    "controller_manager"
    "controller_manager_msgs"
    "pluginlib"
    "rosgraph_msgs"
    "unitree_hg"
)
target_link_libraries(g1_main_loop
    g1_system_interface
    legged_system_interface
)

install(
    TARGETS g1_main_loop
    DESTINATION lib/${PROJECT_NAME}
)
