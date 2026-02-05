cmake_minimum_required(VERSION 3.8)
project(legged_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(imu_complementary_filter REQUIRED)
find_package(unitree_go REQUIRED)

add_executable(mid360_imu_node
  src/mid360_imu.cpp
)
target_compile_features(mid360_imu_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(mid360_imu_node
  rclcpp
  sensor_msgs
  message_filters
  tf2
  tf2_ros
  imu_complementary_filter
)

add_executable(gravity_alignment_node
  src/gravity_alignment.cpp
)
target_compile_features(gravity_alignment_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(gravity_alignment_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(dual_imu_static_tf_node
  src/dual_imu_static_tf.cpp
)
target_compile_features(dual_imu_static_tf_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(dual_imu_static_tf_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  unitree_go
)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})
install(TARGETS
  mid360_imu_node
  gravity_alignment_node
  dual_imu_static_tf_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
