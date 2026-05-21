cmake_minimum_required(VERSION 3.8)
project(go2_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY config launch meshes urdf mjcf rviz2 dae
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  include(CTest)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  add_test(
    NAME test_bringup_rl_policy_profile
    COMMAND ${Python3_EXECUTABLE} -m unittest
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_bringup_rl_policy_profile.py
  )

  add_test(
    NAME test_bringup_nav_launch
    COMMAND ${Python3_EXECUTABLE} -m unittest
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_bringup_nav_launch.py
  )

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
