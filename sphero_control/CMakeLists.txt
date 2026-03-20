cmake_minimum_required(VERSION 3.8)
project(sphero_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

install(DIRECTORY launch src DESTINATION share/${PROJECT_NAME})

add_executable(tangentional_escape src/tangentional_escape.cpp)
ament_target_dependencies(tangentional_escape rclcpp geometry_msgs sensor_msgs)
install(TARGETS tangentional_escape DESTINATION lib/${PROJECT_NAME})

add_executable(tr_without_fb src/tr_without_fb.cpp)
ament_target_dependencies(tr_without_fb rclcpp geometry_msgs sensor_msgs)
install(TARGETS tr_without_fb DESTINATION lib/${PROJECT_NAME})

add_executable(tr_fb_lyapunov src/tr_fb_lyapunov.cpp)
ament_target_dependencies(tr_fb_lyapunov rclcpp geometry_msgs sensor_msgs)
install(TARGETS tr_fb_lyapunov DESTINATION lib/${PROJECT_NAME})

add_executable(tr_fb_v_alpha_control src/tr_fb_v_alpha_control.cpp)
ament_target_dependencies(tr_fb_v_alpha_control rclcpp geometry_msgs sensor_msgs std_msgs)
install(TARGETS tr_fb_v_alpha_control DESTINATION lib/${PROJECT_NAME})

add_executable(tr_fb_geometry_controller src/tr_fb_geometry_controller.cpp)
ament_target_dependencies(tr_fb_geometry_controller rclcpp geometry_msgs sensor_msgs std_msgs)
install(TARGETS tr_fb_geometry_controller DESTINATION lib/${PROJECT_NAME})

add_executable(tr_fb_almost_global_controller src/tr_fb_almost_global_controller.cpp)
ament_target_dependencies(tr_fb_almost_global_controller rclcpp geometry_msgs sensor_msgs std_msgs)
install(TARGETS tr_fb_almost_global_controller DESTINATION lib/${PROJECT_NAME})