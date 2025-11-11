cmake_minimum_required(VERSION 3.8)
project(m2wr_twist)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

add_executable(m2wr_twist src/m2wr_twist.cpp)
ament_target_dependencies(m2wr_twist rclcpp geometry_msgs)
install(TARGETS m2wr_twist DESTINATION lib/${PROJECT_NAME})

add_executable(parse_lidar src/parse_lidar.cpp)
ament_target_dependencies(parse_lidar rclcpp sensor_msgs)
install(TARGETS parse_lidar DESTINATION lib/${PROJECT_NAME})

add_executable(get_odom src/get_odom.cpp)
ament_target_dependencies(get_odom rclcpp nav_msgs tf2 tf2_geometry_msgs)
install(TARGETS get_odom DESTINATION lib/${PROJECT_NAME})

add_executable(tr_without_fb src/tr_without_fb.cpp)
ament_target_dependencies(tr_without_fb rclcpp nav_msgs tf2 tf2_geometry_msgs)
install(TARGETS tr_without_fb DESTINATION lib/${PROJECT_NAME})

add_executable(dummy_subs src/dummy_subs.cpp)
ament_target_dependencies(dummy_subs rclcpp geometry_msgs)
install(TARGETS dummy_subs DESTINATION lib/${PROJECT_NAME})