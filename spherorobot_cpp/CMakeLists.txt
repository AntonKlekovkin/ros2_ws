cmake_minimum_required(VERSION 3.8)
project(spherorobot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# install( DIRECTORY src DESTINATION share/${PROJECT_NAME})

add_library(pendulum_drive_plugin SHARED plugins/pendulum_drive_plugin.cpp)

target_link_libraries(pendulum_drive_plugin
  ${gazebo_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${gazebo_msgs_LIBRARIES}
)

ament_target_dependencies(pendulum_drive_plugin
  gazebo
  rclcpp
  gazebo_ros
  std_msgs
  sensor_msgs
  gazebo_msgs
)

install(TARGETS pendulum_drive_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

install(DIRECTORY launch urdf meshes src plugins worlds config DESTINATION share/${PROJECT_NAME})

add_executable(platform_control src/platform_control.cpp)
ament_target_dependencies(platform_control rclcpp geometry_msgs)
install(TARGETS platform_control DESTINATION lib/${PROJECT_NAME})

add_executable(sphero_control src/sphero_control.cpp)
ament_target_dependencies(sphero_control rclcpp std_msgs sensor_msgs)
install(TARGETS sphero_control DESTINATION lib/${PROJECT_NAME})