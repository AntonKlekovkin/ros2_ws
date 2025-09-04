cmake_minimum_required(VERSION 3.8)
project(diff_drive_gazebo_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Создаем библиотеку плагина
add_library(diff_drive_plugin SHARED src/diff_drive_plugin.cpp)

# Настраиваем include директории
target_include_directories(diff_drive_plugin PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
)

# Настраиваем линковку
target_link_libraries(diff_drive_plugin
  ${GAZEBO_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
)

# Указываем зависимости
ament_target_dependencies(diff_drive_plugin
  rclcpp
  geometry_msgs
  sensor_msgs
  gazebo_ros
  nav_msgs
  tf2_geometry_msgs
)

# Устанавливаем библиотеку
install(TARGETS diff_drive_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
