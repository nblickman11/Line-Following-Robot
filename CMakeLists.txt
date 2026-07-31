cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# must find gstreamer manually
# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0>=1.14)
pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

include_directories(
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_BASE_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
)

link_directories(
  ${GSTREAMER_LIBRARY_DIRS}
)

# Manually find the libgpiod C++ library
find_library(GPIOD_LIB NAMES gpiodcxx REQUIRED)
include_directories(/usr/include)

add_executable(motor_controller src/motor_controller.cpp)
target_link_libraries(motor_controller ${GPIOD_LIB})
ament_target_dependencies(motor_controller rclcpp geometry_msgs)

add_executable(camera_driver_node src/camera_driver_node.cpp)
ament_target_dependencies(camera_driver_node rclcpp sensor_msgs image_transport cv_bridge OpenCV)

add_executable(image_filter_node src/image_filter_node.cpp)
ament_target_dependencies(image_filter_node rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(line_follower_node src/line_follower_node.cpp)
ament_target_dependencies(line_follower_node rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)

target_link_libraries(camera_driver_node
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_BASE_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
)

install(TARGETS
  line_follower_node
  image_filter_node
  camera_driver_node
  motor_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
