cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
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

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(LIBGPIOD REQUIRED libgpiod)
#include_directories(${LIBGPIOD_INCLUDE_DIRS})

# Manually find the libgpiod C++ library
find_library(GPIOD_LIB NAMES gpiodcxx REQUIRED)
include_directories(/usr/include)


add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

add_executable(motor_driver_node src/motor_driver_node.cpp)
target_link_libraries(motor_driver_node ${GPIOD_LIB})
#target_include_directories(motor_driver_node PRIVATE ${LIBGPIOD_INCLUDE_DIRS})
#target_link_libraries(motor_driver_node ${LIBGPIOD_LIBRARIES})
ament_target_dependencies(motor_driver_node rclcpp)

add_executable(motor_controller src/motor_controller.cpp)
target_link_libraries(motor_controller ${GPIOD_LIB})
#target_include_directories(motor_driver_node PRIVATE ${LIBGPIOD_INCLUDE_DIRS})
#target_link_libraries(motor_driver_node ${LIBGPIOD_LIBRARIES})
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
  motor_driver_node
  cpp_node
  robot_news_station
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
