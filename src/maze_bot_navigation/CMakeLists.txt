cmake_minimum_required(VERSION 3.8)
project(maze_bot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Add executables
add_executable(maze_navigator src/maze_navigator.cpp)
add_executable(vision_target_detector src/vision_target_detector.cpp)
ament_target_dependencies(maze_navigator
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
)

# Link OpenCV libraries
target_link_libraries(maze_navigator ${OpenCV_LIBRARIES})

# Vision target detector dependencies
ament_target_dependencies(vision_target_detector
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
)
target_link_libraries(vision_target_detector ${OpenCV_LIBRARIES})

# Remove advanced components for now

# Install targets
install(TARGETS
  maze_navigator
  vision_target_detector
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
