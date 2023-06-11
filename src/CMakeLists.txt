cmake_minimum_required(VERSION 3.8)
project(mr_ekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tuw_geometry REQUIRED)
find_package(tuw_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_library(ekf src/ekf.cpp src/ekf_visualization.cpp)
target_include_directories(ekf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ekf PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(ekf
  tuw_geometry
  OpenCV)

add_executable(ekf_node src/standalone_ekf.cpp src/ekf_node.cpp src/ekf_node_parameter.cpp)
target_include_directories(ekf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ekf_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(ekf_node ekf)
ament_target_dependencies(ekf_node  
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tuw_geometry
  tuw_geometry_msgs
  message_filters
  OpenCV)

install(TARGETS ekf_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}/
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ekf
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_libraries(
  ekf
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
