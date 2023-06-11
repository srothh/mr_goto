cmake_minimum_required(VERSION 3.8)
project(mr_pf)

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

add_library(pf src/particle_filter.cpp src/particle_filter_visualization.cpp)
target_include_directories(pf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pf PUBLIC c_std_99 cxx_std_23)  # Require C99 and C++17
ament_target_dependencies(pf
  tuw_geometry
  OpenCV)

add_executable(pf_node src/standalone_pf.cpp src/particle_filter_node.cpp src/particle_filter_node_parameter.cpp)
target_include_directories(pf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
#target_compile_features(pf_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(pf_node pf)
ament_target_dependencies(pf_node
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tuw_geometry
  OpenCV)

install(TARGETS pf_node
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
  TARGETS pf
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_libraries(
  pf
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
