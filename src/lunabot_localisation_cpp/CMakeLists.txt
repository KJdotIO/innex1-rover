cmake_minimum_required(VERSION 3.8)
project(lunabot_localisation_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(legal_lidar_filter_core
  src/legal_lidar_filter_core.cpp
)
set_target_properties(legal_lidar_filter_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(legal_lidar_filter_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(legal_lidar_filter_core
  sensor_msgs
)

add_library(legal_lidar_filter_component SHARED
  src/legal_lidar_filter_node.cpp
)
target_include_directories(legal_lidar_filter_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(legal_lidar_filter_component
  legal_lidar_filter_core
)
ament_target_dependencies(legal_lidar_filter_component
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
)

rclcpp_components_register_node(legal_lidar_filter_component
  PLUGIN "lunabot_localisation_cpp::LegalLidarFilterNode"
  EXECUTABLE legal_lidar_filter_cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS legal_lidar_filter_core legal_lidar_filter_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_legal_lidar_filter_core
    test/test_legal_lidar_filter_core.cpp
  )
  target_link_libraries(test_legal_lidar_filter_core
    legal_lidar_filter_core
  )
  target_include_directories(test_legal_lidar_filter_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  )

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
)
ament_package()
