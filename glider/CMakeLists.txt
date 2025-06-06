cmake_minimum_required(VERSION 3.8)
project(glider)
#TODO better project name

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(node_plugins "")

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
  ${yaml-cpp_INCLUDE_DIRS}
)

# Libraries  
# Make sure to add your source files
add_library(${PROJECT_NAME} SHARED
  src/imu_buffer.cpp
  src/factor_manager.cpp
  src/params.cpp
  src/quaternion.cpp
  src/odometry.cpp
  src/state.cpp
  ros/factor_node.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    gtsam
    gtsam_unstable
    yaml-cpp
    tbb
    rclcpp::rclcpp
    rclcpp_components::component
    ${sensor_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
)

rclcpp_components_register_nodes(${PROJECT_NAME} "glider::FactorNode")
set(node_plugins "${node_plugins}glider::FactorNode;$<TARGET_FILE:${PROJECT_NAME}>\n")

# executables
add_executable(glider_composition
  ros/node.cpp
)

target_link_libraries(glider_composition
  rclcpp::rclcpp
  ${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files if you have any
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  glider_composition
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_filters
  tf2_eigen
  Eigen3
)

ament_package()

# TESTS
enable_testing()
# build utm test
add_executable(utm_test test/test_ll_to_utm.cpp)
target_link_libraries(utm_test
  GTest::GTest
  GTest::Main
  ${PROJECT_NAME}
)

#build params test
add_executable(params_test test/test_params.cpp)
target_link_libraries(params_test
  GTest::GTest
  GTest::Main
  ${PROJECT_NAME}
)

#build factor tests
add_executable(factor_test test/test_factors.cpp)
target_link_libraries(factor_test
  GTest::GTest
  GTest::Main
  ${PROJECT_NAME}
)

add_executable(quaternion_test test/test_quaternion.cpp)
target_link_libraries(quaternion_test
  GTest::GTest
  GTest::Main
  ${PROJECT_NAME}
)

add_executable(heading_test test/test_heading.cpp)
target_link_libraries(heading_test
  GTest::GTest
  GTest::Main
  ${PROJECT_NAME}
)

include(GoogleTest)
gtest_discover_tests(utm_test)
gtest_discover_tests(params_test)
gtest_discover_tests(factor_test)
gtest_discover_tests(quaternion_test)
gtest_discover_tests(heading_test)
