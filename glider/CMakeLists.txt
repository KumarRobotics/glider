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

option(BUILD_ROS "build the ros package" ON)
option(BUILD_TESTS "build test scripts" OFF)

if(BUILD_ROS)
    message("Build Ros is enabled")
    add_definitions(-DBUILD_ROS)
else()
    message("Build Ros is disabled")
endif()

if(BUILD_TESTS)
    message("Build Tests is enabled")
    add_definitions(-DBUILD_TESTS)
else()
    message("Build Tests is disabled")
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
  src/parameters.cpp
  src/quaternion.cpp
  src/odometry.cpp
  src/state.cpp
  src/glider.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    gtsam
    gtsam_unstable
    yaml-cpp
    tbb
)

if (BUILD_ROS)
  message("Building ROS Package with Ament")
  add_library(${PROJECT_NAME}_ros SHARED
    ros/conversions.cpp
    ros/glider_node.cpp

  )

  target_link_libraries(${PROJECT_NAME}_ros
    Eigen3::Eigen
    ${PROJECT_NAME}
  )

  ament_target_dependencies(${PROJECT_NAME}_ros 
    rclcpp
    rclcpp_components
    sensor_msgs
    geometry_msgs
    nav_msgs
  )

  add_executable(${PROJECT_NAME}_node
    ros/node.cpp
  )

  target_link_libraries(${PROJECT_NAME}_node
    rclcpp::rclcpp
    ${PROJECT_NAME}
    ${PROJECT_NAME}_ros
  )

  rclcpp_components_register_nodes(${PROJECT_NAME} "GliderROS::GliderNode")
  set(node_plugins "${node_plugins}GliderROS::GliderNode;$<TARGET_FILE:${PROJECT_NAME}>\n")

  install(DIRECTORY include/
    DESTINATION include/
  )

  # Install launch and config files
  install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
  )

  install(TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_ros
    EXPORT ${PROJECT_NAME}_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(TARGETS 
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
  )

  ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
  ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_ros)
  ament_export_include_directories(include)
  ament_export_dependencies(
    rclcpp
    rclcpp_components
    sensor_msgs
    geometry_msgs
    nav_msgs
    Eigen3
  )

  ament_package()
endif()

# TESTS
if (BUILD_TESTS)
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
endif()
