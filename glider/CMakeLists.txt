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
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
# ROS 2
if (BUILD_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(gps_msgs REQUIRED)
  find_package(dgps_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)

  set(node_plugins "")
endif()
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
  src/parameters.cpp
  src/time.cpp
  src/differential_gps.cpp
  src/odometry.cpp
  src/odometry_with_covariance.cpp
  src/point.cpp
  src/factor_manager.cpp
  src/glider.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    glog::glog
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
    gps_msgs
    dgps_msgs
    tf2_ros
  )

  add_executable(${PROJECT_NAME}_node
    ros/node.cpp
  )

  target_link_libraries(${PROJECT_NAME}_node
    rclcpp::rclcpp
    ${PROJECT_NAME}
    ${PROJECT_NAME}_ros
  )

  rclcpp_components_register_nodes(${PROJECT_NAME}_ros "GliderROS::GliderNode")
  set(node_plugins "${node_plugins}GliderROS::GliderNode;$<TARGET_FILE:${PROJECT_NAME}_ros>\n")

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
    dgps_msgs
    Eigen3
  )

  ament_package()
else()
  message("Configuring non-ROS installation")
  
  # Install the library
  install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )
  
  # Install headers
  install(DIRECTORY include/
    DESTINATION include
  )
  
  # Install config files
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
  
  # Generate and install CMake config files
  include(CMakePackageConfigHelpers)
  
  # Generate the config file that includes the exports
  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
  )
  
  # Generate the version file
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY AnyNewerVersion
  )
  
  # Install the config files
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION lib/cmake/${PROJECT_NAME}
  )
  
  # Install the export set
  install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
  )
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

    add_executable(params_test test/test_params.cpp)
    target_link_libraries(params_test
        GTest::GTest
        GTest::Main
        ${PROJECT_NAME}
    )

    add_executable(manager_test test/test_factor_manager.cpp)
    target_link_libraries(manager_test
        GTest::GTest
        GTest::Main
        ${PROJECT_NAME}
    )

    add_executable(glider_test test/test_glider.cpp)
    target_link_libraries(glider_test
        GTest::GTest
        GTest::Main
        ${PROJECT_NAME}
    )

    add_executable(odometry_test test/test_odometry.cpp)
    target_link_libraries(odometry_test
        GTest::GTest
        GTest::Main
        ${PROJECT_NAME}
    )

    add_executable(odometry_with_cov_test test/test_odometry_w_cov.cpp)
    target_link_libraries(odometry_with_cov_test
        GTest::GTest
        GTest::Main
        ${PROJECT_NAME}
    )

    add_executable(diff_gps_test test/test_differential_gps.cpp)
    target_link_libraries(diff_gps_test
        GTest::GTest
        GTest::Main
        ${PROJECT_NAME}
    )


    add_executable(ros_conversion_test test/test_ros_conversions.cpp)
    target_link_libraries(ros_conversion_test
        GTest::GTest
        GTest::Main
        ${PROJECT_NAME}
        ${PROJECT_NAME}_ros
    )

    include(GoogleTest)
    gtest_discover_tests(utm_test)
    gtest_discover_tests(params_test)
    gtest_discover_tests(manager_test)
    gtest_discover_tests(glider_test)
    gtest_discover_tests(odometry_test)
    gtest_discover_tests(odometry_with_cov_test)
    gtest_discover_tests(diff_gps_test)
    gtest_discover_tests(ros_conversion_test)
endif()
