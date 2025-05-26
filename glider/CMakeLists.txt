cmake_minimum_required(VERSION 3.8)
project(glider)
#TODO better project name

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES glider
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
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
  ros/factor_node.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    gtsam
    yaml-cpp
    tbb
    ${catkin_LIBRARIES}
)

# executables
add_executable(glider_ros
  ros/node.cpp
)

target_link_libraries(glider_ros
  ${PROJECT_NAME}
)

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
