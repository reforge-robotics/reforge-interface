cmake_minimum_required(VERSION 3.24)
project(reforge_standard_bots_qualification VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ReforgeShaper 2.0.9 CONFIG REQUIRED)
find_package(ReforgeShaperRos2 CONFIG REQUIRED)

add_library(
  standard_bots_qualification
  src/harness.cpp
  src/ros_transport.cpp
)
target_compile_features(standard_bots_qualification PUBLIC cxx_std_17)
target_include_directories(
  standard_bots_qualification
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  standard_bots_qualification
  ReforgeShaper::backend ReforgeShaper::ros2
)
ament_target_dependencies(
  standard_bots_qualification rclcpp sensor_msgs trajectory_msgs
)

add_executable(standard_bots_qualification_cli src/main.cpp)
target_compile_features(standard_bots_qualification_cli PRIVATE cxx_std_17)
target_link_libraries(
  standard_bots_qualification_cli standard_bots_qualification
)
ament_target_dependencies(
  standard_bots_qualification_cli rclcpp sensor_msgs trajectory_msgs
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(
    standard_bots_qualification PRIVATE -Wall -Wextra -Wpedantic -Werror
  )
  target_compile_options(
    standard_bots_qualification_cli PRIVATE -Wall -Wextra -Wpedantic -Werror
  )
endif()

include(CTest)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(standard_bots_qualification_tests tests/test_harness.cpp)
  target_compile_definitions(
    standard_bots_qualification_tests
    PRIVATE REFORGE_INTERFACE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../.."
  )
  target_link_libraries(
    standard_bots_qualification_tests standard_bots_qualification
  )
  ament_target_dependencies(
    standard_bots_qualification_tests
    rclcpp sensor_msgs trajectory_msgs
  )
endif()

install(
  TARGETS standard_bots_qualification standard_bots_qualification_cli
  EXPORT standard_bots_qualification_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(standard_bots_qualification_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs trajectory_msgs)
ament_package()
