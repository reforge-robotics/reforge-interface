cmake_minimum_required(VERSION 3.24)
project(standard_bots_shaper_example LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ReforgeShaper 2.0.9 CONFIG REQUIRED)
find_package(ReforgeShaperRos2 CONFIG REQUIRED)

set(STANDARD_BOTS_MODEL_DIRECTORY
    "${CMAKE_CURRENT_SOURCE_DIR}/../../models/current/shaper")
set(STANDARD_BOTS_URDF "${CMAKE_CURRENT_SOURCE_DIR}/../../urdf/modelone.urdf")

add_library(standard_bots_ros_transport standard_bots_ros_transport.cpp)
target_compile_features(standard_bots_ros_transport PUBLIC cxx_std_17)
target_include_directories(standard_bots_ros_transport PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
ament_target_dependencies(standard_bots_ros_transport trajectory_msgs)

add_executable(standard_bots_shaper_example main.cpp)
target_compile_features(standard_bots_shaper_example PRIVATE cxx_std_17)
target_compile_definitions(
  standard_bots_shaper_example PRIVATE
  SHAPER_CPP_DEFAULT_MODEL_DIRECTORY="${STANDARD_BOTS_MODEL_DIRECTORY}"
  SHAPER_CPP_DEFAULT_URDF="${STANDARD_BOTS_URDF}"
  SHAPER_CPP_REFORGE_VERSION="${ReforgeShaper_VERSION}")
target_link_libraries(
  standard_bots_shaper_example standard_bots_ros_transport
  ReforgeShaper::backend ReforgeShaper::ros2)
ament_target_dependencies(standard_bots_shaper_example trajectory_msgs)

include(CTest)
if(BUILD_TESTING)
  add_test(NAME standard_bots_shaper_example
           COMMAND standard_bots_shaper_example
                   "${STANDARD_BOTS_MODEL_DIRECTORY}" "${STANDARD_BOTS_URDF}")
  add_executable(standard_bots_ros_transport_tests
    tests/test_standard_bots_ros_transport.cpp)
  target_compile_features(standard_bots_ros_transport_tests PRIVATE cxx_std_17)
  target_link_libraries(standard_bots_ros_transport_tests standard_bots_ros_transport)
  ament_target_dependencies(standard_bots_ros_transport_tests trajectory_msgs)
  add_test(NAME standard_bots_ros_transport_tests
           COMMAND standard_bots_ros_transport_tests)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  foreach(target standard_bots_ros_transport standard_bots_shaper_example
                 standard_bots_ros_transport_tests)
    if(TARGET ${target})
      target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
  endforeach()
endif()

ament_package()
