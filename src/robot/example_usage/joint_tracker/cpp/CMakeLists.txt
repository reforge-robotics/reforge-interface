cmake_minimum_required(VERSION 3.16)
project(ReforgeJointTrackerExampleUsage LANGUAGES CXX)

set(CMAKE_FIND_USE_PACKAGE_REGISTRY FALSE)
set(CMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY FALSE)

find_package(ReforgeJointTracker CONFIG REQUIRED)
if(NOT TARGET ReforgeJointTracker::joint_tracker)
    message(FATAL_ERROR
        "The installed ReforgeJointTracker package did not define "
        "ReforgeJointTracker::joint_tracker."
    )
endif()

get_target_property(
    _reforge_joint_tracker_example_target_imported
    ReforgeJointTracker::joint_tracker
    IMPORTED
)
if(NOT _reforge_joint_tracker_example_target_imported)
    message(FATAL_ERROR
        "ReforgeJointTracker::joint_tracker must be an imported target "
        "provided by the installed ReforgeJointTracker package."
    )
endif()
get_target_property(
    _reforge_joint_tracker_example_target_includes
    ReforgeJointTracker::joint_tracker
    INTERFACE_INCLUDE_DIRECTORIES
)
if(NOT _reforge_joint_tracker_example_target_includes)
    message(FATAL_ERROR
        "ReforgeJointTracker::joint_tracker is missing its public "
        "INTERFACE_INCLUDE_DIRECTORIES export."
    )
endif()

include(CheckCXXSourceCompiles)
set(
    _reforge_joint_tracker_example_previous_try_compile_target_type
    "${CMAKE_TRY_COMPILE_TARGET_TYPE}"
)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(
    _reforge_joint_tracker_example_previous_required_includes
    "${CMAKE_REQUIRED_INCLUDES}"
)
set(
    CMAKE_REQUIRED_INCLUDES
    ${_reforge_joint_tracker_example_target_includes}
)
check_cxx_source_compiles([=[
#include <reforge_core/joint_tracker/config.hpp>

int main() {
    reforge::joint_tracker::JointTrackerConfig config;
    config.optimizer_options.n_apply_scale = 1.0e-9;
    return 0;
}
]=] REFORGE_JOINT_TRACKER_EXAMPLE_INSTALLED_SDK_HAS_OPTIMIZER_OPTIONS)
set(
    CMAKE_TRY_COMPILE_TARGET_TYPE
    "${_reforge_joint_tracker_example_previous_try_compile_target_type}"
)
set(
    CMAKE_REQUIRED_INCLUDES
    "${_reforge_joint_tracker_example_previous_required_includes}"
)
if(NOT REFORGE_JOINT_TRACKER_EXAMPLE_INSTALLED_SDK_HAS_OPTIMIZER_OPTIONS)
    message(FATAL_ERROR
        "The installed ReforgeJointTracker package does not provide "
        "JointTrackerConfig::optimizer_options. Install a newer Reforge SDK."
    )
endif()

add_executable(joint_tracker_example_usage main.cpp)
target_compile_features(joint_tracker_example_usage PRIVATE cxx_std_17)
set_target_properties(joint_tracker_example_usage PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(
    joint_tracker_example_usage
    PRIVATE ReforgeJointTracker::joint_tracker
)
target_compile_definitions(joint_tracker_example_usage PRIVATE
    REFORGE_JOINT_TRACKER_EXAMPLE_MODELS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../example_models")
