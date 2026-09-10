cmake_minimum_required(VERSION 3.16)
project(ReforgeJointTrackerExampleUsage LANGUAGES CXX)

option(
    REFORGE_JOINT_TRACKER_EXAMPLE_REQUIRE_INSTALLED_SDK
    "Require an installed ReforgeJointTracker CMake package instead of using the repository source fallback"
    ON
)

set(CMAKE_FIND_USE_PACKAGE_REGISTRY FALSE)
set(CMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY FALSE)

if(
    NOT REFORGE_JOINT_TRACKER_EXAMPLE_REQUIRE_INSTALLED_SDK
    AND ReforgeJointTracker_DIR
    AND NOT EXISTS "${ReforgeJointTracker_DIR}/ReforgeJointTrackerTargets.cmake"
)
    unset(ReforgeJointTracker_DIR CACHE)
endif()

if(REFORGE_JOINT_TRACKER_EXAMPLE_REQUIRE_INSTALLED_SDK)
    find_package(ReforgeJointTracker CONFIG REQUIRED)
endif()

if(
    REFORGE_JOINT_TRACKER_EXAMPLE_REQUIRE_INSTALLED_SDK
    AND NOT TARGET ReforgeJointTracker::joint_tracker
)
    message(FATAL_ERROR
        "The installed ReforgeJointTracker package did not define "
        "ReforgeJointTracker::joint_tracker."
    )
endif()

if(REFORGE_JOINT_TRACKER_EXAMPLE_REQUIRE_INSTALLED_SDK)
    # Keep this preflight scoped to the example's only imported dependency. It
    # catches incomplete SDK package exports before compiling the example and
    # leaves unrelated native Shaper package checks untouched.
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
endif()

if(REFORGE_JOINT_TRACKER_EXAMPLE_REQUIRE_INSTALLED_SDK)
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
    get_target_property(
        _reforge_joint_tracker_example_sdk_includes
        ReforgeJointTracker::joint_tracker
        INTERFACE_INCLUDE_DIRECTORIES
    )
    if(_reforge_joint_tracker_example_sdk_includes)
        set(
            CMAKE_REQUIRED_INCLUDES
            ${_reforge_joint_tracker_example_sdk_includes}
        )
    endif()
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
            "JointTrackerConfig::optimizer_options. Example 2B requires this "
            "newer n_apply_scale API. Upgrade reforge-core-joint-tracker to a "
            "package built from this branch/release, or configure this "
            "repository checkout with "
            "-DREFORGE_JOINT_TRACKER_EXAMPLE_REQUIRE_INSTALLED_SDK=OFF to use "
            "the local native source."
        )
    endif()
endif()

if(NOT TARGET ReforgeJointTracker::joint_tracker)
    get_filename_component(
        _reforge_joint_tracker_example_repo_root
        "${CMAKE_CURRENT_LIST_DIR}/../../../../.."
        ABSOLUTE
    )
    set(
        _reforge_joint_tracker_example_native_source
        "${_reforge_joint_tracker_example_repo_root}/src/core_sdk/native"
    )
    if(NOT EXISTS "${_reforge_joint_tracker_example_native_source}/CMakeLists.txt")
        message(FATAL_ERROR
            "ReforgeJointTracker was not found and the repository source fallback "
            "is unavailable. Install reforge-core-joint-tracker or configure with "
            "-DCMAKE_PREFIX_PATH=<install-prefix>."
        )
    endif()

    # Locate the header-only nlohmann_json dependency for the source fallback.
    # Probe several interpreters because the interpreter CMake selects is not
    # always the one the developer or CI installed the Python packages into.
    find_package(Python3 COMPONENTS Interpreter QUIET)
    find_program(_reforge_joint_tracker_example_path_python NAMES python3 python)
    file(GLOB _reforge_joint_tracker_example_repo_pythons
        "${_reforge_joint_tracker_example_repo_root}/.venv/bin/python3"
        "${_reforge_joint_tracker_example_repo_root}/venv/bin/python3"
    )
    set(_reforge_joint_tracker_example_python_candidates "")
    if(DEFINED ENV{VIRTUAL_ENV})
        list(APPEND _reforge_joint_tracker_example_python_candidates
            "$ENV{VIRTUAL_ENV}/bin/python3")
    endif()
    list(APPEND _reforge_joint_tracker_example_python_candidates
        ${_reforge_joint_tracker_example_repo_pythons})
    if(Python3_Interpreter_FOUND)
        list(APPEND _reforge_joint_tracker_example_python_candidates
            "${Python3_EXECUTABLE}")
    endif()
    if(_reforge_joint_tracker_example_path_python)
        list(APPEND _reforge_joint_tracker_example_python_candidates
            "${_reforge_joint_tracker_example_path_python}")
    endif()
    list(REMOVE_DUPLICATES _reforge_joint_tracker_example_python_candidates)

    foreach(_reforge_joint_tracker_example_python
            IN LISTS _reforge_joint_tracker_example_python_candidates)
        if(REFORGE_NLOHMANN_JSON_INCLUDE_DIR)
            break()
        endif()
        if(NOT EXISTS "${_reforge_joint_tracker_example_python}")
            continue()
        endif()
        # `nlohmann_json.get_include()` is the package's own accessor and works
        # for editable, user-site, and virtualenv installs alike.
        execute_process(
            COMMAND
                "${_reforge_joint_tracker_example_python}" -c
                "import importlib.util, pathlib; spec = importlib.util.find_spec('nlohmann_json'); mod = importlib.import_module('nlohmann_json') if spec else None; p = pathlib.Path(mod.get_include()) if mod is not None and hasattr(mod, 'get_include') else None; print(p if p is not None and (p / 'nlohmann' / 'json.hpp').is_file() else '')"
            RESULT_VARIABLE _reforge_joint_tracker_example_python_result
            OUTPUT_VARIABLE _reforge_joint_tracker_example_nlohmann_include
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        if(
            _reforge_joint_tracker_example_python_result EQUAL 0
            AND _reforge_joint_tracker_example_nlohmann_include
        )
            set(
                REFORGE_NLOHMANN_JSON_INCLUDE_DIR
                "${_reforge_joint_tracker_example_nlohmann_include}"
                CACHE PATH
                "nlohmann_json include directory for repository fallback"
                FORCE
            )
        endif()
    endforeach()

    if(NOT REFORGE_NLOHMANN_JSON_INCLUDE_DIR)
        file(GLOB _reforge_joint_tracker_example_venv_nlohmann_candidates
            "${_reforge_joint_tracker_example_repo_root}/.venv/lib/python*/site-packages/nlohmann_json/include"
            "${_reforge_joint_tracker_example_repo_root}/venv/lib/python*/site-packages/nlohmann_json/include"
        )
        foreach(_reforge_joint_tracker_example_candidate
                IN LISTS _reforge_joint_tracker_example_venv_nlohmann_candidates)
            if(
                EXISTS
                    "${_reforge_joint_tracker_example_candidate}/nlohmann/json.hpp"
            )
                set(
                    REFORGE_NLOHMANN_JSON_INCLUDE_DIR
                    "${_reforge_joint_tracker_example_candidate}"
                    CACHE PATH
                    "nlohmann_json include directory for repository fallback"
                    FORCE
                )
                break()
            endif()
        endforeach()
    endif()

    if(NOT REFORGE_NLOHMANN_JSON_INCLUDE_DIR)
        message(FATAL_ERROR
            "The repository source fallback needs the header-only nlohmann_json "
            "3.12.0 dependency, but no interpreter provided it. Install it with "
            "`pip install nlohmann-json==3.12.0` into the environment you build "
            "from, or pass -DREFORGE_NLOHMANN_JSON_INCLUDE_DIR=<include-dir>."
        )
    endif()

    if(NOT REFORGE_SHAPER_CLARABEL_ROOT AND DEFINED ENV{REFORGE_SHAPER_CLARABEL_ROOT})
        set(
            REFORGE_SHAPER_CLARABEL_ROOT
            "$ENV{REFORGE_SHAPER_CLARABEL_ROOT}"
            CACHE PATH
            "Materialized Clarabel.cpp source root for the repository fallback"
            FORCE
        )
    endif()

    set(REFORGE_BUILD_SHAPER OFF CACHE BOOL "" FORCE)
    set(REFORGE_BUILD_JOINT_TRACKER ON CACHE BOOL "" FORCE)
    set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
    set(REFORGE_JOINT_TRACKER_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
    add_subdirectory(
        "${_reforge_joint_tracker_example_native_source}"
        "${CMAKE_CURRENT_BINARY_DIR}/reforge_core_native"
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
