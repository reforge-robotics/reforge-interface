cmake_minimum_required(VERSION 3.24)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(ReforgeShaperExample VERSION 0.1.0 LANGUAGES CXX)
    find_package(ReforgeShaper CONFIG REQUIRED)
endif()

find_package(nlohmann_json 3.12.0 EXACT CONFIG QUIET)

if(NOT TARGET nlohmann_json::nlohmann_json)
    set(
        REFORGE_NLOHMANN_JSON_INCLUDE_DIR
        ""
        CACHE PATH
        "Directory containing the pinned nlohmann/json.hpp header"
    )
    find_path(
        _reforge_nlohmann_json_include_dir
        NAMES nlohmann/json.hpp
        HINTS "${REFORGE_NLOHMANN_JSON_INCLUDE_DIR}"
    )
    if(NOT _reforge_nlohmann_json_include_dir)
        message(
            FATAL_ERROR
            "nlohmann_json 3.12.0 was not found. Install the pinned "
            "nlohmann-json Python package and pass "
            "-DREFORGE_NLOHMANN_JSON_INCLUDE_DIR=<package>/include."
        )
    endif()
    add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED)
    set_target_properties(
        nlohmann_json::nlohmann_json
        PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${_reforge_nlohmann_json_include_dir}"
    )
endif()

if(NOT TARGET ReforgeShaper::backend)
    message(FATAL_ERROR "ReforgeShaper::backend is required by the C++ example")
endif()

set(_reforge_shaper_example_package_version "unknown")
if(DEFINED ReforgeShaper_VERSION)
    set(_reforge_shaper_example_package_version "${ReforgeShaper_VERSION}")
elseif(DEFINED PROJECT_VERSION)
    set(_reforge_shaper_example_package_version "${PROJECT_VERSION}")
endif()

get_target_property(
    _reforge_backend_links ReforgeShaper::backend INTERFACE_LINK_LIBRARIES
)
if("${_reforge_backend_links}" MATCHES "Python3|matplotlib|numpy")
    message(
        FATAL_ERROR
        "ReforgeShaper::backend must not expose example plotting dependencies"
    )
endif()

add_executable(shaper_example_usage main.cpp)
target_compile_features(shaper_example_usage PRIVATE cxx_std_17)
target_compile_definitions(
    shaper_example_usage
    PRIVATE
        REFORGE_SHAPER_EXAMPLE_PACKAGE_VERSION="${_reforge_shaper_example_package_version}"
)
target_link_libraries(
    shaper_example_usage
    PRIVATE ReforgeShaper::backend nlohmann_json::nlohmann_json
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(
        shaper_example_usage PRIVATE -Wall -Wextra -Wpedantic -Werror
    )
endif()

add_custom_command(
    TARGET shaper_example_usage POST_BUILD
    COMMAND
        "${CMAKE_COMMAND}" -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/assets"
        "$<TARGET_FILE_DIR:shaper_example_usage>/assets"
    COMMAND
        "${CMAKE_COMMAND}" -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/../../../urdf/test_robot.urdf"
        "$<TARGET_FILE_DIR:shaper_example_usage>/assets/test_robot.urdf"
    COMMAND
        "${CMAKE_COMMAND}" -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/plotting/matplotlib_renderer.py"
        "$<TARGET_FILE_DIR:shaper_example_usage>/matplotlib_renderer.py"
    COMMENT "Copying the synthetic model fixture and test_robot.urdf"
)
