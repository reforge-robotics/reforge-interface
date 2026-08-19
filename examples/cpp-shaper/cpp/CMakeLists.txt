cmake_minimum_required(VERSION 3.24)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(ReforgeShaperExample VERSION 0.1.0 LANGUAGES CXX)
    find_package(ReforgeShaper 2.0.9 CONFIG REQUIRED)
endif()

if(NOT TARGET ReforgeShaper::backend)
    message(FATAL_ERROR "ReforgeShaper::backend is required by the C++ example")
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
execute_process(
    COMMAND
        "${Python3_EXECUTABLE}" -c
        "import matplotlib; import numpy"
    RESULT_VARIABLE _reforge_shaper_plot_import_result
    ERROR_VARIABLE _reforge_shaper_plot_import_error
)
if(NOT _reforge_shaper_plot_import_result EQUAL 0)
    string(STRIP "${_reforge_shaper_plot_import_error}" _plot_import_error)
    message(
        FATAL_ERROR
        "The C++ example requires Python Matplotlib and NumPy. On Ubuntu "
        "24.04 install python3-dev, python3-matplotlib, and python3-numpy. "
        "Python import error: ${_plot_import_error}"
    )
endif()

set(_reforge_shaper_example_package_version "unknown")
if(DEFINED ReforgeShaper_VERSION)
    set(_reforge_shaper_example_package_version "${ReforgeShaper_VERSION}")
elseif(DEFINED PROJECT_VERSION)
    set(_reforge_shaper_example_package_version "${PROJECT_VERSION}")
endif()

add_library(
    reforge_shaper_example_utility STATIC
    example_utility.cpp
    example_plot_data.cpp
)
target_compile_features(reforge_shaper_example_utility PUBLIC cxx_std_17)
target_include_directories(
    reforge_shaper_example_utility PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(
    reforge_shaper_example_utility PUBLIC ReforgeShaper::backend
)

add_library(reforge_shaper_example_plotting STATIC example_plotting.cpp)
target_compile_features(reforge_shaper_example_plotting PRIVATE cxx_std_17)
target_compile_definitions(
    reforge_shaper_example_plotting PRIVATE WITHOUT_NUMPY
)
target_include_directories(
    reforge_shaper_example_plotting
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_include_directories(
    reforge_shaper_example_plotting
    SYSTEM PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/third_party"
)
target_link_libraries(
    reforge_shaper_example_plotting
    PRIVATE reforge_shaper_example_utility Python3::Python
)

get_target_property(
    _reforge_backend_links ReforgeShaper::backend INTERFACE_LINK_LIBRARIES
)
if("${_reforge_backend_links}" MATCHES "Python3|matplotlib|numpy")
    message(
        FATAL_ERROR
        "ReforgeShaper::backend must not expose example plotting dependencies"
    )
endif()

add_executable(shaper_example_usage shaper_example_usage.cpp)
target_compile_features(shaper_example_usage PRIVATE cxx_std_17)
target_compile_definitions(
    shaper_example_usage
    PRIVATE
        REFORGE_SHAPER_EXAMPLE_PACKAGE_VERSION="${_reforge_shaper_example_package_version}"
)
target_link_libraries(
    shaper_example_usage
    PRIVATE
        reforge_shaper_example_plotting
        reforge_shaper_example_utility
        ReforgeShaper::backend
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(
        reforge_shaper_example_utility PRIVATE -Wall -Wextra -Wpedantic -Werror
    )
    target_compile_options(
        shaper_example_usage PRIVATE -Wall -Wextra -Wpedantic -Werror
    )
    target_compile_options(
        reforge_shaper_example_plotting
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Werror
            -Wno-deprecated-declarations
    )
endif()

add_custom_command(
    TARGET shaper_example_usage POST_BUILD
    COMMAND
        "${CMAKE_COMMAND}" -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/../assets"
        "$<TARGET_FILE_DIR:shaper_example_usage>/assets"
    COMMENT "Copying the shared Standard Bots example assets"
)
add_custom_command(
    TARGET shaper_example_usage POST_BUILD
    COMMAND
        "${CMAKE_COMMAND}" -E make_directory
        "$<TARGET_FILE_DIR:shaper_example_usage>/validation"
    COMMAND
        "${CMAKE_COMMAND}" -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/validation/expected_metrics.json"
        "$<TARGET_FILE_DIR:shaper_example_usage>/validation/expected_metrics.json"
    COMMENT "Copying the canonical C++ runtime baseline manifest"
)

option(
    REFORGE_SHAPER_EXAMPLE_ENABLE_PARITY_TESTS
    "Run Python/C++ Standard Bots native parity and hardware-free audits"
    OFF
)
if(REFORGE_SHAPER_EXAMPLE_ENABLE_PARITY_TESTS)
    include(CTest)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(
        NAME shaper_example_standard_bots_parity
        COMMAND
            "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/validation/compare_with_python.py"
            --binary "$<TARGET_FILE:shaper_example_usage>"
            --assets-dir "${CMAKE_CURRENT_SOURCE_DIR}/../assets"
            --expected-metrics
            "${CMAKE_CURRENT_SOURCE_DIR}/validation/expected_metrics.json"
    )
    add_test(
        NAME shaper_example_hardware_free_audit
        COMMAND
            "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/validation/audit_hardware_free.py"
            --binary "$<TARGET_FILE:shaper_example_usage>"
            --source "${CMAKE_CURRENT_SOURCE_DIR}/shaper_example_usage.cpp"
            --assets-dir "${CMAKE_CURRENT_SOURCE_DIR}/../assets"
    )
    add_test(
        NAME shaper_example_plot_regression
        COMMAND
            "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/validation/validate_plots.py"
            --binary "$<TARGET_FILE:shaper_example_usage>"
            --assets-dir "${CMAKE_CURRENT_SOURCE_DIR}/../assets"
            --expected
            "${CMAKE_CURRENT_SOURCE_DIR}/validation/expected_plots.json"
    )
    add_test(
        NAME shaper_example_consumer_failures
        COMMAND
            "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/validation/validate_consumer.py"
            --binary "$<TARGET_FILE:shaper_example_usage>"
            --source-dir "${CMAKE_CURRENT_SOURCE_DIR}"
            --assets-dir "${CMAKE_CURRENT_SOURCE_DIR}/../assets"
            --expected-metrics
            "${CMAKE_CURRENT_SOURCE_DIR}/validation/expected_metrics.json"
            --package-dir "${ReforgeShaper_DIR}"
            --cmake-prefix-path "${CMAKE_PREFIX_PATH}"
    )
endif()
