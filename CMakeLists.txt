cmake_minimum_required(VERSION 3.14)
project(uflash CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Reuse the local upac project as a normal CMake dependency.
add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../upac"
    "${CMAKE_CURRENT_BINARY_DIR}/upac-build"
    EXCLUDE_FROM_ALL
)

# Find libusb-1.0
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

# FTXUI — TUI library for uflash-tui
include(FetchContent)
FetchContent_Declare(
    ftxui
    GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
    GIT_TAG        v5.0.0
    GIT_SHALLOW    TRUE
)
# Build only what we need; skip FTXUI's own tests/examples
set(FTXUI_BUILD_DOCS    OFF CACHE BOOL "" FORCE)
set(FTXUI_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(FTXUI_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(ftxui)

# ── uflash (CLI) ──────────────────────────────────────────────────────────────
add_executable(uflash
    src/main.cpp
    src/cli_options.cpp
    src/flash_workflow.cpp
    src/usb_device.cpp
    src/bsl_protocol.cpp
    src/transfer_pipeline.cpp
)

target_include_directories(uflash PRIVATE include ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(uflash PRIVATE PkgConfig::LIBUSB upac_lib pugixml)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(uflash PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ── uflash-tui ────────────────────────────────────────────────────────────────
add_executable(uflash-tui
    src/tui_app.cpp
)

target_include_directories(uflash-tui PRIVATE include ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(uflash-tui
    PRIVATE
    upac_lib
    pugixml
    ftxui::screen
    ftxui::dom
    ftxui::component
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(uflash-tui PRIVATE -Wall -Wextra -Wpedantic)
endif()
