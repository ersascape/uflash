cmake_minimum_required(VERSION 3.10)
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

# --- uflash Utility ---
add_executable(uflash
    src/main.cpp
    src/cli_options.cpp
    src/flash_workflow.cpp
    src/usb_device.cpp
    src/bsl_protocol.cpp
    src/transfer_pipeline.cpp
)

add_executable(uflash-tui
    src/tui_app.cpp
)

target_include_directories(uflash
    PRIVATE
    include
    ${LIBUSB_INCLUDE_DIRS}
)

target_include_directories(uflash-tui
    PRIVATE
    include
    ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(uflash
    PRIVATE
    PkgConfig::LIBUSB
    upac_lib
    pugixml
)

target_link_libraries(uflash-tui
    PRIVATE
    PkgConfig::LIBUSB
    upac_lib
    pugixml
)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(uflash PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(uflash-tui PRIVATE -Wall -Wextra -Wpedantic)
endif()
