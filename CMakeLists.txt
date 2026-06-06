cmake_minimum_required(VERSION 3.16)
project(gr0x_motor VERSION 1.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Global settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Good warnings — catch issues early
add_compile_options(-Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# Core library — transport + motor protocol
# ---------------------------------------------------------------------------
add_library(gr0x_motor
    src/socketcan_transport.cpp
    src/robstride_motor.cpp
)

target_include_directories(gr0x_motor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---------------------------------------------------------------------------
# Test executables
# ---------------------------------------------------------------------------

# Milestone 1: discover motor on CAN bus
add_executable(scan_test test/scan_test.cpp)
target_link_libraries(scan_test PRIVATE gr0x_motor)

# Milestone 2: read position/velocity/temperature (uncomment when ready)
add_executable(feedback_test test/feedback_test.cpp)
target_link_libraries(feedback_test PRIVATE gr0x_motor)

# Milestone 3: enable + MIT position command (uncomment when ready)
add_executable(control_test test/control_test.cpp)
target_link_libraries(control_test PRIVATE gr0x_motor)
