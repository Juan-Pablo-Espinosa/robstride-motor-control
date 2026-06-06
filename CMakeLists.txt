cmake_minimum_required(VERSION 3.16)
project(gr0x_motor VERSION 1.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Global settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# Core library
# ---------------------------------------------------------------------------
add_library(gr0x_motor
    src/socketcan_transport.cpp
    src/robstride_motor.cpp
    src/motor_bus.cpp
)

target_include_directories(gr0x_motor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# motor_bus uses pthreads for std::thread
find_package(Threads REQUIRED)
target_link_libraries(gr0x_motor PUBLIC Threads::Threads)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
add_executable(scan_test test/scan_test.cpp)
target_link_libraries(scan_test PRIVATE gr0x_motor)

add_executable(feedback_test test/feedback_test.cpp)
target_link_libraries(feedback_test PRIVATE gr0x_motor)

add_executable(control_test test/control_test.cpp)
target_link_libraries(control_test PRIVATE gr0x_motor)

# ---------------------------------------------------------------------------
# Examples
# ---------------------------------------------------------------------------

# Terminal motor studio — requires ncurses
find_package(Curses REQUIRED)
add_executable(motor_studio examples/motor_studio.cpp)
target_link_libraries(motor_studio PRIVATE gr0x_motor ${CURSES_LIBRARIES})
target_include_directories(motor_studio PRIVATE ${CURSES_INCLUDE_DIR})
