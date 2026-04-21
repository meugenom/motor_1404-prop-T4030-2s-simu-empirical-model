cmake_minimum_required(VERSION 3.16)
project(motor_1404_prop_t4030-2s-empirical-model VERSION 1.3.0 LANGUAGES CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform selection
set(PLATFORM "posix" CACHE STRING "Target platform: posix | none | stm32")

# Libraries
add_library(motor_model STATIC
    src/core/motor.cpp
    src/platform/${PLATFORM}/platform.c
)

# Include directories
target_include_directories(motor_model PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/includes
)

if(PLATFORM STREQUAL "posix")
    # Compiler warnings
    add_compile_options(-Wall -Wextra -Wpedantic)
    # UNIT TEST POSIX
    add_executable(test_motor tests/posix/test_motor.cpp)
    target_link_libraries(test_motor PRIVATE motor_model)
    enable_testing()
    add_test(NAME motor_tests COMMAND test_motor)
endIf()

# === AUSGABE ===
message(STATUS "motor_1404_prop_t4030-2s-empirical-model - Build konfiguriert")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")