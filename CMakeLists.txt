cmake_minimum_required(VERSION 3.16)
project(motor_1404_prop_t4030-2s-empirical-model VERSION 1.3.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform selection
set(PLATFORM "posix" CACHE STRING "Target platform: posix | stm32")

# Libraries
add_library(motor_model STATIC
    src/core/motor.cpp
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

if(PLATFORM STREQUAL "stm32")    
    add_executable(test_motor_stm32
        tests/stm32/test_motor_stm32.cpp 
        tests/stm32/startup.c)
    target_include_directories(test_motor_stm32 PRIVATE tests/stm32)

    target_link_libraries(test_motor_stm32 PRIVATE motor_model)

    # Say Linker to use our memory map
    target_link_options(test_motor_stm32 PRIVATE
        "-T${CMAKE_CURRENT_SOURCE_DIR}/tests/stm32/stm32f4.ld"
        "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/test_motor_stm32.map"
    )

endif()

# === AUSGABE ===
message(STATUS "motor_1404_prop_t4030-2s-empirical-model - Build konfiguriert")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")