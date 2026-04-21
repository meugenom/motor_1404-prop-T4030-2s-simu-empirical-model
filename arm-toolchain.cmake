# Say CMAke to build our project without knowing our platform
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# IMPORTANT: Tell CMake not to try to link an executable when checking the compiler,
# but to build only a static library (this solves the CMakeTestCCompiler error)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Specify the cross-compilers
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

# Flags for STM32F4 (Cortex-M4F)
set(CMAKE_C_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -g --specs=nosys.specs" CACHE INTERNAL "C Compiler flags")
#set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS}" CACHE INTERNAL "C++ Compiler flags")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-exceptions -fno-rtti -fno-unwind-tables" CACHE INTERNAL "C++ Compiler flags")