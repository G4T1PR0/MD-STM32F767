cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME STM_MD)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user sources here
    Core/App/app_main.cpp
    Core/App/Devices/McuAbstractionLayer/stm32halAbstractionLayer.cpp
    Core/App/Devices/Driver/A3921.cpp
    Core/App/Devices/Driver/batteryVoltageSensor.cpp
    Core/App/Devices/Driver/currentSensor.cpp
    Core/App/Devices/Driver/encoder.cpp
    Core/App/Devices/Driver/steerAngleSensor.cpp
    Core/App/Algo/CommandReciever.cpp
    Core/App/Algo/MotorController.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user defined include paths
    Core/App
    Core/App/McuAbstractionLayer
    Core/App/Device/Driver
    Core/App/Device/Driver/Interface
    Core/App/Algorithm
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
