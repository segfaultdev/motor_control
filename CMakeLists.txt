cmake_minimum_required(VERSION 3.22)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)
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
set(CMAKE_PROJECT_NAME motor_control)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable

# Add all FreeRTOS C sources (use a glob to collect them)
file(GLOB FREERTOS_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/*.c"
)

# Add FreeRTOS portable port sources (ARM Cortex-M4F)
file(GLOB FREERTOS_PORTABLE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/portable/GCC/ARM_CM4F/*.c"
)

# Add FreeRTOS memory management
file(GLOB FREERTOS_MEMMANG_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/portable/MemMang/*.c"
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE 
    ${FREERTOS_SOURCES} 
    ${FREERTOS_PORTABLE_SOURCES}
    ${FREERTOS_MEMMANG_SOURCES}
    ${CMAKE_SOURCE_DIR}/Core/Src/printk.c
    ${CMAKE_SOURCE_DIR}/Core/Src/ili9488.c
    ${CMAKE_SOURCE_DIR}/Core/Src/ui.c
    ${CMAKE_SOURCE_DIR}/Core/Src/btn.c
    ${CMAKE_SOURCE_DIR}/Core/Src/my_tasks.c


)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/include
    ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/portable/GCC/ARM_CM4F
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Generate hex and bin files from elf executable
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating hex and bin files..."
)

# Attempt to find the official STM32CubeProgrammer CLI
set(CUBE_PROGRAMMER "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe")

if(CUBE_PROGRAMMER)
    # If Cube Programmer is found, use it (Works on Win/Linux/Mac)
    add_custom_target(flash
        COMMAND ${CUBE_PROGRAMMER} -c port=SWD -w ${CMAKE_PROJECT_NAME}.bin 0x08000000 -v -rst
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMENT "Flashing with STM32CubeProgrammer..."
    )
else()
    # Fallback to st-flash (Open Source) if Cube Programmer isn't found
    if(CMAKE_HOST_WIN32)
        # Windows fallback (assuming st-flash is in PATH)
        add_custom_target(flash
            COMMAND st-flash --reset write ${CMAKE_PROJECT_NAME}.bin 0x08000000
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            DEPENDS ${CMAKE_PROJECT_NAME}
            COMMENT "Flashing with st-flash (Windows)..."
        )
    else()
        # Linux/Mac fallback
        add_custom_target(flash
            COMMAND st-flash --reset write ${CMAKE_PROJECT_NAME}.bin 0x08000000
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            DEPENDS ${CMAKE_PROJECT_NAME}
            COMMENT "Flashing with st-flash (Linux)..."
        )
    endif()
endif()