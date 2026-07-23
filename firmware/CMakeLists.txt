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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME 1132_bot)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and CXX languages
enable_language(C CXX ASM)

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
    ${CMAKE_SOURCE_DIR}/Core/Src/robot_tasks.cpp
    ${CMAKE_SOURCE_DIR}/Core/Src/tca9548a.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/src
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# xy_robotkit module sources / includes
include(${CMAKE_SOURCE_DIR}/xy_robot_modules.cmake)

# Add each module root directory to include path so headers like
# `BMI088Sensor.hpp` (which sits at Modules/<Mod>/) and patterns like
# `#include "include/BMI088driver.h"` (relative to the module root) resolve.
file(GLOB XY_ROBOT_MODULE_ROOTS LIST_DIRECTORIES true "${CMAKE_SOURCE_DIR}/Modules/*")
foreach(mod_root ${XY_ROBOT_MODULE_ROOTS})
    if(IS_DIRECTORY "${mod_root}")
        list(APPEND XY_ROBOT_MODULE_INCLUDES "${mod_root}")
    endif()
endforeach()
if(XY_ROBOT_MODULE_INCLUDES)
    list(REMOVE_DUPLICATES XY_ROBOT_MODULE_INCLUDES)
endif()

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${XY_ROBOT_MODULE_SOURCES})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${XY_ROBOT_MODULE_INCLUDES})

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
list(REMOVE_ITEM CMAKE_CXX_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# DAP-Link 烧录/调试目标
set(FLASH_OPENOCD_CFG ${CMAKE_SOURCE_DIR}/openocd.cfg CACHE FILEPATH "OpenOCD DAPLink 配置文件")

if(WIN32)
    set(FLASH_TOOL_SCRIPT ${CMAKE_SOURCE_DIR}/tools/flash-daplink.ps1)
    set(FLASH_INTERFACE_CFG interface/cmsis-dap.cfg CACHE STRING "OpenOCD 接口配置")
    set(FLASH_TARGET_CFG target/stm32f4x.cfg CACHE STRING "OpenOCD 目标配置")

    add_custom_target(flash
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMAND powershell.exe
                -NoProfile
                -ExecutionPolicy Bypass
                -File ${FLASH_TOOL_SCRIPT}
                -ElfFile $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
                -InterfaceCfg ${FLASH_INTERFACE_CFG}
                -TargetCfg ${FLASH_TARGET_CFG}
        COMMENT "使用 DAP-Link 烧录 ${CMAKE_PROJECT_NAME}"
        USES_TERMINAL
        VERBATIM
    )
else()
    set(FLASH_OPENOCD_DEFAULT "$ENV{HOME}/.local/xpack-openocd/bin/openocd")
    if(EXISTS "${FLASH_OPENOCD_DEFAULT}")
        set(FLASH_OPENOCD_INIT "${FLASH_OPENOCD_DEFAULT}")
    else()
        find_program(FLASH_OPENOCD_INIT
            NAMES openocd
            DOC "OpenOCD 可执行文件"
        )
        if(NOT FLASH_OPENOCD_INIT)
            set(FLASH_OPENOCD_INIT openocd)
        endif()
    endif()
    set(FLASH_OPENOCD ${FLASH_OPENOCD_INIT} CACHE FILEPATH "OpenOCD 可执行文件")

    add_custom_target(flash
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMAND ${FLASH_OPENOCD}
                -f ${FLASH_OPENOCD_CFG}
                -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify reset exit"
        COMMENT "使用 DAP-Link 烧录 ${CMAKE_PROJECT_NAME}"
        USES_TERMINAL
        VERBATIM
    )

    # OpenOCD GDB Server 调试目标
    # 用法: cmake --build build/Debug --target debug-server
    # 然后在另一个终端: gdb-multiarch build/Debug/1132_bot.elf -ex "target extended-remote :3333"
    add_custom_target(debug-server
        COMMAND ${FLASH_OPENOCD}
                -f ${FLASH_OPENOCD_CFG}
        COMMENT "启动 OpenOCD GDB Server (端口 3333)，在另一个终端用 gdb-multiarch 连接"
        USES_TERMINAL
        VERBATIM
    )
endif()
