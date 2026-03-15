cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME WHEEL)
project(${CMAKE_PROJECT_NAME})

# Enable compile command to ease indexing
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable ASM and C languages
enable_language(C ASM)

add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# 1. 添加所有源文件 (.c)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/main.c
    Hardware/Motor/motor.c
    Hardware/Sensors/sensors.c
    Control/pid.c
    Hardware/OLED/oled.c
    # 如果你有 OLED 文件夹，请务必取消下面这行的注释并确认路径正确
    Hardware/OLED/oled.c 
)

# 2. 添加所有头文件路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc
    Drivers/CMSIS/Device/ST/STM32F1xx/Include
    Drivers/CMSIS/Include
    Hardware/Motor 
    Hardware/Encoder
    Hardware/Sensors 
    Control
    # 如果有 OLED 文件夹，添加如下：
    Hardware/OLED
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    DEBUG
    USE_HAL_DRIVER
    STM32F103xB
)

# Remove wrong libob.a library dependency
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Link libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)