cmake_minimum_required(VERSION 3.22)

# Project name
project(Ctrboard-H7-bottom-core C ASM)

# Toolchain for STM32
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# Embedded toolchain (must be set before any try_compile)
set(TOOLCHAIN_PREFIX "/opt/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi")
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}-ar)
set(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}-ranlib)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}-size)

# C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS "-T ${CMAKE_SOURCE_DIR}/STM32H723XG_FLASH.ld -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -Wl,--gc-sections -Wl,-Map=Ctrboard-H7-bottom-core.map -Wl,--print-memory-usage -specs=nosys.specs -specs=nano.specs")

# Core configuration
set(CORE_MCU "STM32H723RBTx")
set(CORE_FREQ 550000000)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Create executable target before adding subdirectories
add_executable(Ctrboard-H7-bottom-core)

# Include stm32cubemx configuration (must come after target creation)
add_subdirectory(cmake/stm32cubemx)

# Generate linker symbols
set_source_files_properties(
    ${CMAKE_SOURCE_DIR}/startup_stm32h723xx.s
    PROPERTIES LANGUAGE ASM
)

# Additional clean files
set_target_properties(Ctrboard-H7-bottom-core PROPERTIES
    ADDITIONAL_CLEAN_FILES "Ctrboard-H7-bottom-core.elf;Ctrboard-H7-bottom-core.bin;Ctrboard-H7-bottom-core.hex"
)

# Generate flashable images after linking
add_custom_command(TARGET Ctrboard-H7-bottom-core POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
        $<TARGET_FILE:Ctrboard-H7-bottom-core>
        $<TARGET_FILE_DIR:Ctrboard-H7-bottom-core>/Ctrboard-H7-bottom-core.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
        $<TARGET_FILE:Ctrboard-H7-bottom-core>
        $<TARGET_FILE_DIR:Ctrboard-H7-bottom-core>/Ctrboard-H7-bottom-core.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:Ctrboard-H7-bottom-core>
    COMMENT "Generating HEX/BIN images"
)