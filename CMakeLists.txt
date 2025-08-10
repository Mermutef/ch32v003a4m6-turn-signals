cmake_minimum_required(VERSION 3.16)
project(belka C CXX ASM)

## Настройка toolchain
set(TOOLCHAIN_PATH "/home/mermutef/riscv-gcc12")
set(CMAKE_C_COMPILER "${TOOLCHAIN_PATH}/bin/riscv-wch-elf-gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PATH}/bin/riscv-wch-elf-g++")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PATH}/bin/riscv-wch-elf-gcc")
set(CMAKE_OBJCOPY "${TOOLCHAIN_PATH}/bin/riscv-wch-elf-objcopy")
set(CMAKE_SIZE "${TOOLCHAIN_PATH}/bin/riscv-wch-elf-size")

# Устанавливаем переменную для поиска cc1
set(ENV{GCC_EXEC_PREFIX} "${TOOLCHAIN_PATH}/libexec/gcc/")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# Правильные флаги для CH32V00x (rv32ec архитектура)
add_compile_options(-march=rv32ec -mabi=ilp32e -msmall-data-limit=8 -msave-restore -fsigned-char)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
add_compile_options(-Wunused -Wuninitialized)
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(Core Debug Peripheral/inc User)

# Правильный startup файл для CH32V00x
file(GLOB_RECURSE SOURCES "Core/*.c" "Debug/*.c" "Peripheral/src/*.c" "User/*.c" "Startup/startup_ch32v00x.S")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Ld/Link.ld)

# Путь к библиотекам для rv32ec/ilp32e
set(TOOLCHAIN_LIB_PATH "${TOOLCHAIN_PATH}/riscv-wch-elf/lib/rv32ec/ilp32e")
set(GCC_LIB_PATH "${TOOLCHAIN_PATH}/lib/gcc/riscv-wch-elf/12.2.0/rv32ec/ilp32e")

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-nostartfiles --specs=nano.specs --specs=nosys.specs)
add_link_options(-T ${LINKER_SCRIPT})
add_link_options(-L${TOOLCHAIN_LIB_PATH})
add_link_options(-L${GCC_LIB_PATH})

# uncomment to use printf float
#add_link_options(-u _printf_float)

add_executable(${PROJECT_NAME}.elf
        ${SOURCES}
        ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")