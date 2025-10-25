cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 23)

set(OUTPUT_DIR "${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}")

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(RP8086 C CXX ASM)

pico_sdk_init()

add_executable(${PROJECT_NAME}
        main.c
        cpu.c
        cpu_bus.c
)

pico_define_boot_stage2(slower_boot ${PICO_DEFAULT_BOOT_STAGE2_FILE})
target_compile_definitions(slower_boot PRIVATE PICO_FLASH_SPI_CLKDIV=4)
pico_set_boot_stage2(${PROJECT_NAME} slower_boot)

# 16megs of flash on purple pico clones
target_compile_definitions(${PROJECT_NAME} PRIVATE
        PICO_FLASH_SIZE_BYTES=16777216
        #        PICO_PRINTF_ALWAYS_INCLUDED=0
        #        PICO_STACK_SIZE=1024
        #        PICO_HEAP_SIZE=1024
        #PICO_PANIC_FUNCTION=
)


pico_generate_pio_header(${PROJECT_NAME}
        ${CMAKE_CURRENT_LIST_DIR}/i8086_bus.pio
        OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/
)

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
        pico_runtime
        pico_stdio
        pico_multicore
        hardware_pio
        hardware_pwm
)

pico_add_extra_outputs(${PROJECT_NAME})
target_link_options(${PROJECT_NAME} PRIVATE -Xlinker --print-memory-usage --data-sections --function-sections)