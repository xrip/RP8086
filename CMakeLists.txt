cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(OUTPUT_DIR "${CMAKE_SOURCE_DIR}/bin/")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(PICO_FLASH_SIZE_BYTES 16777216)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(RP8086 C CXX ASM)

pico_sdk_init()
add_compile_options(
#                -flto=auto
#                -fwhole-program
        #        -ffunction-sections
        #        -fdata-sections
        -Ofast
)


add_link_options(-flto -fwhole-program)

add_executable(${PROJECT_NAME}
        src/main.c
        src/cpu.c
        src/cpu_bus.c
        src/common.c
)

add_subdirectory(drivers/graphics)
add_subdirectory(drivers/vga-nextgen)
target_link_libraries(${PROJECT_NAME} PRIVATE vga-nextgen)
message(STATUS "VGA display enabled")

target_compile_definitions(${PROJECT_NAME} PRIVATE
        VGA
        VGA_BASE_PIN=30
        PICO_FLASH_SIZE_BYTES=16777216
        PICO_PRINTF_ALWAYS_INCLUDED=0
        PICO_PANIC_FUNCTION=0
)


pico_generate_pio_header(${PROJECT_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/i8086_bus.pio
        OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/
)

if (USE_USB_CDC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG=1)
    pico_enable_stdio_uart(${PROJECT_NAME} 0)
    pico_enable_stdio_usb(${PROJECT_NAME} 1)
else ()
    set(FAMILY rp2350)
    set(BOARD pico_sdk)
    add_subdirectory(drivers/usbhid)
    target_include_directories(${PROJECT_NAME} PRIVATE ./host/)
    target_link_libraries(${PROJECT_NAME} PRIVATE
            usbhid
            tinyusb_host
            tinyusb_board
    )
endif ()


target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/rom
)

target_link_libraries(${PROJECT_NAME} PRIVATE
        graphics

        pico_runtime
        pico_stdlib
        pico_stdio
        pico_multicore
        hardware_pio
        hardware_pwm
)

pico_set_binary_type(${PROJECT_NAME} copy_to_ram)

pico_add_extra_outputs(${PROJECT_NAME})

# Post-build step: increment build number and copy firmware files with incremental suffix
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND}
        -DOUTPUT_DIR=${OUTPUT_DIR}
        -DPROJECT_NAME=${PROJECT_NAME}
        -DBUILD_NUMBER_FILE=${CMAKE_SOURCE_DIR}/.build_number
        -P ${CMAKE_SOURCE_DIR}/rename_with_build_number.cmake
        COMMENT "Appending incremental build number to firmware filenames"
)
execute_process(
        COMMAND git rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
        COMMAND git rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
        PICO_BUILD_NAME="${BUILD_NAME}"
        PICO_GIT_BRANCH="${GIT_BRANCH}"
        PICO_GIT_COMMIT="${GIT_COMMIT_HASH}"
)

pico_set_program_name(${PROJECT_NAME} "${PROJECT_NAME}")
pico_set_program_version(${PROJECT_NAME} "${GIT_BRANCH} ${GIT_COMMIT_HASH}")
pico_set_linker_script(${PROJECT_NAME} "${CMAKE_SOURCE_DIR}/memmap.ld")
target_link_options(${PROJECT_NAME} PRIVATE
        -Xlinker
        --print-memory-usage
        --data-sections
        --function-sections
        -Wl,--wrap=atexit,--wrap=abort,--strip-all,--gc-sections # size optimizations
)