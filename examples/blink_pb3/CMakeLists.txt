cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Project
project(blink_pb3 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable
add_executable(${CMAKE_PROJECT_NAME}
  src/main.c
)

# SDK libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
  pico_stdlib
)

if (PICO_CYW43_SUPPORTED)
  target_link_libraries(${CMAKE_PROJECT_NAME}
    pico_cyw43_arch_none
  )
endif()

# create map/bin/hex file etc.
pico_add_extra_outputs(${CMAKE_PROJECT_NAME})

# Linker
set_property(TARGET ${CMAKE_PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS "-Wl,--print-memory-usage")

if(${PICO_PLATFORM} STREQUAL "rp2040")
  pico_set_linker_script(${CMAKE_PROJECT_NAME} ${CMAKE_SOURCE_DIR}/memmap_default_rp2040.ld)
elseif(${PICO_PLATFORM} MATCHES "rp2350")
  pico_set_linker_script(${CMAKE_PROJECT_NAME} ${CMAKE_SOURCE_DIR}/memmap_default_rp2350.ld)
endif()

