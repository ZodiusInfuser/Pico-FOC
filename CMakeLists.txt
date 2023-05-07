cmake_minimum_required(VERSION 3.12)

#include(pimoroni_pico_import.cmake)
include(pico_sdk_import.cmake)

# Gooey boilerplate
project(pico-foc C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_subdirectory(examples)
add_subdirectory(src)