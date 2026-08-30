# CMakeLists.txt – bno085 ESP-IDF component (I2C variant)
#
# Place this folder under your project's components/ directory:
#   components/bno085/
#     CMakeLists.txt
#     include/bno085.hpp
#     source/bno085.cpp
#     SH2/               ← Hillcrest SH-2 C library

cmake_minimum_required(VERSION 3.16)

set(reqs driver esp_timer freertos log)

if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.2")
    list(APPEND reqs esp_driver_gpio esp_driver_i2c)
endif()

idf_component_register(
    SRC_DIRS
        "source"
        "SH2"

    INCLUDE_DIRS
        "include"
        "SH2"

    REQUIRES
        ${reqs}
)
