# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

#相对路径main.c
set(PROJECT_SRC_DIRS "." "./Src")
set(PROJECT_INC_DIRS "." "./Inc")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(battery_health_inference_RJ)

