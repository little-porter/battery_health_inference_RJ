# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(PROJECT_SRC_DIRS "." "./Src" "./model/modelData" "./model/Soc" "./model/Soh" "./model/Rsk" "./alarm" "./ModelVerify" "./Config" "./tflm")
set(PROJECT_INC_DIRS "." "./Inc" "./model/Soc" "./model/Soh" "./model/Rsk" "./alarm" "./ModelVerify" "./Config" "./tflm")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(PROJECT_VER "2.1.5")
project(battery_inference_ota)

