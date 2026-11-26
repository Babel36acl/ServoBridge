# ServoBridge 固件根构建文件
cmake_minimum_required(VERSION 3.16)

# 版本号：CI 发版时用环境变量 SERVOBRIDGE_VERSION=<tag 去掉 v> 覆盖；
# 不设则走 IDF 默认机制（git describe，无 tag 时为短哈希）。
# 必须写在 project() 之前，IDF 只在 PROJECT_VER 未定义时才自己推断。
if(NOT "$ENV{SERVOBRIDGE_VERSION}" STREQUAL "")
    set(PROJECT_VER "$ENV{SERVOBRIDGE_VERSION}")
    message(STATUS "PROJECT_VER 由 SERVOBRIDGE_VERSION 指定：${PROJECT_VER}")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ServoBridge)
