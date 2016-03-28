cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set( CMAKE_BUILD_TYPE "Debug")

project(test)
set( LOCALBUILDS "$ENV{HOME}/local_builds")

set( SRC_FILES
    "${PROJECT_SOURCE_DIR}/src/test.cpp"
    "${PROJECT_SOURCE_DIR}/../../src/RegionSorter2.cpp")

include_directories( "${PROJECT_SOURCE_DIR}/../../include")

set( OpenCV_DIR "${LOCALBUILDS}/libs/opencv")
find_package( OpenCV 2.4 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

add_executable( ${PROJECT_NAME} ${SRC_FILES})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS})
