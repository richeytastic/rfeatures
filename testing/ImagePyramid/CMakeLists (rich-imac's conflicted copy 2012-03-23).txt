cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set( CMAKE_BUILD_TYPE "Debug")

set( CMAKE_MODULE_PATH "$ENV{HOME}/dev/CMakeModules")
set( CMAKE_INCLUDE_PATH "$ENV{HOME}/dev/include")
set( CMAKE_LIBRARY_PATH "$ENV{HOME}/lib")
set( CMAKE_COLOR_MAKEFILE TRUE)
set( CMAKE_VERBOSE_MAKEFILE FALSE)

project(test)

set( SRC_DIR "src")
set( SRC_FILES "${SRC_DIR}/test.cpp")

find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package( RFeatures REQUIRED)
include_directories( ${RFeatures_INCLUDE_DIR})

add_executable( ${PROJECT_NAME} ${SRC_FILES})
target_link_libraries( "${PROJECT_NAME}" ${OpenCV_LIBS})
target_link_libraries( "${PROJECT_NAME}" ${RFeatures_LIBRARY})
