cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project( test)

set( CMAKE_COLOR_MAKEFILE TRUE)
set( CMAKE_VERBOSE_MAKEFILE FALSE)

#set( CMAKE_CXX_FLAGS "-Wno-deprecated -Wno-deprecated-declarations -Wno-error=unknown-pragmas -O3")
set( CMAKE_BUILD_TYPE "Debug")

set( LOCALBUILDS "$ENV{HOME}/local_builds")
set( LOCALLIBS "${LOCALBUILDS}/libs")

set( CMAKE_MODULE_PATH "${LOCALBUILDS}/CMakeModules")
set( CMAKE_LIBRARY_PATH "${LOCALLIBS}")

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

set( SRC_DIR "${PROJECT_SOURCE_DIR}")
set( SOURCES ${SRC_DIR}/main.cpp)

set( OpenCV_DIR "${LOCALLIBS}/opencv")
find_package( OpenCV 2.4 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package( RFeatures REQUIRED)
include_directories( ${RFeatures_INCLUDE_DIR})

find_package( RLIB REQUIRED)
include_directories( ${RLIB_INCLUDE_DIR})

add_executable( ${PROJECT_NAME} ${SOURCES})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries( ${PROJECT_NAME} ${RFeatures_LIBRARY})
target_link_libraries( ${PROJECT_NAME} ${RLIB_LIBRARY})


