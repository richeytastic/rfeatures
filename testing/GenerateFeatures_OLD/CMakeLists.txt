cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set( CMAKE_BUILD_TYPE "Release")

set( LOCALBUILDS "$ENV{HOME}/local_builds")
set( LOCALLIBS "${LOCALBUILDS}/libs")
set( CMAKE_MODULE_PATH "${LOCALBUILDS}/CMakeModules")
set( CMAKE_LIBRARY_PATH "${LOCALLIBS}")
set( CMAKE_COLOR_MAKEFILE TRUE)
set( CMAKE_VERBOSE_MAKEFILE FALSE)

project(genfeatures)

set( SRC_FILES
    "${PROJECT_SOURCE_DIR}/src/main.cpp")

find_package( Boost 1.4 REQUIRED system)
include_directories( ${Boost_INCLUDE_DIRS})

set( OpenCV_DIR "${LOCALLIBS}/opencv")
find_package( OpenCV 2.4 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package( RFeatures REQUIRED)
include_directories( ${RFeatures_INCLUDE_DIR})

add_executable( ${PROJECT_NAME} ${SRC_FILES})
target_link_libraries( ${PROJECT_NAME} ${RFeatures_LIBRARY})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries( ${PROJECT_NAME} ${Boost_LIBRARIES})
