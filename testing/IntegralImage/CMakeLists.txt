cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set( CMAKE_BUILD_TYPE "Debug")

set( CMAKE_MODULE_PATH "$ENV{HOME}/dev/CMakeModules")
set( CMAKE_INCLUDE_PATH "$ENV{HOME}/dev/include")
set( CMAKE_LIBRARY_PATH "$ENV{HOME}/lib")
set( CMAKE_COLOR_MAKEFILE TRUE)
set( CMAKE_VERBOSE_MAKEFILE FALSE)

project(test)
set( LOCALBUILDS "$ENV{HOME}/local_builds")

set( SRC_FILES
    "${PROJECT_SOURCE_DIR}/main.cpp"
    "${PROJECT_SOURCE_DIR}/../../src/FeatureUtils.cpp")


include_directories( "${PROJECT_SOURCE_DIR}/../../include")

set( OpenCV_DIR "${LOCALBUILDS}/libs/opencv")
find_package( OpenCV 2.4 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

set( BOOST_ROOT "${LOCALBUILDS}/libs/boost")
set( Boost_USE_STATIC_LIBS ON)
set( Boost_USE_MULTITHREADED ON)
set( Boost_USE_STATIC_RUNTIME ON)
find_package( Boost 1.4 REQUIRED COMPONENTS filesystem regex system serialization thread)
include_directories( ${Boost_INCLUDE_DIRS})

add_executable( ${PROJECT_NAME} ${SRC_FILES})
target_link_libraries( "${PROJECT_NAME}" ${OpenCV_LIBS})
