cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set( CMAKE_BUILD_TYPE "Release")

project(dumppano)
set( LOCALLIBS "$ENV{HOME}/local_builds/libs")

set( SRC_FILES
    "${PROJECT_SOURCE_DIR}/src/test.cpp"
    "${PROJECT_SOURCE_DIR}/../../src/View.cpp"
    "${PROJECT_SOURCE_DIR}/../../src/Panorama.cpp")

include_directories( "${PROJECT_SOURCE_DIR}/../../include")

#set( OpenCV_DIR "${LOCALLIBS}/opencv")
find_package( OpenCV 2.4 REQUIRED core highgui)
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package( Boost 1.4 REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS})

add_executable( ${PROJECT_NAME} ${SRC_FILES})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS})
