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

find_package( PCL 1.2 REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS})
link_directories( ${PCL_LIBRARY_DIRS})
add_definitions( ${PCL_DEFINITIONS})

find_package( Boost 1.4 REQUIRED COMPONENTS filesystem regex system serialization)
include_directories( ${Boost_INCLUDE_DIRS})

find_package( RLIB REQUIRED)
include_directories( ${RLIB_INCLUDE_DIR})

add_executable( ${PROJECT_NAME} ${SRC_FILES})
target_link_libraries( "${PROJECT_NAME}" ${PCL_LIBRARIES})
target_link_libraries( "${PROJECT_NAME}" ${Boost_LIBRARIES})
target_link_libraries( "${PROJECT_NAME}" ${RLIB_LIBRARY})

