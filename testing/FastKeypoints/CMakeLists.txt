cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project( test)

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

#set( CMAKE_CXX_FLAGS "-Wno-deprecated -Wno-deprecated-declarations -Wno-error=unknown-pragmas -O3")
set( CMAKE_BUILD_TYPE "Debug")

set( LOCALBUILDS "$ENV{HOME}/local_builds")
set( LOCALLIBS "${LOCALBUILDS}/libs")
set( CMAKE_MODULE_PATH "${LOCALBUILDS}/CMakeModules")
set( CMAKE_LIBRARY_PATH "${LOCALLIBS}")

set( CMAKE_COLOR_MAKEFILE TRUE)
set( CMAKE_VERBOSE_MAKEFILE FALSE)

set( SRC_DIR "${PROJECT_SOURCE_DIR}/src")
set( SOURCES ${SRC_DIR}/main.cpp)

set( BOOST_ROOT "${LOCALLIBS}/boost")
set( Boost_USE_STATIC_LIBS ON)
set( Boost_NO_SYSTEM_PATHS TRUE)
set( Boost_USE_STATIC_RUNTIME ON)
find_package( Boost 1.47 REQUIRED COMPONENTS filesystem regex chrono thread system serialization)
include_directories( "${Boost_INCLUDE_DIRS}")

set( OpenCV_DIR "${LOCALLIBS}/opencv")
find_package( OpenCV 2.4 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package( JPEG REQUIRED)
include_directories( ${JPEG_INCLUDE_DIR})

set( PCL_DIR "${LOCALLIBS}/PCL-1.6.0/share/pcl-1.6")
find_package( PCL 1.2 REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS})

find_package( RLIB REQUIRED)
include_directories( ${RLIB_INCLUDE_DIR})

find_package( RFeatures REQUIRED)
include_directories( ${RFeatures_INCLUDE_DIR})

add_executable( ${PROJECT_NAME} ${SOURCES})
target_link_libraries( ${PROJECT_NAME} ${RFeatures_LIBRARY})
target_link_libraries( ${PROJECT_NAME} ${RLIB_LIBRARY})
target_link_libraries( ${PROJECT_NAME} ${JPEG_LIBRARIES})
target_link_libraries( ${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS})
