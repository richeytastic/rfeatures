cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set( CMAKE_BUILD_TYPE "Release")

set( LOCALBUILDS "$ENV{HOME}/local_builds")
set( LOCALLIBS "${LOCALBUILDS}/libs")
set( CMAKE_MODULE_PATH "${LOCALBUILDS}/CMakeModules")
set( CMAKE_LIBRARY_PATH "${LOCALLIBS}")

project(test)
set( LOCAL_LIBS "$ENV{HOME}/local_builds/libs")

set( SRC_FILES
    "${PROJECT_SOURCE_DIR}/src/test.cpp"
    "${PROJECT_SOURCE_DIR}/../../src/ImageGradientsBuilder.cpp"
    "${PROJECT_SOURCE_DIR}/../../src/RangeBuilder.cpp"
    "${PROJECT_SOURCE_DIR}/../../src/RangeGradientsBuilder.cpp"
    "${PROJECT_SOURCE_DIR}/../../src/ProHOG.cpp"
    "${PROJECT_SOURCE_DIR}/../../src/FeatureOperator.cpp"
    "${PROJECT_SOURCE_DIR}/../../src/DepthSegmenter.cpp"
    "${PROJECT_SOURCE_DIR}/../../src/FeatureUtils.cpp")

include_directories( "${PROJECT_SOURCE_DIR}/../../include")

#set( OpenCV_DIR "${LOCAL_LIBS}/opencv")
find_package( OpenCV 2.4 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

set( BOOST_ROOT "${LOCAL_LIBS}/boost")
set( Boost_USE_STATIC_LIBS ON)
set( Boost_USE_MULTITHREADED ON)
set( Boost_USE_STATIC_RUNTIME ON)
find_package( Boost 1.4 REQUIRED COMPONENTS filesystem regex system serialization thread)
include_directories( ${Boost_INCLUDE_DIRS})

find_package( RLIB REQUIRED)
include_directories( "${RLIB_INCLUDE_DIR}")

add_executable( ${PROJECT_NAME} ${SRC_FILES})
target_link_libraries( ${PROJECT_NAME} ${RLIB_LIBRARY})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS})
#target_link_libraries( ${PROJECT_NAME} ${RFeatures_LIBRARY})
