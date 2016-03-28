cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set( CMAKE_BUILD_TYPE "Debug")

project(test)

set( SRC_FILES
    "${PROJECT_SOURCE_DIR}/src/main.cpp"
    "${PROJECT_SOURCE_DIR}/src/Superpixels.cpp"
    "${PROJECT_SOURCE_DIR}/src/SLIC.cpp")

include_directories( "${PROJECT_SOURCE_DIR}/include")

set( MY_LIBS "$ENV{HOME}/local_builds/libs")

set( OpenCV_DIR "${MY_LIBS}/opencv")
find_package( OpenCV 2.4 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

set( BOOST_ROOT "${MY_LIBS}/boost")
find_package( Boost 1.4 REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS})

add_executable( ${PROJECT_NAME} ${SRC_FILES})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS})
