cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set( CMAKE_BUILD_TYPE "Debug")

project(test)

set( SRC_FILES
    "${PROJECT_SOURCE_DIR}/src/test.cpp"
    "${PROJECT_SOURCE_DIR}/../../src/HOG.cpp"
    "${PROJECT_SOURCE_DIR}/../../src/FeatureUtils.cpp")

include_directories( "${PROJECT_SOURCE_DIR}/../../include")

set( OpenCV_DIR "$ENV{HOME}/local_builds/opencv")
find_package( OpenCV 2.4 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

add_executable( ${PROJECT_NAME} ${SRC_FILES})
target_link_libraries( "${PROJECT_NAME}" ${OpenCV_LIBS})
