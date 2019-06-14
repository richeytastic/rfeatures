# =============================================================================
# The rFeatures CMake configuration file.
#
#           ** File generated automatically, DO NOT MODIFY! ***

# To use from an external project, in your project's CMakeLists.txt add:
#   FIND_PACKAGE( rFeatures REQUIRED)
#   INCLUDE_DIRECTORIES( rFeatures ${rFeatures_INCLUDE_DIRS})
#   LINK_DIRECTORIES( ${rFeatures_LIBRARY_DIR})
#   TARGET_LINK_LIBRARIES( MY_TARGET_NAME ${rFeatures_LIBRARIES})
#
# This module defines the following variables:
#   - rFeatures_FOUND         : True if rFeatures is found.
#   - rFeatures_ROOT_DIR      : The root directory where rFeatures is installed.
#   - rFeatures_INCLUDE_DIRS  : The rFeatures include directories.
#   - rFeatures_LIBRARY_DIR   : The rFeatures library directory.
#   - rFeatures_LIBRARIES     : The rFeatures imported libraries to link to.
#
# =============================================================================

get_filename_component( rFeatures_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component( rFeatures_ROOT_DIR  "${rFeatures_CMAKE_DIR}"           PATH)

set( rFeatures_INCLUDE_DIRS "${rFeatures_ROOT_DIR}/../include" CACHE PATH "The rFeatures include directories.")
set( rFeatures_LIBRARY_DIR  "${rFeatures_ROOT_DIR}"            CACHE PATH "The rFeatures library directory.")

include( "${CMAKE_CURRENT_LIST_DIR}/Macros.cmake")
get_library_suffix( _lsuff)
set( _hints rFeatures${_lsuff} librFeatures${_lsuff})
find_library( rFeatures_LIBRARIES NAMES ${_hints} PATHS "${rFeatures_LIBRARY_DIR}/static" "${rFeatures_LIBRARY_DIR}")
set( rFeatures_LIBRARIES     ${rFeatures_LIBRARIES}         CACHE FILE "The rFeatures imported libraries to link to.")

# handle QUIETLY and REQUIRED args and set rFeatures_FOUND to TRUE if all listed variables are TRUE
include( "${CMAKE_ROOT}/Modules/FindPackageHandleStandardArgs.cmake")
find_package_handle_standard_args( rFeatures rFeatures_FOUND rFeatures_LIBRARIES rFeatures_INCLUDE_DIRS)
