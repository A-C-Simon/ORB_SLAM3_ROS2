# Try to find ORB_SLAM3
# Set alternative paths to search for using ORB_SLAM3_DIR
# Once done this will define
# You should ensure your ORB_SLAM3 can run correctly
#
# To help the search ORB_SLAM3_ROOT_DIR environment variable as the path to ORB_SLAM3 root folder
#  e.g. `set( ORB_SLAM3_ROOT_DIR=~/ORB_SLAM3) `
set(ORB_SLAM3_ROOT_DIR "" CACHE PATH "Path to the ORB_SLAM3 source/build tree")
if(NOT ORB_SLAM3_ROOT_DIR AND DEFINED ENV{ORB_SLAM3_ROOT_DIR})
  set(ORB_SLAM3_ROOT_DIR "$ENV{ORB_SLAM3_ROOT_DIR}")
endif()
if(NOT ORB_SLAM3_ROOT_DIR)
  get_filename_component(ORB_SLAM3_ROOT_DIR
    "${CMAKE_CURRENT_LIST_DIR}/../../ORB_SLAM3" ABSOLUTE)
endif()

# message(${ORB_SLAM3_ROOT_DIR})
# message(${ORB_SLAM3_ROOT_DIR}/include)
# message(${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/DBoW2)

# Find ORB_SLAM3
find_path(ORB_SLAM3_INCLUDE_DIR NAMES System.h
          PATHS ${ORB_SLAM3_ROOT_DIR}/include NO_DEFAULT_PATH)

find_library(ORB_SLAM3_LIBRARY NAMES ORB_SLAM3 libORB_SLAM3
             PATHS ${ORB_SLAM3_ROOT_DIR}/lib NO_DEFAULT_PATH)

# Find built-in DBoW2
find_path(DBoW2_INCLUDE_DIR NAMES Thirdparty/DBoW2/DBoW2/BowVector.h
          PATHS ${ORB_SLAM3_ROOT_DIR} NO_DEFAULT_PATH)

find_library(DBoW2_LIBRARY NAMES DBoW2
             PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib NO_DEFAULT_PATH)

# Find built-in g2o
find_library(g2o_LIBRARY NAMES g2o
             PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib NO_DEFAULT_PATH)



include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ORB_SLAM3_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(ORB_SLAM3  DEFAULT_MSG
                                  ORB_SLAM3_LIBRARY ORB_SLAM3_INCLUDE_DIR DBoW2_INCLUDE_DIR DBoW2_LIBRARY g2o_LIBRARY)

mark_as_advanced(ORB_SLAM3_INCLUDE_DIR ORB_SLAM3_LIBRARY )

set(ORB_SLAM3_LIBRARIES ${ORB_SLAM3_LIBRARY} ${DBoW2_LIBRARY} ${g2o_LIBRARY})
set(ORB_SLAM3_INCLUDE_DIRS ${ORB_SLAM3_INCLUDE_DIR} ${DBoW2_INCLUDE_DIR})
