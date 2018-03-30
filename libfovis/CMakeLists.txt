cmake_minimum_required(VERSION 2.8.3)
project(libfovis)

find_package(catkin REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS libfovis
  LIBRARIES libfovis
)

add_definitions(-Wall -O3 -march=native)
#add_definitions( -DUSE_FAST_SETTINGS )
include_directories(libfovis
  ${catkin_INCLUDE_DIRS}
)

add_library(libfovis
    libfovis/frame.cpp
    libfovis/visual_odometry.cpp
    libfovis/fast.cpp
    libfovis/motion_estimation.cpp
    libfovis/gauss_pyramid.c
    libfovis/refine_motion_estimate.cpp
    libfovis/tictoc.cpp
    libfovis/primesense_depth.cpp
    libfovis/initial_homography_estimation.cpp
    libfovis/grid_filter.cpp
    libfovis/intensity_descriptor.cpp
    libfovis/pyramid_level.cpp
    libfovis/feature_matcher.cpp
    libfovis/refine_feature_match.cpp
    libfovis/stereo_depth.cpp
    libfovis/stereo_frame.cpp
    libfovis/depth_image.cpp
    libfovis/rectification.cpp
    libfovis/stereo_rectify.cpp
    libfovis/internal_utils.cpp
    libfovis/normalize_image.cpp)

set_target_properties(libfovis PROPERTIES SOVERSION 1)
