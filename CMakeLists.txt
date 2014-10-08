cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(segmentation)

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
  endif()

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories (include)

add_library (precision_tracker
  src/track_manager_color.cpp
  src/tracker.cpp
  src/precision_tracker.cpp
  src/motion_model.cpp
  src/adh_tracker3d.cpp
  src/down_sampler.cpp
  src/scored_transform.cpp
  src/density_grid_evaluator.cpp
  src/lf_rgbd_6d_evaluator.cpp
  src/high_res_timer.cpp
  src/alignment_evaluator.cpp

  include/track_manager_color.h
  include/tracker.h
  include/precision_tracker.h
  include/motion_model.h
  include/adh_tracker3d.h
  include/down_sampler.h
  include/scored_transform.h
  include/density_grid_evaluator.h
  include/lf_rgbd_6d_evaluator.h
  include/high_res_timer.h
  include/alignment_evaluator.h)

add_executable (test_tracking test_tracking.cpp)
target_link_libraries (test_tracking ${PCL_LIBRARIES} precision_tracker)
