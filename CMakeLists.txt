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

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories (include)

add_library (precision_tracker
  src/track_manager_color.cpp
  src/tracker.cpp
  src/precision_tracker.cpp
  src/motion_model.cpp
  src/adh_tracker3d.cpp
  src/down_sampler.cpp
  src/scored_transform.cpp
  src/density_grid_3d_evaluator.cpp
  src/density_grid_2d_evaluator.cpp
  src/lf_rgbd_6d_evaluator.cpp
  src/high_res_timer.cpp
  src/alignment_evaluator.cpp

  include/precision_tracking/track_manager_color.h
  include/precision_tracking/tracker.h
  include/precision_tracking/precision_tracker.h
  include/precision_tracking/motion_model.h
  include/precision_tracking/adh_tracker3d.h
  include/precision_tracking/down_sampler.h
  include/precision_tracking/scored_transform.h
  include/precision_tracking/density_grid_3d_evaluator.h
  include/precision_tracking/density_grid_2d_evaluator.h
  include/precision_tracking/lf_rgbd_6d_evaluator.h
  include/precision_tracking/high_res_timer.h
  include/precision_tracking/params.h
  include/precision_tracking/alignment_evaluator.h)

add_executable (test_tracking test_tracking.cpp)
target_link_libraries (test_tracking ${PCL_LIBRARIES} precision_tracker)
