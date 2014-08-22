cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
  endif()

project(tracker)

set (PCL_DIR /home/davheld/code/pcl-trunk/release/share/pcl-1.7)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_library(track_manager_color track_manager_color.h track_manager_color.cpp)

include_directories ("${PROJECT_SOURCE_DIR}")

add_executable (test_tracking test_tracking.cpp track_manager_color.cpp
  tracker.cpp precision_tracker.cpp motion_model.cpp
  adh_tracker3d.cpp down_sampler.cpp scored_transform.cpp
  density_grid_evaluator.cpp lf_rgbd_6d_evaluator.cpp high_res_timer.cpp
  alignment_evaluator.cpp)
target_link_libraries (test_tracking ${PCL_LIBRARIES})
