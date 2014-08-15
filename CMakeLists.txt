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
add_executable (tracker tracker.cpp track_manager_color.cpp model_builder.cpp
  fast_functions.cpp precision_tracker.cpp model_builder.cpp motion_model.cpp
  transform_helper.cpp adh_tracker3d.cpp down_sampler.cpp lf_discrete_3d.cpp
  scored_transform.cpp density_grid_tracker.cpp lf_rgbd_6d.cpp
  high_res_timer.cpp)
target_link_libraries (tracker ${PCL_LIBRARIES})
