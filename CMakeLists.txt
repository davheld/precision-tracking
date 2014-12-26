cmake_minimum_required(VERSION 2.8)
project(precision_tracking)

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(Boost REQUIRED thread signals)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree io)

include_directories (include ${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_library (${PROJECT_NAME}
  src/adh_tracker3d.cpp
  src/alignment_evaluator.cpp
  src/density_grid_2d_evaluator.cpp
  src/density_grid_3d_evaluator.cpp
  src/down_sampler.cpp
  src/high_res_timer.cpp
  src/lf_rgbd_6d_evaluator.cpp
  src/motion_model.cpp
  src/precision_tracker.cpp
  src/scored_transform.cpp
  src/sensor_specs.cpp
  src/track_manager_color.cpp
  src/tracker.cpp

  include/precision_tracking/adh_tracker3d.h
  include/precision_tracking/alignment_evaluator.h
  include/precision_tracking/density_grid_2d_evaluator.h
  include/precision_tracking/density_grid_3d_evaluator.h
  include/precision_tracking/down_sampler.h
  include/precision_tracking/high_res_timer.h
  include/precision_tracking/lf_rgbd_6d_evaluator.h
  include/precision_tracking/motion_model.h
  include/precision_tracking/params.h
  include/precision_tracking/precision_tracker.h
  include/precision_tracking/scored_transform.h
  include/precision_tracking/sensor_specs.h
  include/precision_tracking/track_manager_color.h
  include/precision_tracking/tracker.h
)

add_executable (test_tracking test_tracking.cpp)
target_link_libraries(${PROJECT_NAME} ${EIGEN_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries (test_tracking ${PROJECT_NAME})

