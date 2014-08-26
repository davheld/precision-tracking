1) Prerequisites:

1A) Install PCL:
http://pointclouds.org/downloads/

1B) Install CMake:

In Ubuntu, type:
sudo apt-get install cmake

Other platforms:
Install CMake from the binary appropriate for your OS:
http://www.cmake.org/cmake/resources/software.html

2) Install the tracker repository:

git clone ... .

3) Run the tracker:

mkdir build
cd build
cmake ..
make
./test_tracking ../testColor.tm ../gtFolder

4) Using the tracker in your own project

At the top of your file, add:

#include "tracker.h"

When you want to get a velocity estimate, add this code:

Tracker precision_tracker;
Eigen::Vector3f estimated_velocity;
tracker->addPoints(points, timestamp, centroid, &estimated_velocity);
