CONTENTS
--------
   
 * Introduction
 * Requirements
 * Installation
 * Configuration
 * Maintainers
 * License

INTRODUCTION
------------
Our tracker takes as input 2 segmented point clouds and combines 3D shape, color (when available), and motion cues to accurately estimate the velocity of each object in real-time.  For more information, see our project page:
http://stanford.edu/~davheld/anytime_tracking.html

REQUIREMENTS
------------
To run the tracker, make sure you have installed PCL and CMake.

To install PCL, see:
http://pointclouds.org/downloads/

To install CMake on Ubuntu, type:
sudo apt-get install cmake

To install CMake on other platforms, see:
http://www.cmake.org/cmake/resources/software.html

INSTALLATION
------------

Install the tracker repository:

git clone ... .

mkdir build
cd build
cmake ..
make

Run the tracker:

./test_tracking ../testColor.tm ../gtFolder

CONFIGURATION
-------------


USAGE
-----
4) Using the tracker in your own project

At the top of your file, add:

#include "tracker.h"

When you want to get a velocity estimate, add this code:

  Tracker precision_tracker;
  Eigen::Vector3f estimated_velocity;
  tracker->addPoints(points, timestamp, sensor_horizontal_resolution,
                     sensor_vertical_resolution, &estimated_velocity);

where:
points = the currently observed points for the object that you wish to track
timestamp = the time that these points were observed
sensor_horizontal_resolution = the horizontal resolution of the sensor
  for an object at distance observed
sensor_vertical_resolution = the vertical resolution of the sensor
  for an object at distance observed
estimated_velocity = the estimated translational velocity.  The tracker
  only outputs the estimated horizontal velocity - vertical motion and
  rotation are not currently estimated.  The velocity estimate is 0 for the
  first frame in which an object is observed and is the estimated velocity
  thereafter.

The horizontal and vertical resolution depend on the sensor that is used.
For the 64-beam Velodyne spinning at 10 Hz, given the horizontal distance from
the sensor to the tracked object, the sensor resolution can be computed as
follows (taken from test_tracking.cpp):

  // The horizontal resolution for the 64-beam Velodyne spinning at 10 Hz
  // is 0.18 degrees.
  const double velodyne_horizontal_angular_res = 0.18;

  // There are 64 beams spanning 26.8 vertical degrees, so the average spacing
  // between each beam is computed as follows.
  const double velodyne_vertical_angular_res = 26.8 / 63;

  // We convert the angular resolution to meters for a given range.
  const double velodyne_horizontal_res =
      2 * horizontal_distance *
      tan(velodyne_horizontal_angular_res / 2.0 * pi / 180.0);
  const double velodyne_vertical_res =
      2 * horizontal_distance *
      tan(velodyne_vertical_angular_res / 2.0 * pi / 180.0);

MAINTAINERS
-----------


LICENSE
-------

