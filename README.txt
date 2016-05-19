CONTENTS
--------
   
 * Introduction
 * Requirements
 * Installation
 * Configuration
 * Usage
 * Maintainers
 * License
 * Citations

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
First get the tracker repository from github:  

git clone https://github.com/davheld/precision-tracking.git

To build the tracker, make sure that you are in the git repository for the tracker, and then run:

mkdir build
cd build
cmake ..
make

To test the tracker is working, you first need to download some test data.  You can do this by running:
cd ..
wget http://cs.stanford.edu/people/davheld/public/test.tm

Now to test that the tracker is working, run:

cd build
./test_tracking ../test.tm ../gtFolder

This will execute a test script which will run 5 different versions of the tracker on the test data.  Each version has a different speed / accuracy tradeoff, as explained in the print statements that will appear on your screen.

If you are using ROS, then you can use CMakeLists.txt.ros (just rename this as CMakeLists.txt) and package.xml to compile the tracker.

CONFIGURATION
-------------
The tracker is currently configured assuming that you are recording data with the HDL-64E Velodyne Lidar.  If you are recording data with some other 3D sensor, if you are using a pair of stereo cameras, or if you are using some other method to obtain 3D point clouds, then you will need to adjust some configuration parameters accordingly.

First, in params.h, you will need to change the parameter kMinMeasurementVariance to be equal to the average sensor noise, in meters.  For the HDL-64E Velodyne Lidar, the sensor noise is usually less than 3 cm, so this value is set to 0.03.

Next, you will need to compute the horizontal and vertical sensor resolution of your sensor as a function of the distance to each tracked object.  For the HDL-64E Velodyne Lidar, there is a function getSensorResolution in sensor_specs.cpp which performs this calculation.  You can add a function that performs a similar calculation based on the resolution of your sensor.

After that, you should be all set to start precisely estimating object velocities!

USAGE
-----
To using the tracker in your own project, at the top of your file, add the header:

#include "precision_tracking/tracker.h"

For each object that you want to track, create a tracker:

  precision_tracking::Tracker tracker;
  precision_tracking::Params params;
  Eigen::Vector3f estimated_velocity;
  tracker.setPrecisionTracker(
          boost::make_shared<precision_tracking::PrecisionTracker>(&params));

Each time you observe the object, call the addPoints function:

  tracker.addPoints(points, timestamp, sensor_horizontal_resolution,
                     sensor_vertical_resolution, &estimated_velocity);

where:
points = the currently observed points for the object that you wish to track
timestamp = the time that these points were observed
sensor_horizontal_resolution = the horizontal resolution of the sensor
  at the distance of the tracked object
sensor_vertical_resolution = the vertical resolution of the sensor
  at the distance of the tracked object
estimated_velocity = the returned estimated translational velocity.  The tracker
  only outputs the estimated horizontal velocity - vertical motion and
  rotation are not currently estimated.  The velocity estimate is 0 for the
  first frame in which an object is observed and thereafter the
  estimated velocity is returned.

The horizontal and vertical resolution depend on the sensor that is used as well
as the distance to the tracked object; see the CONFIGURATION section above.

If you want to track many objects in parallel, it will be slightly more efficient to create a pool of trackers and have each thread use a tracker from that pool.  See test_tracking.cpp for an example.

MAINTAINERS
-----------
For questions about the tracker, contact David Held: davheld@cs.stanford.edu

LICENSE
-------
The tracker comes with an academic license for non-commercial purposes.  
For a commercial license to this software, please contact the 
Stanford Office of Technology and Licensing at info@otlmail.stanford.edu and mention the docket 14-219.

CITATIONS
---------
If you use this tracker in a scientific publication, please cite:

David Held, Jesse Levinson, Sebastian Thrun, Silvio Savarese.
Combining 3D Shape, Color, and Motion for Robust Anytime Tracking.
Robotics: Science and Systems (RSS), 2014

David Held, Jesse Levinson, Sebastian Thrun.
Precision Tracking with Sparse 3D and Dense Color 2D Data.
International Conference on Robotics and Automation (ICRA), 2013

