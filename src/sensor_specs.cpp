/*
 * sensor_specs.cpp
 *
 *      Author: davheld
 *
 */

#include "precision_tracking/sensor_specs.h"

#include <cmath>

namespace precision_tracking {

// Computes the sensor resolution for an object at a given distance,
// for the 64-beam Velodyne.
void getSensorResolution(const Eigen::Vector3f& centroid_local_coordinates,
                         double* sensor_horizontal_res,
                         double* sensor_vertical_res) {
  // Get the distance to the tracked object.
  const double horizontal_distance =
      sqrt(pow(centroid_local_coordinates(0), 2) +
           pow(centroid_local_coordinates(1), 2));

  // The horizontal resolution for the 64-beam Velodyne spinning at 10 Hz
  // is 0.18 degrees.
  const double velodyne_horizontal_angular_res = 0.18;

  // There are 64 beams spanning 26.8 vertical degrees, so the average spacing
  // between each beam is computed as follows.
  const double velodyne_vertical_angular_res = 26.8 / 63;

  // We convert the angular resolution to meters for a given range.
  const double velodyne_horizontal_res =
      2 * horizontal_distance *
      tan(velodyne_horizontal_angular_res / 2.0 * M_PI / 180.0);
  const double velodyne_vertical_res =
      2 * horizontal_distance *
      tan(velodyne_vertical_angular_res / 2.0 * M_PI / 180.0);

  *sensor_horizontal_res = velodyne_horizontal_res;
  *sensor_vertical_res = velodyne_vertical_res;
}

} // namespace precision_tracking

