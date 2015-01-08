/*
 * sensor_specs.h
 *
 *      Author: davheld
 *
 * Functions specific to the sensor being used to collect 3D point cloud data.
 *
 */

#ifndef __PRECISION_TRACKING__SENSOR_SPECS_H
#define __PRECISION_TRACKING__SENSOR_SPECS_H

#include <Eigen/Eigen>

namespace precision_tracking {

void getSensorResolution(const Eigen::Vector3f& centroid_local_coordinates,
                         double* sensor_horizontal_res,
                         double* sensor_vertical_res);
} // namespace precision_tracking

#endif // __PRECISION_TRACKING__SENSOR_SPECS_H
