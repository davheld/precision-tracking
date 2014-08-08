/*
 * density_grid_tracker.cpp
 *
 *  Created on: Sep 1, 2013
 *      Author: davheld
 */

#include "density_grid_tracker.h"

#include <stdlib.h>
#include <numeric>
#include <boost/math/constants/constants.hpp>

#include <pcl/common/common.h>

#include "model_builder.h"

namespace {

// We assume that there are this many independent points per object.  Beyond
// this many, we discount the measurement model accordingly.
const double max_discount_points = getenv("MAX_DISCOUNT") ? atof(getenv("MAX_DISCOUNT")) : 150.0;

// How far to spill over in the density grid (number of sigmas).
const double kSpilloverRadius = getenv("SPILLOVER_RADIUS") ? atof(getenv("SPILLOVER_RADIUS")) : 2.0;

// Factor to multiply the sensor resolution for our measurement model.
// We model each point as a Gaussian: exp(-x^2 / 2 sigma^2)
// With sigma^2 = (sensor_resolution * kSigmaFactor)^2 + other terms.
const double kSigmaFactor = getenv("SIGMA_FACTOR") ? atof(getenv("SIGMA_FACTOR")) : 0.5;

// Factor to multiply the particle sampling resolution for our measurement  model.
// We model each point as a Gaussian: exp(-x^2 / 2 sigma^2)
// With sigma^2 = (sampling_resolution * kSigmaGridFactor)^2 + other terms.
const double kSigmaGridFactor = getenv("SIGMA_GRID_FACTOR") ? atof(getenv("SIGMA_GRID_FACTOR")) : 1;

// The noise in our sensor which is independent of the distance to the tracked object.
// We model each point as a Gaussian: exp(-x^2 / 2 sigma^2)
// With sigma^2 = kMinMeasurementVariance^2 + other terms.
const double kMinMeasurementVariance = getenv("MIN_MEASUREMENT_VAR") ? atof(getenv("MIN_MEASUREMENT_VAR")) : 0.03;

// We add this to our Gaussian so we don't give 0 probability to points
// that don't align.
// We model each point as a Gaussian: exp(-x^2 / 2 sigma^2) + kSmoothingFactor
const double kSmoothingFactor = getenv("SMOOTHING_FACTOR") ? atof(getenv("SMOOTHING_FACTOR")) : 1;

// We multiply our log measurement probability by this factor, to decrease
// our confidence in the measurement model (e.g. to take into account
// dependencies between neighboring points).
const double kMeasurementDiscountFactor = getenv("MEASUREMENT_DISCOUNT_3D") ? atof(getenv("MEASUREMENT_DISCOUNT_3D")) : 1;

const bool k_NNTracking = true;

const double pi = boost::math::constants::pi<double>();

// Total size = 3.7 GB
// At a resolution of 1.2 cm, a 10 m wide object will take 1000 cells.
const int kMaxXSize = k_NNTracking ? 1 : 1000;
const int kMaxYSize = k_NNTracking ? 1 : 1000;
// At a resolution of 1.2 cm, a 5 m tall object will take 500 cells.
const int kMaxZSize = k_NNTracking ? 1 : 500;

using std::vector;
using std::pair;
using std::max;
using std::min;

// Get the density grid index for a given 3D point.
template <class Point>
void pointToIndex(const Point& pt,
    const double xy_gridStep,
    const double z_gridStep,
		const double x_offset,
		const double y_offset,
    const double z_offset,
		int* xIndex, int* yIndex, int* zIndex) {
  *xIndex = round(pt.x / xy_gridStep + x_offset);
  *yIndex = round(pt.y / xy_gridStep + y_offset);
  *zIndex = round(pt.z / z_gridStep + z_offset);
}

}  // namespace

// Initialize the density grid to all have log(kSmoothingFactor), so we do
// not give a probability of 0 to any location.
DensityGridTracker::DensityGridTracker()
  : fast_functions_(FastFunctions::getInstance()),
    density_grid_(kMaxXSize, vector<vector<double> >(
        kMaxYSize, vector<double>(kMaxZSize, log(kSmoothingFactor)))) {
}

DensityGridTracker::~DensityGridTracker() {
	// TODO Auto-generated destructor stub
}

void DensityGridTracker::track(
    const double xy_stepSize,
    const double z_stepSize,
    const pair <double, double>& xRange,
    const pair <double, double>& yRange,
    const pair <double, double>& zRange,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
    const Eigen::Vector3f& current_points_centroid,
		const MotionModel& motion_model,
		const double horizontal_distance,
		const double down_sample_factor,
		const double point_ratio,
    ScoredTransforms<ScoredTransformXYZ>* transforms) {
	// Find all candidate xyz transforms.
  vector<XYZTransform> xyz_transforms;
  createCandidateXYZTransforms(xy_stepSize, z_stepSize,
      xRange, yRange, zRange, &xyz_transforms);

  // Get scores for each of the xyz transforms.
  scoreXYZTransforms(
  		current_points, prev_points, current_points_centroid,
  		xy_stepSize, z_stepSize,
  		xyz_transforms, motion_model, horizontal_distance, down_sample_factor,
  		point_ratio, transforms);
}

void DensityGridTracker::createCandidateXYZTransforms(
    const double xy_step_size,
    const double z_step_size,
    const std::pair <double, double>& xRange,
    const std::pair <double, double>& yRange,
    const std::pair <double, double>& zRange_orig,
    std::vector<XYZTransform>* transforms) {
  if (xy_step_size == 0) {
    printf("Error - xy step size must be > 0");
    exit(1);
  }

  if (z_step_size == 0) {
    printf("Error - z step size must be > 0");
    exit(1);
  }

  // Make sure we hit 0 in our z range, in case the step is too large.
  std::pair<double, double> zRange;
  if (z_step_size > fabs(zRange.second - zRange_orig.first)) {
    zRange.first = 0;
    zRange.second = 0;
  } else {
    zRange.first = zRange_orig.first;
    zRange.second = zRange_orig.second;
  }

  // Compute the number of transforms along each direction.
  const int num_x_locations = (xRange.second - xRange.first) / xy_step_size + 1;
  const int num_y_locations = (yRange.second - yRange.first) / xy_step_size + 1;
  const int num_z_locations = (zRange.second - zRange.first) / z_step_size + 1;

  // Reserve space for all of the transforms.
  transforms->reserve(num_x_locations * num_y_locations * num_z_locations);

  const double volume = pow(xy_step_size, 2) * z_step_size;

  // Create a list of candidate transforms.
  for (double x = xRange.first; x <= xRange.second; x += xy_step_size){
    for (double y = yRange.first; y <= yRange.second; y += xy_step_size){
      for (double z = zRange.first; z <= zRange.second; z +=z_step_size){
        XYZTransform transform(x, y, z, volume);
        transforms->push_back(transform);
      }
    }
  }
}

void DensityGridTracker::scoreXYZTransforms(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
    const Eigen::Vector3f& current_points_centroid,
    const double xy_stepSize,
    const double z_stepSize,
    const vector<XYZTransform>& transforms,
    const MotionModel& motion_model,
    const double horizontal_distance,
    const double down_sample_factor,
    const double point_ratio,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms) {
  // Determine the size and minimum density for the density grid.
  computeDensityGridSize(prev_points, xy_stepSize, z_stepSize, point_ratio,
      horizontal_distance, down_sample_factor);

  computeDensityGrid(prev_points);

  const size_t num_transforms = transforms.size();

  // Compute scores for all of the transforms using the density grid.
  scored_transforms->clear();
  scored_transforms->reserve(num_transforms);
  for(size_t i = 0; i < num_transforms; ++i){
    const XYZTransform& transform = transforms[i];
    const double& x = transform.x;
    const double& y = transform.y;
    const double& z = transform.z;
    const double& volume = transform.volume;

    const double log_prob = getLogProbability(current_points, minPt,
        xy_grid_step, z_grid_step, motion_model, x, y, z);

    // Save the complete transform with its log probability.
    const ScoredTransformXYZ scored_transform(x, y, z, log_prob, volume);
    scored_transforms->addScoredTransform(scored_transform);
  }
}

void DensityGridTracker::computeDensityGridSize(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
    const double xy_stepSize,
    const double z_stepSize,
    const double point_ratio,
    const double horizontal_distance,
    const double down_sample_factor) {
  // Get the appropriate size for the grid.
  xy_grid_step = xy_stepSize;
  z_grid_step = z_stepSize;

  if (prev_points->size() < max_discount_points) {
      discount_factor_ = 1;
  } else {
      discount_factor_ = max_discount_points / prev_points->size();
  }

  // Find the min and max of the previous points.
  pcl::PointXYZRGB maxPt;
  pcl::getMinMax3D(*prev_points, minPt, maxPt);

  const double z_range = maxPt.z - minPt.z;

  const double z_centering = fabs(z_grid_step - z_range) / 2;

  const double epsilon = 0.0001;

  // Add some padding.
  minPt.x -= (2 * xy_grid_step + epsilon);
  minPt.y -= (2 * xy_grid_step + epsilon);
  minPt.z -= (2 * z_grid_step + z_centering);

  maxPt.x += 2 * xy_grid_step;
  maxPt.y += 2 * xy_grid_step;
  maxPt.z += 2 * z_grid_step;

  // Find the appropriate size for the density grid.
  xSize_ = min(kMaxXSize, max(1, static_cast<int>(
      ceil((maxPt.x - minPt.x) / xy_grid_step))));
  ySize_ = min(kMaxYSize, max(1, static_cast<int>(
      ceil((maxPt.y - minPt.y) / xy_grid_step))));
  zSize_ = min(kMaxZSize, max(1, static_cast<int>(
      ceil((maxPt.z - minPt.z) / z_grid_step))));

  // Reset the density grid to the default value.
  const double default_val = log(kSmoothingFactor);
  for (int i = 0; i < xSize_; ++i) {
    for (int j = 0; j < ySize_; ++j) {
      std::fill(
          density_grid_[i][j].begin(),
          density_grid_[i][j].begin() + zSize_, default_val);
    }
  }

  // Compute the sensor horizontal resolution
  const double velodyne_horizontal_res = 2 * horizontal_distance * tan(.18 / 2.0 * pi / 180.0);

  //printf("Horizontal res: %lf, kSigmaFactor: %lf\n", velodyne_horizontal_res, kSigmaFactor);

  // The vertical resolution for the Velodyne is 2.2 * the horizontal resolution.
  const double velodyne_vertical_res = 2.2 * velodyne_horizontal_res;

  const double error1_xy = kSigmaGridFactor * xy_stepSize;
  const double error2_xy = velodyne_horizontal_res * kSigmaFactor;
  spillover_sigma_xy = sqrt(pow(error1_xy, 2) + pow(error2_xy, 2) + pow(kMinMeasurementVariance, 2));

  const double error1_z = kSigmaGridFactor * z_stepSize;
  const double error2_z = velodyne_vertical_res * kSigmaFactor;
  spillover_sigma_z = sqrt(pow(error1_z, 2) + pow(error2_z, 2) + pow(kMinMeasurementVariance, 2));

  num_spillover_steps_xy =
      ceil(kSpilloverRadius * spillover_sigma_xy / xy_grid_step - 1);
  num_spillover_steps_z = 1;
  //*num_spillover_steps_z = 1;
  //*num_spillover_steps_z =
  //    ceil(kSpilloverRadius * *spillover_sigma_z / *z_grid_step - 1);

  minDensity = kSmoothingFactor;
}

void DensityGridTracker::computeDensityGrid(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& points) {
  // Apply this offset when converting from the point location to the index.
  const double x_offset = -minPt.x / xy_grid_step;
  const double y_offset = -minPt.y / xy_grid_step;
  const double z_offset = -minPt.z / z_grid_step;

  // Convert the sigma to a factor such that
  // exp(-x^2 * grid_size^2 / 2 sigma^2) = exp(x^2 * factor)
  // where x is the number of grid steps.
    const double xy_exp_factor = -1.0 * pow(xy_grid_step, 2) / (2 * pow(spillover_sigma_xy, 2));
    const double z_exp_factor = -1.0 * pow(z_grid_step, 2) / (2 * pow(spillover_sigma_z, 2));


    /*const int max_spillover =  2 * pow(num_spillover_steps_xy, 2);
    vector<double> spillovers(max_spillover + 1);
    for (int i = 0; i <= max_spillover; ++i) {
      spillovers[i] = fast_functions_.getFastLog(exp(i * xy_exp_factor) + min_density);
    }*/

    //const int max_spillover =  2 * pow(num_spillover_steps_xy, 2);

    // For z_diff = 0;
    vector<vector<double> > spillovers0(num_spillover_steps_xy + 1,
        vector<double>(num_spillover_steps_xy + 1));
    for (int i = 0; i <= num_spillover_steps_xy; ++i) {
      const int i_dist_sq = pow(i,2);
      for (int j = 0; j <= num_spillover_steps_xy; ++j) {
        spillovers0[i][j] = fast_functions_.getFastLog(
            exp((i_dist_sq + pow(j,2)) * xy_exp_factor)
            + minDensity);
      }
    }

    //For z_diff = 1;
    vector<vector<double> > spillovers1(num_spillover_steps_xy + 1,
        vector<double>(num_spillover_steps_xy + 1));
    for (int i = 0; i <= num_spillover_steps_xy; ++i) {
      const int i_dist_sq = pow(i,2);
      for (int j = 0; j <= num_spillover_steps_xy; ++j) {
        spillovers1[i][j] = fast_functions_.getFastLog(
            exp((i_dist_sq + pow(j,2)) * xy_exp_factor + z_exp_factor)
            + minDensity);
      }
    }

  // Build the density grid
  size_t num_points = points->size();

  for (size_t i = 0; i < num_points; ++i) {
    const pcl::PointXYZRGB& pt = (*points)[i];

    // Find the indices for this point.
    const int x_index = round(pt.x / xy_grid_step + x_offset);
    const int y_index = round(pt.y / xy_grid_step + y_offset);
    const int z_index = round(pt.z / z_grid_step + z_offset);

    //int x_index, y_index, z_index;
    //pointToIndex(pt, xy_grid_step, z_grid_step, x_offset, y_offset,
    //    z_offset, &x_index, &y_index, &z_index);

    const int z_spill = std::min(std::max(2, z_index), zSize_ - 3);

    //if (num_spillover_steps_xy > 0 || num_spillover_steps_z > 0) {
      // Spill over into neighboring regions (but not to the borders, which
      // represent emptiness.
      const int max_x_index = min(xSize_ - 2, x_index + num_spillover_steps_xy);
      const int max_y_index = min(ySize_ - 2, y_index + num_spillover_steps_xy);
      //const int max_z_index = min(zSize_ - 2, z_index + num_spillover_steps_z);

      const int min_x_index = max(1, x_index - num_spillover_steps_xy);
      const int min_y_index = max(1, y_index - num_spillover_steps_xy);
      //const int min_z_index = max(1, z_index - num_spillover_steps_z);

      for (int x_spill = min_x_index; x_spill <= max_x_index; ++x_spill){
        //const double x_distance_sq = pow(x_index - x_spill, 2);
        const int x_diff = abs(x_index - x_spill);

        for (int y_spill = min_y_index; y_spill <= max_y_index; ++y_spill) {
          const int y_diff = abs(y_index - y_spill);

          //const double xy_grid_distance_sq = x_distance_sq + pow(y_index - y_spill, 2);
          //const double spillover = spillovers[xy_grid_distance_sq];

    /*for (int x_diff = 0; x_diff <= num_spillover_steps_xy; ++x_diff) {
      for (int y_diff = 0; y_diff <= num_spillover_steps_xy; ++y_diff) {

        const double spillover = spillovers2[x_diff][y_diff];

        for (int x_mult = -1; x_mult <= 2; x_mult += 2) {
          const int x_spill = max(1, min(xSize_ - 2, x_index + x_diff * x_mult));

          for (int y_mult = -1; y_mult <= 2; y_mult += 2) {
            const int y_spill = max(1, min(ySize_ - 2, y_index + y_diff * y_mult));

            double& density_val = densityMap3D_[x_spill][y_spill][z_spill];

            density_val = max(density_val, spillover);
          }
        }*/

          const double spillover0 = spillovers0[x_diff][y_diff];

          density_grid_[x_spill][y_spill][z_spill] =
              max(density_grid_[x_spill][y_spill][z_spill], spillover0);

          const double spillover1 = spillovers1[x_diff][y_diff];

          density_grid_[x_spill][y_spill][z_spill+1] =
              max(density_grid_[x_spill][y_spill][z_spill+1], spillover1);

          density_grid_[x_spill][y_spill][z_spill-1] =
              max(density_grid_[x_spill][y_spill][z_spill-1], spillover1);

        }
      }
    }
}

double DensityGridTracker::getLogProbability(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
		const pcl::PointXYZRGB& minPt,
    const double xy_gridStep,
    const double z_gridStep,
		const MotionModel& motion_model,
		const double x,
		const double y,
    const double z) const {

	// Amount of total overlap.
	double numOccupied = 0;

	const double x_offset = (x - minPt.x) / xy_gridStep;
	const double y_offset = (y - minPt.y) / xy_gridStep;
  const double z_offset = (z - minPt.z) / z_gridStep;

  // Iterate over every point, and look up its score in the density grid.
	const size_t num_points = current_points->size();
  for (size_t i = 0; i < num_points; ++i) {
  	// Extract the point so we can compute its score.
  	const pcl::PointXYZRGB& pt = (*current_points)[i];

    // Cut off indices at the limits.
    const int x_index_shifted = min(max(0, static_cast<int>(round(pt.x / xy_gridStep + x_offset))), xSize_ - 1);;
    const int y_index_shifted = min(max(0, static_cast<int>(round(pt.y / xy_gridStep + y_offset))), ySize_ - 1);
    const int z_index_shifted = min(max(0, static_cast<int>(round(pt.z / z_gridStep + z_offset))), zSize_ - 1);

    /*if (z_index_shifted != 2) {
      //printf("Scoring transform, point: %d, pt.z: %lf, z_index: %d\n", i, pt.z, z_index_shifted);
    }*/

    //printf("z_index_shifted: %d\n", z_index_shifted);

    numOccupied += density_grid_[x_index_shifted][y_index_shifted][z_index_shifted];
  }

  // Compute the motion model probability.
  const double motion_model_prob = motion_model.computeScore(x, y, z);

  // Compute the log measurement probability.
  const double log_measurement_prob = numOccupied;

  // Combine the motion model score with the (discounted) measurement score to
  // get the final log probability.
  const double log_prob =
      log(motion_model_prob) + discount_factor_ * kMeasurementDiscountFactor * log_measurement_prob;

  //printf("motion_model_prob: %lf, log_measurement_prob: %lf\n",
  //    motion_model_prob, log_measurement_prob);

  return log_prob;
}
