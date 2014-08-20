/*
 * lf_discrete_3d.cpp
 *
 * Created on: May 1, 2014
 *      Author: davheld
 *
 * Discretized version of the likelihood field tracker from
 * Probabilistic Robotics, Thrun, et al, 2005.
 *
 * Parameters are taken based on the latent surface model from RSS 2014,
 * Held, et al.
 *
 * The 3d volume is discretized and the probability values are cached
 * after they are computed.
 */

#include "lf_discrete_3d.h"

#include <utility>
#include <vector>
#include <algorithm>

#include <pcl/common/common.h>

using std::max;
using std::min;
using std::pair;
using std::vector;

namespace {

// Approximation factor for finding the nearest neighbor.
// Set to 0 to find the exact nearest neighbor.
// For a reasonable speedup, set to 2.
const double kSearchTreeEpsilon = 2;

// Total size = 3.7 GB
// At a resolution of 1.2 cm, a 10 m wide object will take 1000 cells.
const int kMaxXSize = 1000;
const int kMaxYSize = 1000;
// At a resolution of 1.2 cm, a 5 m tall object will take 500 cells.
const int kMaxZSize = 500;

}


LFDiscrete3d::LFDiscrete3d()
    : lf_cache_3D_log_prob_(kMaxXSize * kMaxYSize * kMaxZSize),
      lf_cache_3D_cached_(kMaxXSize * kMaxYSize * kMaxZSize),
      searchTree_(false),  //  //By setting sorted to false,
                                // the radiusSearch operations will be faster.
      max_nn_(1),
      nn_indices_(max_nn_),
      nn_sq_dists_(max_nn_),
      fast_functions_(FastFunctions::getInstance())
{
}

LFDiscrete3d::~LFDiscrete3d() {
  // TODO Auto-generated destructor stub
}

void LFDiscrete3d::setPrevPoints(
        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr prev_points) {
      AlignmentEvaluator::setPrevPoints(prev_points);
  // Set the input cloud for the search tree to the previous points for NN
  // lookups.
  searchTree_.setInputCloud(prev_points_);

  // Set search tree epsilon for a speedup.
  searchTree_.setEpsilon(kSearchTreeEpsilon);
}

void LFDiscrete3d::init(
        const double xy_sampling_resolution,
        const double z_sampling_resolution,
        const double xy_sensor_resolution,
        const double z_sensor_resolution,
        const size_t num_current_points) {
    AlignmentEvaluator::init(xy_sampling_resolution, z_sampling_resolution,
                             xy_sensor_resolution,
                             z_sensor_resolution,
                             num_current_points);

    // Get the appropriate size for the grid.
    xy_grid_step_ = xy_sampling_resolution;

    // If we are not sampling in the z-direction, then create grid cells
    // with the ratio based on the sensor resolution.
    z_grid_step_ =
        z_sampling_resolution > 0 ? z_sampling_resolution :
          xy_sampling_resolution * (z_sensor_resolution / xy_sensor_resolution);

    // Find the min and max of the previous points.
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D(*prev_points_, min_pt_, max_pt);

    const double epsilon = 0.0001;

    // We add one grid step of padding to allow for inexact matches.  The outer
    // grid cells are kept empty and are used to represent the empty space
    // around the tracked object.
    min_pt_.x -= (2 * xy_grid_step_ + epsilon);
    min_pt_.y -= (2 * xy_grid_step_ + epsilon);

    // If we have a large step size in the z-direction, we want to center
    // the object within the grid cell.
    const double z_range = max_pt.z - min_pt_.z;
    const double z_centering = fabs(z_grid_step_ - z_range) / 2;
    min_pt_.z -= (2 * z_grid_step_ + z_centering);

    min_center_pt_.x = min_pt_.x + xy_grid_step_ / 2;
    min_center_pt_.y = min_pt_.y + xy_grid_step_ / 2;
    min_center_pt_.z = min_pt_.z + z_grid_step_ / 2;

    // We add one grid step of padding to allow for inexact matches.  The outer
    // grid cells are kept empty and are used to represent the empty space
    // around the tracked object.
    max_pt.x += 2 * xy_grid_step_;
    max_pt.y += 2 * xy_grid_step_;
    max_pt.z += 2 * z_grid_step_;

    // Find the appropriate size for the density grid.
    xSize_ = min(kMaxXSize, max(1, static_cast<int>(
        ceil((max_pt.x - min_pt_.x) / xy_grid_step_))));
    ySize_ = min(kMaxYSize, max(1, static_cast<int>(
        ceil((max_pt.y - min_pt_.y) / xy_grid_step_))));
    zSize_ = min(kMaxZSize, max(1, static_cast<int>(
        ceil((max_pt.z - min_pt_.z) / z_grid_step_))));

  xySize_ = xSize_ * ySize_;

  std::fill(lf_cache_3D_cached_.begin(),
      lf_cache_3D_cached_.begin() + xSize_ * ySize_ * zSize_, false);
}

double LFDiscrete3d::getLogProbability(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
    const Eigen::Vector3f& ,
    const MotionModel& motion_model,
    const double delta_x, const double delta_y, const double delta_z) {
  // Total log measurement probability.
  double log_measurement_prob = 0;

  // Compute the grid offset to apply to each point based on the proposed
  // displacement (x, y, z).
  const double x_offset = (delta_x - min_pt_.x) / xy_grid_step_;
  const double y_offset = (delta_y - min_pt_.y) / xy_grid_step_;
  const double z_offset = (delta_z - min_pt_.z) / z_grid_step_;

  // Iterate over every point, and look up its score in the density grid.
  const size_t num_points = current_points->size();

  for (size_t i = 0; i < num_points; ++i) {
    // Extract the point so we can compute its score.
    const pcl::PointXYZRGB& pt = (*current_points)[i];

    // TODO - compare the speed here to branching.
    // Compute the grid-cell for this point.
    // Cut off indices at the limits.
    const int x_index = min(max(0,
        static_cast<int>(pt.x / xy_grid_step_ + x_offset)), xSize_ - 1);
    const int y_index = min(max(0,
        static_cast<int>(pt.y / xy_grid_step_ + y_offset)), ySize_ - 1);
    const int z_index = min(max(0,
        static_cast<int>(pt.z / z_grid_step_ + z_offset)), zSize_ - 1);

    // Look up the score in the density grid.
    //log_measurement_prob += densityMap3D_[x_index_shifted][y_index_shifted][z_index_shifted];

    const int index = get_index(x_index, y_index, z_index);

    // Find if the object was cached.
    const bool cached = lf_cache_3D_cached_[index];

    // Compute the probability, if necessary, and cache.
    if (!cached){
      lf_cache_3D_log_prob_[index] = get_log_prob(x_index, y_index, z_index);
      lf_cache_3D_cached_[index] = true;
    }

    log_measurement_prob += lf_cache_3D_log_prob_[index];
  }

  // Compute the motion model probability.
  const double motion_model_prob = motion_model.computeScore(
              delta_x, delta_y, delta_z);

  // Combine the motion model score with the (discounted) measurement score to
  // get the final log probability.
  const double log_prob = log(motion_model_prob) +
          measurement_discount_factor_ * log_measurement_prob;

  return log_prob;
}

double LFDiscrete3d::get_log_prob(
    const int x_index, const int y_index, const int z_index) {
    // Get the 3d position for the center of this grid cell.
  pt_.x = (x_index * xy_grid_step_ + min_center_pt_.x);
  pt_.y = (y_index * xy_grid_step_ + min_center_pt_.y);
  pt_.z = (z_index * z_grid_step_ + min_center_pt_.z);

  searchTree_.nearestKSearch(pt_, max_nn_, nn_indices_, nn_sq_dists_);

  // Compute the log probability of this neighbor match.
  // The NN search is isotropic, but our scoring function is not!
  // To acccount for this, we weight the NN search only by the isotropic
  // xyz_exp_factor_.
  const double log_point_match_prob_i = nn_sq_dists_[0] * xyz_exp_factor_;

  // Compute the probability of this neighbor match.
  const double point_match_prob_spatial_i = exp(
      log_point_match_prob_i);

  // Compute the probability for this point.
  const double point_prob = point_match_prob_spatial_i + smoothing_factor_;

  // Compute the log.
  const double log_point_prob = fast_functions_.getFastLog(point_prob);
  //const double log_point_prob = log(point_prob);

  return log_point_prob;
}

