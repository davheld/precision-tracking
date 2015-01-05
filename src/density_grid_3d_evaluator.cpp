/*
 * density_grid_3d_evaluator.cpp
 *
 *  Created on: Sep 1, 2013
 *      Author: davheld
 *
 */


#include <stdlib.h>
#include <numeric>

#include <pcl/common/common.h>

#include <precision_tracking/density_grid_3d_evaluator.h>


namespace precision_tracking {

namespace {

using std::vector;
using std::pair;
using std::max;
using std::min;

}  // namespace

// Initialize the density grid to all have log(kSmoothingFactor), so we do
// not give a probability of 0 to any location.
DensityGrid3dEvaluator::DensityGrid3dEvaluator(const Params *params)
  : AlignmentEvaluator(params)
  , density_grid_(params_->kMaxXSize, vector<vector<double> >(
                    params_->kMaxYSize, vector<double>(
                      params_->kMaxZSize, log(smoothing_factor_))))
{

}

DensityGrid3dEvaluator::~DensityGrid3dEvaluator()
{
	// TODO Auto-generated destructor stub
}

void DensityGrid3dEvaluator::init(const double xy_sampling_resolution,
          const double z_sampling_resolution,
          const double sensor_horizontal_resolution,
          const double sensor_vertical_resolution,
          const size_t num_current_points)
{
  AlignmentEvaluator::init(xy_sampling_resolution, z_sampling_resolution,
                           sensor_horizontal_resolution,
                           sensor_vertical_resolution, num_current_points);

  computeDensityGridParameters(
        prev_points_, xy_sampling_resolution, z_sampling_resolution,
        sensor_horizontal_resolution, sensor_vertical_resolution);

  computeDensityGrid(prev_points_);
}

void DensityGrid3dEvaluator::computeDensityGridParameters(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
    const double xy_sampling_resolution,
    const double z_sampling_resolution,
    const double xy_sensor_resolution,
    const double z_sensor_resolution)
{
  // Get the appropriate size for the grid.
  xy_grid_step_ = xy_sampling_resolution;

  // If we are not sampling in the z-direction, then create grid cells
  // with the ratio based on the sensor resolution.
  z_grid_step_ =
      z_sampling_resolution > 0 ? z_sampling_resolution :
        xy_sampling_resolution * (z_sensor_resolution / xy_sensor_resolution);

  // Find the min and max of the previous points.
  pcl::PointXYZRGB max_pt;
  pcl::getMinMax3D(*prev_points, min_pt_, max_pt);

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

  // We add one grid step of padding to allow for inexact matches.  The outer
  // grid cells are kept empty and are used to represent the empty space
  // around the tracked object.
  max_pt.x += 2 * xy_grid_step_;
  max_pt.y += 2 * xy_grid_step_;
  max_pt.z += 2 * z_grid_step_;

  // Find the appropriate size for the density grid.
  xSize_ = min(params_->kMaxXSize, max(1, static_cast<int>(
      ceil((max_pt.x - min_pt_.x) / xy_grid_step_))));
  ySize_ = min(params_->kMaxYSize, max(1, static_cast<int>(
      ceil((max_pt.y - min_pt_.y) / xy_grid_step_))));
  zSize_ = min(params_->kMaxZSize, max(1, static_cast<int>(
      ceil((max_pt.z - min_pt_.z) / z_grid_step_))));

  // Reset the density grid to the default value.
  const double default_val = log(smoothing_factor_);
  for (int i = 0; i < xSize_; ++i) {
    for (int j = 0; j < ySize_; ++j) {
      std::fill(
          density_grid_[i][j].begin(),
          density_grid_[i][j].begin() + zSize_, default_val);
    }
  }

  // In our discrete grid, we want to compute the Gaussian for a certian
  // number of grid cells away from the point.
  num_spillover_steps_xy_ =
      ceil(params_->kSpilloverRadius * sigma_xy_ / xy_grid_step_ - 1);
  // Our implementation requires that we spill over at least 1 cell in the
  // z direction.
  num_spillover_steps_z_ =
      max(1.0, ceil(params_->kSpilloverRadius * sigma_z_ / z_grid_step_ - 1));
}

void DensityGrid3dEvaluator::computeDensityGrid(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& points)
{
  // Apply this offset when converting from the point location to the index.
  const double x_offset = -min_pt_.x / xy_grid_step_;
  const double y_offset = -min_pt_.y / xy_grid_step_;
  const double z_offset = -min_pt_.z / z_grid_step_;

  // Convert sigma to a factor such that
  // exp(-x^2 * grid_size^2 / 2 sigma^2) = exp(x^2 * factor)
  // where x is the number of grid steps.
  const double xy_exp_factor =
      -1.0 * pow(xy_grid_step_, 2) / (2 * pow(sigma_xy_, 2));
  const double z_exp_factor =
      -1.0 * pow(z_grid_step_, 2) / (2 * pow(sigma_z_, 2));

  // For any given point, the density falls off as a Gaussian to
  // neighboring regions.
  // Pre-compute the density spillover for different cell distances.
  vector<vector<vector<double> > > spillovers(
        num_spillover_steps_xy_ + 1, vector<vector<double> >(
          num_spillover_steps_xy_ + 1, vector<double>(
            num_spillover_steps_z_ + 1)));
  for (int i = 0; i <= num_spillover_steps_xy_; ++i) {
    const int i_dist_sq = pow(i, 2);

    for (int j = 0; j <= num_spillover_steps_xy_; ++j) {
      const int j_dist_sq = pow(j, 2);
      const double log_xy_density = (i_dist_sq + j_dist_sq) * xy_exp_factor;

      for (int k = 0; k <= num_spillover_steps_z_; ++k) {
        const int k_dist_sq = pow(k, 2);
        const double log_z_density = k_dist_sq * z_exp_factor;

        spillovers[i][j][k] = log(
              exp(log_xy_density + log_z_density) + smoothing_factor_);
      }
    }
  }

  if (num_spillover_steps_z_ == 0) {
    printf("Error - we assume that we are spilling at least 1 in the"
           "z-direction\n");
  }

  // Build the density grid
  size_t num_points = points->size();

  for (size_t i = 0; i < num_points; ++i) {
    const pcl::PointXYZRGB& pt = (*points)[i];

    // Find the indices for this point.
    const int x_index = round(pt.x / xy_grid_step_ + x_offset);
    const int y_index = round(pt.y / xy_grid_step_ + y_offset);
    const int z_index = round(pt.z / z_grid_step_ + z_offset);

    // Add limit checks to make sure we don't segfault
    if (x_index < 1 || x_index > xSize_ - 2) {
      continue;
    }
    if (y_index < 1 || y_index > ySize_ - 2) {
      continue;
    }

    // Spill the probability density into neighboring regions as a Guassian
    // (but not to the borders, which represent the empty space around the
    // tracked object)
    const int max_x_index =
        max(1, min(xSize_ - 2, x_index + num_spillover_steps_xy_));
    const int max_y_index =
        max(1, min(ySize_ - 2, y_index + num_spillover_steps_xy_));

    const int min_x_index =
        min(xSize_ - 2, max(1, x_index - num_spillover_steps_xy_));
    const int min_y_index =
        min(ySize_ - 2, max(1, y_index - num_spillover_steps_xy_));

    if (num_spillover_steps_z_ > 1) {
      const int max_z_index =
          max(1, min(zSize_ - 2, z_index + num_spillover_steps_z_));
      const int min_z_index =
          max(1, min(zSize_ - 2, z_index - num_spillover_steps_z_));

      // Spill the probability into neighboring cells as a Guassian.
      for (int x_spill = min_x_index; x_spill <= max_x_index; ++x_spill){
        const int x_diff = abs(x_index - x_spill);

        for (int y_spill = min_y_index; y_spill <= max_y_index; ++y_spill) {
          const int y_diff = abs(y_index - y_spill);

          for (int z_spill = min_z_index; z_spill <= max_z_index; ++z_spill) {
            const int z_diff = abs(z_index - z_spill);

          const double spillover = spillovers[x_diff][y_diff][z_diff];

          density_grid_[x_spill][y_spill][z_spill] =
              max(density_grid_[x_spill][y_spill][z_spill], spillover);
          }
        }
      }
    } else {
      // This is an optimization that we can do if we are only spilling
      // over 1 grid cell, which happens fairy often.

      // For z, we only spill up one and down one, so pre-compute these.
      const int z_spill = std::min(std::max(1, z_index), zSize_ - 2);
      const int z_spill_up = std::min(z_spill + 1, zSize_ - 2);
      const int z_spill_down = std::max(1, z_spill - 1);

      // Spill the probability into neighboring cells as a Guassian.
      for (int x_spill = min_x_index; x_spill <= max_x_index; ++x_spill){
        const int x_diff = abs(x_index - x_spill);

        for (int y_spill = min_y_index; y_spill <= max_y_index; ++y_spill) {
          const int y_diff = abs(y_index - y_spill);

          const double spillover0 = spillovers[x_diff][y_diff][0];

          density_grid_[x_spill][y_spill][z_spill] =
              max(density_grid_[x_spill][y_spill][z_spill], spillover0);

          const double spillover1 = spillovers[x_diff][y_diff][1];

          density_grid_[x_spill][y_spill][z_spill_up] =
              max(density_grid_[x_spill][y_spill][z_spill_up], spillover1);

          density_grid_[x_spill][y_spill][z_spill_down] =
              max(density_grid_[x_spill][y_spill][z_spill_down], spillover1);

        }
      }
    }
  }
}

double DensityGrid3dEvaluator::getLogProbability(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
    const Eigen::Vector3f& ,
    const MotionModel& motion_model,
    const double delta_x, const double delta_y, const double delta_z)
{
  // Amount of total log probability density for the given alignment.
  double total_log_density = 0;

  // Offset to apply to each point to get the new position.
  const double x_offset = (delta_x - min_pt_.x) / xy_grid_step_;
  const double y_offset = (delta_y - min_pt_.y) / xy_grid_step_;
  const double z_offset = (delta_z - min_pt_.z) / z_grid_step_;

  // Iterate over every point and look up its log probability density
  // in the density grid.
	const size_t num_points = current_points->size();
  for (size_t i = 0; i < num_points; ++i) {
    // Extract the point so we can compute its probability.
  	const pcl::PointXYZRGB& pt = (*current_points)[i];

    // We shift each point based on the proposed alignment, to try to
    // align the current points with the previous points.  We then
    // divide by the grid step to find the appropriate cell in the density
    // grid.
    const int x_index_shifted =
        min(max(0, static_cast<int>(round(pt.x / xy_grid_step_ + x_offset))),
            xSize_ - 1);
    const int y_index_shifted =
        min(max(0, static_cast<int>(round(pt.y / xy_grid_step_ + y_offset))),
            ySize_ - 1);
    const int z_index_shifted =
        min(max(0, static_cast<int>(round(pt.z / z_grid_step_ + z_offset))),
            zSize_ - 1);

    // Look up the log density of this grid cell and add to the total density.
    total_log_density +=
        density_grid_[x_index_shifted][y_index_shifted][z_index_shifted];
  }

  // Compute the motion model probability.
  const double motion_model_prob = motion_model.computeScore(
              delta_x, delta_y, delta_z);

  // Compute the log measurement probability.
  const double log_measurement_prob = total_log_density;

  // Combine the motion model score with the (discounted) measurement score to
  // get the final log probability.
  const double log_prob = log(motion_model_prob) +
      measurement_discount_factor_ * log_measurement_prob;

  return log_prob;
}

} // namespace precision_tracking
