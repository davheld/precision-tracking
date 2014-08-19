/*
 * density_grid_tracker.h
 *
 *  Created on: Sep 1, 2013
 *      Author: davheld
 *
 * Compute the probability of a given set of alignments.
 * To do this quickly, we pre-cache probability values in a density grid for
 * fast lookups.
 *
 */

#ifndef DENSITY_GRID_TRACKER_H_
#define DENSITY_GRID_TRACKER_H_

#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "scored_transform.h"
#include "alignment_evaluator.h"


struct ScoredTransform;
struct XYZTransform;
class MotionModel;

class DensityGridTracker : public AlignmentEvaluator {
public:
  DensityGridTracker();
  virtual ~DensityGridTracker();

private:
  void init(const double xy_sampling_resolution,
            const double z_sampling_resolution,
            const double sensor_horizontal_resolution,
            const double sensor_vertical_resolution);

  double getLogProbability(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const Eigen::Vector3f& current_points_centroid,
      const MotionModel& motion_model,
      const double x, const double y, const double z);

  void computeDensityGridParameters(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
      const double xy_sampling_resolution,
      const double z_sampling_resolution,
      const double xy_sensor_resolution,
      const double z_sensor_resolution);

  // Pre-cache probability values in a density grid for fast lookups.
  void computeDensityGrid(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points);

  // Get the score of this transform, using the density grid.
  double getLogProbability(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const pcl::PointXYZRGB& min_pt_,
      const double xy_gridStep,
      const double z_gridStep,
      const MotionModel& motion_model,
      const double x,
      const double y,
      const double z) const;

  // A grid used to pre-cache probability values for fast lookups.
  std::vector<std::vector<std::vector<double> >  > density_grid_;

  // The size of the resulting grid.
  int xSize_;
  int ySize_;
  int zSize_;

  // How much to discount the measurement model, based on dependencies
  // between points.
  double discount_factor_;

  // The step size of the density grid.
  double xy_grid_step_;
  double z_grid_step_;

  // The minimum point of the previous set of points used for tracking.
  pcl::PointXYZRGB min_pt_;

  // In our discrete grid, we want to compute the Gaussian probability
  // for this many grid cells away from each point.  As we get farther
  // away from the point, the probability of the Guassian goes to 0
  // so we only need to compute the probability at a limited number
  // of grid cells for each point.
  int num_spillover_steps_xy_;
  int num_spillover_steps_z_;
};

#endif /* DENSITY_GRID_TRACKER_H_ */

