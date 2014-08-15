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


struct ScoredTransform;
struct XYZTransform;
class MotionModel;

// Singleton Class.
class DensityGridTracker {
public:
  virtual ~DensityGridTracker();

  static DensityGridTracker& getInstance()
  {
      static DensityGridTracker instance;
      return instance;
  }

  // Estimate the amount that a point cloud has moved.
  // Inputs:
  // A translation step size, and a range for each translation value
  // (min value, max value) in meters.
  // A rotation step size, and a range for each rotation value
  // (min value, max value) in meters.
  void track(
      const double xy_sampling_resolution,
      const double z_sampling_resolution,
      const std::pair <double, double>& xRange,
      const std::pair <double, double>& yRange,
      const std::pair <double, double>& zRange,
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
      const Eigen::Vector3f& current_points_centroid,
      const MotionModel& motion_model,
      const double xy_sensor_resolution,
      const double z_sensor_resolution,
      ScoredTransforms<ScoredTransformXYZ>* transforms);

  // Score each ofthe xyz transforms.
  void scoreXYZTransforms(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
      const double xy_sampling_resolution,
      const double z_sampling_resolution,
      const std::vector<XYZTransform>& transforms,
      const MotionModel& motion_model,
      const double xy_sensor_resolution,
      const double z_sensor_resolution,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms);

  // Create a list of candidate xyz transforms.
  static void createCandidateXYZTransforms(
      const double xy_sampling_resolution,
      const double z_sampling_resolution,
      const std::pair <double, double>& xRange,
      const std::pair <double, double>& yRange,
      const std::pair <double, double>& zRange,
      std::vector<XYZTransform>* transforms);

private:
  DensityGridTracker();
  DensityGridTracker(DensityGridTracker const&); // Don't Implement.
  void operator=(DensityGridTracker const&); // Don't implement

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

  // The minimum probability density in the density grid.
  double min_density_;

  // The step size of the density grid.
  double xy_grid_step_;
  double z_grid_step_;

  // The minimum point of the previous set of points used for tracking.
  pcl::PointXYZRGB min_pt_;

  // The variance of the points for alignment.
  double sigma_xy_;
  double sigma_z_;

  // In our discrete grid, we want to compute the Gaussian probability
  // for this many grid cells away from each point.  As we get farther
  // away from the point, the probability of the Guassian goes to 0
  // so we only need to compute the probability at a limited number
  // of grid cells for each point.
  int num_spillover_steps_xy_;
  int num_spillover_steps_z_;
};


// A pure translation.
struct XYZTransform {
public:
  double x, y, z;
  double volume;
  XYZTransform(
      const double& x,
      const double& y,
      const double& z,
      const double& volume)
    :x(x),
     y(y),
     z(z),
     volume(volume)
  {  }
};

#endif /* DENSITY_GRID_TRACKER_H_ */

