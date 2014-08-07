/*
 * density_grid_tracker.h
 *
 *  Created on: Sep 1, 2013
 *      Author: davheld
 */

#ifndef DENSITY_GRID_TRACKER_H_
#define DENSITY_GRID_TRACKER_H_

#include <utility>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "motion_model.h"
#include "scored_transform.h"
#include "fast_functions.h"


struct ScoredTransform;
struct XYZTransform;

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
      const double xy_stepSize,
      const double z_stepSize,
      const std::pair <double, double>& xRange,
      const std::pair <double, double>& yRange,
      const std::pair <double, double>& zRange,
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
      const Eigen::Vector3f& current_points_centroid,
      const MotionModel& motion_model,
      const double horizontal_distance,
      const double down_sample_factor,
      const double point_ratio,
      ScoredTransforms<ScoredTransformXYZ>* transforms);

  // Score each ofthe xyz transforms.
  void scoreXYZTransforms(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
      const Eigen::Vector3f& current_points_centroid,
      const double xy_stepSize,
      const double z_stepSize,
      const std::vector<XYZTransform>& transforms,
      const MotionModel& motion_model,
      const double horizontal_distance,
      const double down_sample_factor,
      const double point_ratio,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms);

  // Create a list of candidate xyz transforms.
  static void createCandidateXYZTransforms(
      const double xy_step_size,
      const double z_step_size,
      const std::pair <double, double>& xRange,
      const std::pair <double, double>& yRange,
      const std::pair <double, double>& zRange,
      std::vector<XYZTransform>* transforms);

private:
  DensityGridTracker();
  DensityGridTracker(DensityGridTracker const&);              // Don't Implement.
  void operator=(DensityGridTracker const&); // Don't implement

  // Determine the size and minimum density for the density grid.
  void computeDensityGridSize(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
      const double xy_stepSize,
      const double z_stepSize,
      const double point_ratio,
      const double horizontal_distance,
      const double down_sample_factor);

  // Given a set of points, and the minimum point in the set,
  // Compute a 3D density grid.
  void computeDensityGrid(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points);

  // Get the score of this transform, using the density grid.
  double getLogProbability(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const pcl::PointXYZRGB& minPt,
      const double xy_gridStep,
      const double z_gridStep,
      const MotionModel& motion_model,
      const double x,
      const double y,
      const double z) const;

  const FastFunctions& fast_functions_;

  std::vector<std::vector<std::vector<double> >  > density_grid_;
  int xSize_;
  int ySize_;
  int zSize_;

  double discount_factor_;

  pcl::PointXYZRGB minPt_;

  double minDensity;
  double xy_grid_step;
  double z_grid_step;
  pcl::PointXYZRGB minPt;
  double spillover_sigma_xy;
  double spillover_sigma_z;
  int num_spillover_steps_xy;
  int num_spillover_steps_z;

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

