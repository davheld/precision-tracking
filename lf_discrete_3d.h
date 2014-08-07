/*
 * LFDiscrete3d.h
 *
 *  Created on: May 1, 2014
 *      Author: davheld
 */

#ifndef LFDISCRETE3D_H_
#define LFDISCRETE3D_H_

//#define PCL_NO_PRECOMPILE

#include <boost/shared_ptr.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/octree/octree_search.h>

#include "motion_model.h"
#include "density_grid_tracker.h"

// Singleton Class.
class LFDiscrete3d {
public:
  virtual ~LFDiscrete3d();

  static LFDiscrete3d& getInstance()
  {
      static LFDiscrete3d instance;
      return instance;
  }

  void setPrevPoints(
      const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > prev_points);

  // Estimate the amount that a point cloud has moved.
  // Inputs:
  // A translation step size, and a range for each translation value
  // (min value, max value) in meters.
  // A rotation step size, and a range for each rotation value
  // (min value, max value) in meters.
  void track(
      const double& xy_stepSize,
      const double& z_stepSize,
      const std::pair <double, double>& xRange,
      const std::pair <double, double>& yRange,
      const std::pair <double, double>& zRange,
      const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > current_points,
      const Eigen::Vector3f &current_points_centroid,
      const MotionModel& motion_model,
      const double horizontal_distance,
      const double down_sample_factor,
      const double point_ratio,
      ScoredTransforms<ScoredTransformXYZ>* transforms);

  // Score each ofthe xyz transforms.
  void scoreXYZTransforms(
      const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >& current_points,
      const Eigen::Vector3f &current_points_centroid,
      const double xy_stepSize,
      const double z_stepSize,
      const std::vector<XYZTransform>& transforms,
      const MotionModel& motion_model,
      const double horizontal_distance,
      const double down_sample_factor,
      const double point_ratio,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms);

  // Get the likelihood field score of the transform (x,y,z) applied to the
  // current points.
  double getLogProbability(
      const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >& current_points,
      const MotionModel& motion_model,
      const double x,
      const double y,
      const double z);

private:
  LFDiscrete3d();
  LFDiscrete3d(LFDiscrete3d const&);              // Don't Implement.
  void operator=(LFDiscrete3d const&); // Don't implement

  void init(const double xy_grid_step,
          const double z_grid_step,
          const double horizontal_distance,
          const double down_sample_factor);

  double get_log_prob(
      const int x_index, const int y_index, const int z_index,
      const double x_vel, const double y_vel, const double z_vel,
      const pcl::PointXYZRGB& pt);

  int get_index(const int x, const int y, const int z) const {
    return z * xySize_ + y * xSize_ + x;
  }

  double computeColorProb(const pcl::PointXYZRGB& prev_pt,
      const pcl::PointXYZRGB& pt, const double point_match_prob_spatial_i) const;

  // The cached probability values for each cell.
  //std::vector<std::vector<std::vector<double> > > lf_cache_3D_prob_;
  std::vector<double> lf_cache_3D_log_prob_;

  // Whether each cell contains a cached probability value.
  //std::vector<std::vector<std::vector<bool> > > lf_cache_3D_cached_;
  std::vector<bool> lf_cache_3D_cached_;

  pcl::KdTreeFLANN<pcl::PointXYZRGB> searchTree_;
  //pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> searchTree_;
  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > prev_points_;

  // The spacing of the discretized grid.
  double xy_grid_step_;
  double z_grid_step_;

  // The location of the minimum point of the discretized grid.
  pcl::PointXYZRGB maxPt_;
  pcl::PointXYZRGB minPt_;

  pcl::PointXYZRGB maxPt_orig_;
  pcl::PointXYZRGB minPt_orig_;

  // The location of the center of the lowest grid cell.
  pcl::PointXYZRGB min_center_pt_;

  // Pre-compute probabilities for different distances.
  //std::vector<double> prob_spatials_;
  //double default_val_;

  double xy_exp_factor_;
  //double z_exp_factor_;
  double min_density_;

  double search_radius_;

  // The number of nearest neighbors to find.
  const int max_nn_;

  // Vector to store the result of the nearest neighbor search.
  //TODO - see if we should use maxSize for the size of these vectors - note that this will cause
  // the KD-tree to return all points within the neighborhood and ignore max_nn
  std::vector<int> nn_indices_;
  std::vector<float> nn_sq_dists_;
  //int nn_index_;
  //float nn_sq_dist_;

  const FastFunctions& fast_functions_;

  // The size of the cache that is being used for this object.  Values
  // outside of this range are ignored.
  int xSize_;
  int ySize_;
  int zSize_;
  int xySize_;

  pcl::PointXYZRGB pt_;
  double z_range_;

  // Color parameters.
  double color_exp_factor1_;
  double color_exp_factor2_;
  double prob_color_match_;


};

#endif /* LFDISCRETE3D_H_ */
