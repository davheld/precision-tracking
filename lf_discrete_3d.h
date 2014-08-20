/*
 * LFDiscrete3d.h
 *
 *  Created on: May 1, 2014
 *      Author: davheld
 */

#ifndef LFDISCRETE3D_H_
#define LFDISCRETE3D_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "motion_model.h"
#include "density_grid_tracker.h"

class LFDiscrete3d : public AlignmentEvaluator {
public:
  LFDiscrete3d();
  virtual ~LFDiscrete3d();

  void setPrevPoints(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr prev_points);

private:
  void init(const double xy_grid_step,
          const double z_grid_step,
          const double horizontal_distance,
          const double down_sample_factor,
          const size_t num_current_points);

  // Get the probability of the translation (x, y, z) applied to the
  // current points.
  double getLogProbability(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const Eigen::Vector3f& current_points_centroid,
      const MotionModel& motion_model,
      const double delta_x, const double delta_y, const double delta_z);

  double get_log_prob(
      const int x_index, const int y_index, const int z_index);

  int get_index(const int x, const int y, const int z) const {
    return z * xySize_ + y * xSize_ + x;
  }

  // The cached probability values for each cell.
  //std::vector<std::vector<std::vector<double> > > lf_cache_3D_prob_;
  std::vector<double> lf_cache_3D_log_prob_;

  // Whether each cell contains a cached probability value.
  //std::vector<std::vector<std::vector<bool> > > lf_cache_3D_cached_;
  std::vector<bool> lf_cache_3D_cached_;

  pcl::KdTreeFLANN<pcl::PointXYZRGB> searchTree_;

  // The spacing of the discretized grid.
  double xy_grid_step_;
  double z_grid_step_;

  // The location of the minimum point of the discretized grid.
  pcl::PointXYZRGB min_pt_;

  // The location of the center of the lowest grid cell.
  pcl::PointXYZRGB min_center_pt_;

  double search_radius_;

  // The number of nearest neighbors to find.
  const int max_nn_;

  // Vector to store the result of the nearest neighbor search.
  //TODO - see if we should use maxSize for the size of these vectors - note that this will cause
  // the KD-tree to return all points within the neighborhood and ignore max_nn
  std::vector<int> nn_indices_;
  std::vector<float> nn_sq_dists_;

  const FastFunctions& fast_functions_;

  // The size of the cache that is being used for this object.  Values
  // outside of this range are ignored.
  int xSize_;
  int ySize_;
  int zSize_;
  int xySize_;

  pcl::PointXYZRGB pt_;
  double z_range_;
};

#endif /* LFDISCRETE3D_H_ */
