/*
 * LFDiscrete3d.h
 *
 *  Created on: May 1, 2014
 *      Author: davheld
 */

#ifndef LF_RGBD_6D_H_
#define LF_RGBD_6D_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "motion_model.h"
#include "scored_transform.h"
#include "density_grid_tracker.h"
#include "alignment_evaluator.h"

// A 6D transform
struct Transform6D {
public:
  Transform6D(
      const double x,
      const double y,
      const double z,
      const double roll,
      const double pitch,
      const double yaw,
      const double volume)
    :x(x),
     y(y),
     z(z),
     roll(roll),
     pitch(pitch),
     yaw(yaw),
     volume(volume)
  {  }

  double x, y, z;
  double roll, pitch, yaw;
  double volume;
};

class LF_RGBD_6D : public AlignmentEvaluator {
public:
  LF_RGBD_6D ();
  virtual ~LF_RGBD_6D();

  void setPrevPoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr prev_points);

  void score3DTransforms(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const Eigen::Vector3f& current_points_centroid,
      const double xy_sampling_resolution,
      const double z_sampling_resolution,
      const double sensor_horizontal_resolution,
      const double sensor_vertical_resolution,
      const std::vector<XYZTransform>& transforms,
      const MotionModel& motion_model,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms);

  // Score each ofthe xyz transforms.
  void score6DTransforms(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& current_points,
      const Eigen::Vector3f &current_points_centroid,
      const double xy_sampling_resolution,
      const double z_sampling_resolution,
      const double sensor_horizontal_resolution,
      const double sensor_vertical_resolution,
      const std::vector<Transform6D>& transforms,
      const MotionModel& motion_model,
      ScoredTransforms<ScoredTransform6D>* scored_transforms);

private:
  // Get the likelihood field score of the transform applied to the
  // current points.
  double getLogProbability(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const Eigen::Vector3f& current_points_centroid,
      const MotionModel& motion_model,
      const double delta_x,
      const double delta_y,
      const double delta_z,
      const double roll,
      const double pitch,
      const double yaw);

  void init(const double xy_sampling_resolution,
            const double z_sampling_resolution,
            const double sensor_horizontal_resolution,
            const double sensor_vertical_resolution);

  double get_log_prob(const pcl::PointXYZRGB& current_pt);

  double computeColorProb(const pcl::PointXYZRGB& prev_pt,
      const pcl::PointXYZRGB& pt, const double point_match_prob_spatial_i) const;

  void makeEigenRotation(
      const double roll, const double pitch, const double yaw,
      Eigen::Quaternion<float>* rotation) const;

  void makeEigenTransform(
      const Eigen::Vector3f& centroid,
      const double delta_x, const double delta_y, const double delta_z,
      const double roll, const double pitch, const double yaw,
      Eigen::Affine3f* transform) const;

  // Search tree from the previous points.
  pcl::KdTreeFLANN<pcl::PointXYZRGB> searchTree_;

  // The number of nearest neighbors to find.
  const int max_nn_;

  // Vector to store the result of the nearest neighbor search.
  std::vector<int> nn_indices_;
  std::vector<float> nn_sq_dists_;

  // Color parameters.
  double color_exp_factor1_;
  double color_exp_factor2_;
  double prob_color_match_;
};

#endif /* LF_RGBD_6D_H_ */
