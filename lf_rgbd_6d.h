/*
 * LFDiscrete3d.h
 *
 *  Created on: May 1, 2014
 *      Author: davheld
 */

#ifndef LFDISCRETE3D_H_
#define LFDISCRETE3D_H_

#include <boost/shared_ptr.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "motion_model.h"

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

class LF_RGBD_6D {
public:
  LF_RGBD_6D ();
  virtual ~LF_RGBD_6D();

  void setPrevPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_points);

  // Estimate the amount that a point cloud has moved.
  // Inputs:
  // A translation step size, and a range for each translation value
  // (min value, max value) in meters.
  // A rotation step size, and a range for each rotation value
  // (min value, max value) in meters.
  void track(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_points,
      const Eigen::Vector3f& current_points_centroid,
      const double xy_sampling_resolution,
      const double z_sampling_resolution,
      const double sensor_horizontal_resolution,
      const double sensor_vertical_resolution,
      const double down_sample_factor,
      const std::pair <double, double>& xRange,
      const std::pair <double, double>& yRange,
      const std::pair <double, double>& zRange,
      const std::pair <double, double>& rollRange,
      const std::pair <double, double>& pitchRange,
      const std::pair <double, double>& yawRange,
      const MotionModel& motion_model,
      ScoredTransforms<ScoredTransform6D>* scored_transforms);

  // Score each ofthe xyz transforms.
  void score6DTransforms(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& current_points,
      const Eigen::Vector3f &current_points_centroid,
      const double xy_sampling_resolution,
      const double z_sampling_resolution,
      const double sensor_horizontal_resolution,
      const double sensor_vertical_resolution,
      const double down_sample_factor,
      const std::vector<Transform6D>& transforms,
      const MotionModel& motion_model,
      ScoredTransforms<ScoredTransform6D>* scored_transforms);

private:
  void createCandidateTransforms(
      const double xy_step_size,
      const double z_step_size,
      const double roll_step_size,
      const double pitch_step_size,
      const double yaw_step_size,
      std::pair <double, double>& xRange,
      std::pair <double, double>& yRange,
      std::pair <double, double>& zRange_orig,
      const std::pair <double, double>& rollRange,
      const std::pair <double, double>& pitchRange,
      const std::pair <double, double>& yawRange,
      std::vector<Transform6D>* transforms);

  // Get the likelihood field score of the transform applied to the
  // current points.
  double getLogProbability(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& current_points,
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
            const double sensor_vertical_resolution,
            const double down_sample_factor);

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

  // Previous points for alignment.
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr prev_points_;

  // Search tree from the previous points.
  pcl::KdTreeFLANN<pcl::PointXYZRGB> searchTree_;

  // Sampling resolution in our particle space.
  double xy_sampling_resolution_;
  double z_sampling_resolution_;

  // Covariance parameters for the measurement model.
  double xy_exp_factor_;
  double z_exp_factor_;
  double xyz_exp_factor_;
  bool isotropic_;

  // Smoothing factor for the measurement model.
  double smoothing_factor_;

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

#endif /* LFDISCRETE3D_H_ */
