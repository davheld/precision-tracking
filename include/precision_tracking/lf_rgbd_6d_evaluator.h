/*
 * lf_rgbd_6d_evaluator.h
 *
 *  Created on: May 1, 2014
 *      Author: davheld
 *
 * Using the likelihood field model from
 * Probabilistic Robotics, Thrun, et al, 2005.
 * to evaluate the probability of a given set of alignments.
 * This evaluator handles 6D transforms (including rotations)
 * and can make use of color information.
 *
 * Parameters are taken based on the latent surface model from RSS 2014,
 * Held, et al.
 *
 */

#ifndef __PRECISION_TRACKING__LF_RGBD_6D_EVALUATOR_H_
#define __PRECISION_TRACKING__LF_RGBD_6D_EVALUATOR_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <precision_tracking/motion_model.h>
#include <precision_tracking/scored_transform.h>
#include <precision_tracking/alignment_evaluator.h>

namespace precision_tracking {

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

class LF_RGBD_6D_Evaluator : public AlignmentEvaluator {
public:
  explicit LF_RGBD_6D_Evaluator (const Params *params);
  virtual ~LF_RGBD_6D_Evaluator();

  void setPrevPoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr prev_points);

  double getPointProbability(
      const pcl::PointXYZRGB& point);

  // Score each ofthe xyz transforms.
  void score6DTransforms(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& current_points,
      const Eigen::Vector3f& current_points_centroid,
      const double xy_sampling_resolution,
      const double z_sampling_resolution,
      const double sensor_horizontal_resolution,
      const double sensor_vertical_resolution,
      const std::vector<Transform6D>& transforms,
      const MotionModel& motion_model,
      ScoredTransforms<ScoredTransform6D>* scored_transforms);

private:
  double getLogProbability(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const Eigen::Vector3f& current_points_centroid,
      const MotionModel& motion_model,
      const double x, const double y, const double z);

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
            const double sensor_vertical_resolution,
            const size_t num_current_points);

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

  // Whether to use color in the measurement model.
  bool use_color_;

  // Color parameters.
  double color_exp_factor1_;
  double color_exp_factor2_;
  double prob_color_match_;
};

} // namespace precision_tracking

#endif /* __PRECISION_TRACKING__LF_RGBD_6D_EVALUATOR_H_ */
