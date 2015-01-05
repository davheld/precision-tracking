/*
 * alignment_evaluator.h
 *
 *  Created on: Aug 18, 2014
 *      Author: davheld
 *
 * Base class for evaluating the probability of different alignments between
 * points in the previous and the current frames.
 *
 */

#ifndef __PRECISION_TRACKING__ALIGNMENT_EVALUATOR_H
#define __PRECISION_TRACKING__ALIGNMENT_EVALUATOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <precision_tracking/motion_model.h>
#include <precision_tracking/scored_transform.h>
#include <precision_tracking/params.h>

namespace precision_tracking {

class AlignmentEvaluator
{
public:
  explicit AlignmentEvaluator(const Params *params);
  virtual ~AlignmentEvaluator();

  virtual void setPrevPoints(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr prev_points);

  // Compute the probability of each of the transforms being the
  // correct alignment of the current points to the previous points.
  virtual void score3DTransforms(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const Eigen::Vector3f& current_points_centroid,
      const double xy_sampling_resolution,
      const double z_sampling_resolution,
      const double sensor_horizontal_resolution,
      const double sensor_vertical_resolution,
      const std::vector<XYZTransform>& transforms,
      const MotionModel& motion_model,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms);

protected:
  virtual void init(const double xy_sampling_resolution,
            const double z_sampling_resolution,
            const double xy_sensor_resolution,
            const double z_sensor_resolution,
            const size_t num_current_points);

  // Get the probability of the translation (x, y, z) applied to the
  // current points.
  virtual double getLogProbability(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const Eigen::Vector3f& current_points_centroid,
      const MotionModel& motion_model,
      const double delta_x, const double delta_y, const double delta_z) = 0;

  const Params *params_;

  // Previous points for alignment.
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr prev_points_;

  // Sampling resolution in our particle space.
  double xy_sampling_resolution_;
  double z_sampling_resolution_;

  // Covariance parameters for the measurement model.
  double sigma_xy_;
  double sigma_z_;

  // Convert the variance to a factor such that
  // exp(-x^2 / 2 sigma^2) = exp(x^2 * exp_factor)
  // where x is the distance.
  double xy_exp_factor_;
  double z_exp_factor_;
  double xyz_exp_factor_;

  // Smoothing factor for the measurement model, so we never assign a point
  // a 0 probability.
  double smoothing_factor_;

  // How much to discount the measurement model, based on dependencies
  // between points.
  double measurement_discount_factor_;
};

} // namespace precision_tracking

#endif // __PRECISION_TRACKING__ALIGNMENT_EVALUATOR_H
