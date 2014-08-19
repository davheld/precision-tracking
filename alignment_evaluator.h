/*
*  Created on: Aug 18, 2014
*      Author: davheld
*/

#ifndef ALIGNMENT_EVALUATOR_H
#define ALIGNMENT_EVALUATOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "motion_model.h"
#include "scored_transform.h"

class AlignmentEvaluator
{
public:
  AlignmentEvaluator();
  virtual ~AlignmentEvaluator();

  virtual void setPrevPoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr prev_points);

  virtual void score3DTransforms(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const Eigen::Vector3f& current_points_centroid,
      const double xy_sampling_resolution,
      const double z_sampling_resolution,
      const double sensor_horizontal_resolution,
      const double sensor_vertical_resolution,
      const std::vector<XYZTransform>& transforms,
      const MotionModel& motion_model,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms) = 0;

protected:
  virtual void init(const double xy_sampling_resolution,
            const double z_sampling_resolution,
            const double sensor_horizontal_resolution,
            const double sensor_vertical_resolution);

  // Previous points for alignment.
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr prev_points_;

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

  double measurement_discount_factor_;
};

#endif // ALIGNMENT_EVALUATOR_H
