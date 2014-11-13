/*
 * tracker.h
 *
 *  Created on: Nov 20, 2011
 *      Author: davheld
 */

#ifndef __PRECISION_TRACKING__TRACKER_H_
#define __PRECISION_TRACKING__TRACKER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <precision_tracking/motion_model.h>
#include <precision_tracking/precision_tracker.h>

namespace precision_tracking {

class Tracker {
public:
  // Default constructor, uses precision tracker with no color and returns
  // the mean of the distribution.
  Tracker();

  // Constructor to choose options for the type of tracking.
  Tracker(const bool use_precision_tracker,
          const bool use_color,
          const bool use_mean);

  virtual ~Tracker();

  void clear();

  void addPoints(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const double timestamp,
      const double sensor_horizontal_resolution,
      const double sensor_vertical_resolution,
      Eigen::Vector3f* estimated_velocity,
      double* alignment_probability);

  const Eigen::Matrix3d get_covariance_velocity() const {
    return motion_model_->get_covariance_velocity();
  }

    Eigen::Vector3d get_mean_delta_position() const {
        return motion_model_->get_mean_delta_position();
  }

  const Eigen::Matrix3d get_covariance_delta_position() const {
    return motion_model_->get_covariance_delta_position();
  }

  const MotionModel & get_motion_model() const {
    return *motion_model_;
  }

private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr previousModel_;
  double prev_timestamp_;

  boost::shared_ptr<MotionModel> motion_model_;
  boost::shared_ptr<PrecisionTracker> precision_tracker_;

  // Whether to use our precision tracker (accurate) or the centroid-based
  // Kalman filter baseline (fast but not accurate).
  bool use_precision_tracker_;

  // Whether to use color - note that using color will make the tracker
  // very slow!
  bool use_color_;

  // Whether to return the mean or mode of the distribution.  The mean
  // typically is more accurate because it accounts for the uncertainty
  // of the distribution, and because it can be computed at a finer resolution
  // than our sampling resolution.
  bool use_mean_;
};

} // namespace precision_tracking

#endif /* __PRECISION_TRACKING__TRACKER_H_ */
