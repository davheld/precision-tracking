/*
 * tracker.h
 *
 *  Created on: Nov 20, 2011
 *      Author: davheld
 *
 * Main class for tracking; updates the object's motion model and calls
 * the appropriate tracker (precision tracker or otherwise).
 *
 */

#ifndef __PRECISION_TRACKING__TRACKER_H_
#define __PRECISION_TRACKING__TRACKER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <precision_tracking/motion_model.h>
#include <precision_tracking/precision_tracker.h>
#include <precision_tracking/params.h>

namespace precision_tracking {

class Tracker {
public:
  /// Default constructor. Does not allocate a precision tracker.
  /// Call setPrecisionTracker if you need one.
  explicit Tracker(const Params *params);

  void clear();

  // Estimates the velocity of an object.  Call this function each
  // time the object is observed.
  void addPoints(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const double current_timestamp,
      const double sensor_horizontal_resolution,
      const double sensor_vertical_resolution,
      Eigen::Vector3f* estimated_velocity);

  // Same as above, but also returns the maximum alignment probability.
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

  void setPrecisionTracker(boost::shared_ptr<PrecisionTracker> precision_tracker) {
    precision_tracker_ = precision_tracker;
  }

private:
  const Params *params_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr previousModel_;
  double prev_timestamp_;

  boost::shared_ptr<MotionModel> motion_model_;
  boost::shared_ptr<PrecisionTracker> precision_tracker_;
};

} // namespace precision_tracking

#endif /* __PRECISION_TRACKING__TRACKER_H_ */
