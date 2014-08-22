/*
 * tracker.h
 *
 *  Created on: Nov 20, 2011
 *      Author: davheld
 */

#ifndef TRACKER_H_
#define TRACKER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "motion_model.h"
#include "precision_tracker.h"

class Tracker {
public:
  Tracker();
  virtual ~Tracker();

  void clear();

  void addPoints(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const double timestamp,
      const Eigen::Vector3f& centroid,
      Eigen::Vector3f* estimated_velocity);

private:

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr previousModel_;
  double prev_timestamp_;

  boost::shared_ptr<MotionModel> motion_model_;
  PrecisionTracker precision_tracker_;
};


#endif /* TRACKER_H_ */
