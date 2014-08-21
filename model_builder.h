/*
 * aligner.h
 *
 *  Created on: Nov 20, 2011
 *      Author: davheld
 */

#ifndef MOVING_SYNCHRONIZER_MODEL_BUILDER_H_
#define MOVING_SYNCHRONIZER_MODEL_BUILDER_H_

#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "motion_model.h"
#include "precision_tracker.h"

class PointsInterpolater;

class ModelBuilder {
public:
  ModelBuilder(const bool visualize);
  virtual ~ModelBuilder();

  void clear();

  void addPoints(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const double& vlfTimestamp,
      const Eigen::Vector3f velo_centroid,
      Eigen::Vector3f* estimated_velocity);

private:

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previousModel;

  double curr_vlf_timestamp;
  double prev_vlf_timestamp;

  double vlf_time_stamp_diff_;

  boost::shared_ptr<MotionModel> motion_model_;

  PrecisionTracker precision_tracker_;

};


#endif /* MOVING_SYNCHRONIZER_MODEL_BUILDER_H_ */
