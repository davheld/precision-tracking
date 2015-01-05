/*
 * precision_tracker.h
 *
 *  Created on: May 12, 2014
 *      Author: davheld
 *
 * Main class for performing the precision tracking.
 *
 */

#ifndef __PRECISION_TRACKING__PRECISION_TRACKER_H_
#define __PRECISION_TRACKING__PRECISION_TRACKER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <precision_tracking/scored_transform.h>
#include <precision_tracking/motion_model.h>
#include <precision_tracking/adh_tracker3d.h>
#include <precision_tracking/down_sampler.h>
#include <precision_tracking/params.h>

namespace precision_tracking {

class PrecisionTracker {
public:
  explicit PrecisionTracker(const Params *params);

  virtual ~PrecisionTracker();

  void track(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& previousModel,
      const double sensor_horizontal_resolution,
      const double sensor_vertical_resolution,
      const MotionModel& motion_model,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms);

private:  
  void estimateRange(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
      std::pair <double, double>* xRange,
      std::pair <double, double>* yRange,
      std::pair <double, double>* zRange) const;

  Eigen::Matrix4f estimateAlignmentCentroidDiff(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& curr_points,
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points) const;

  const Params *params_;

  ADHTracker3d adh_tracker3d_;
  boost::shared_ptr<AlignmentEvaluator> alignment_evaluator_;
  DownSampler down_sampler_;
};

} // namespace precision_tracking

#endif /* __PRECISION_TRACKING__PRECISION_TRACKER_H_ */
