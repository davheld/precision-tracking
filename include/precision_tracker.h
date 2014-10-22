/*
 * precision_tracker.h
 *
 *  Created on: May 12, 2014
 *      Author: davheld
 */

#ifndef PRECISION_TRACKER_H_
#define PRECISION_TRACKER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "scored_transform.h"
#include "motion_model.h"
#include "adh_tracker3d.h"
#include "down_sampler.h"

namespace precision_tracking {

class PrecisionTracker {
public:
  // Whether to include color probabilities when performing the alignment.
  // Using color is more accurate but much slower.
  PrecisionTracker(const bool use_color);

  // Default constructor - does not use color. This is slightly less
  // accurate but much faster than the version with color.
  PrecisionTracker();
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

  ADHTracker3d adh_tracker3d_;
  boost::shared_ptr<AlignmentEvaluator> alignment_evaluator_;
  DownSampler down_sampler_;

  // Whether to include color probabilities when performing the alignment.
  // Using color is more accurate but much slower.
  bool use_color_;
};

} // namespace precision_tracking

#endif /* PRECISION_TRACKER_H_ */
