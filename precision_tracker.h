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

class PrecisionTracker {
public:
  PrecisionTracker();
  virtual ~PrecisionTracker();

  void track(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& previousModel,
      const double horizontal_distance,
      const MotionModel& motion_model,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms);

  static void computeCentroid(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& points,
      Eigen::Vector3f* centroid);
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
};

#endif /* PRECISION_TRACKER_H_ */
