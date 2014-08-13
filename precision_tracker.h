/*
 * precision_tracker.h
 *
 *  Created on: May 12, 2014
 *      Author: davheld
 */

#ifndef PRECISION_TRACKER_H_
#define PRECISION_TRACKER_H_

#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "scored_transform.h"
#include "motion_model.h"
//#include "ap_tracker.h"
#include "ap_tracker3d.h"
//#include "density_grid_tracker.h"
#include "density_grid_tracker.h"

class PrecisionTracker {
public:
  PrecisionTracker();
  virtual ~PrecisionTracker();

  void track(
      const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > current_points,
      const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previousModel,
      const double horizontal_distance,
      const MotionModel& motion_model,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms);

  static void computeCentroid(
      const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > points,
      Eigen::Vector3f* centroid);

  static Eigen::Matrix4f estimateAlignmentCentroidDiff(
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > interpolatedColoredPointsPtr,
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previousModelPtr);

private:
  void findBestLocation(
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > current_points,
      const double max_xy_stepSize,
      const double max_z_stepSize,
      std::pair <double, double> xRange,
      std::pair <double, double> yRange,
      std::pair <double, double> zRange,
      const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > prev_points,
      const Eigen::Vector3f &current_points_centroid,
      const MotionModel& motion_model,
      const double down_sample_factor_prev,
      const double point_ratio,
      const double horizontal_distance,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms);

  //APTracker ap_tracker2d_;
  APTracker3d ap_tracker3d_;
  //const FastFunctions& fast_functions_;

  //LFDiscrete3d& lf_discrete_3d_;
  //DensityGridTracker density_grid_tracker2d_;
  //DensityGridTracker3d& density_grid_tracker3d_;
  //NNTracker3d nn_tracker_;

};

#endif /* PRECISION_TRACKER_H_ */
