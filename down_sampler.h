/*
 * down_sampler.h
 *
 *  Created on: May 12, 2014
 *      Author: davheld
 */

#ifndef DOWN_SAMPLER_H_
#define DOWN_SAMPLER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class DownSampler {
public:
  DownSampler();
  virtual ~DownSampler();

  // Randomly samples points from the input cloud to try to get the output
  // to have as close as possible to targetNumPoints points.
  static void downSamplePointsStochastic(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& points,
      const int targetNumPoints,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& downSampledPoints);

  // Deterministically samples every N points from the input cloud to try to
  // get the output to have as close as possible to targetNumPoints points.
  static void downSamplePointsDeterministic(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& points,
      const int targetNumPoints,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& downSampledPoints);
};

#endif /* DOWN_SAMPLER_H_ */
