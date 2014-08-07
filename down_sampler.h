/*
 * down_sampler.h
 *
 *  Created on: May 12, 2014
 *      Author: davheld
 */

#ifndef DOWN_SAMPLER_H_
#define DOWN_SAMPLER_H_

#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class DownSampler {
public:
  DownSampler();
  virtual ~DownSampler();

  static boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > downSamplePointsStochastic(
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > points,
      const int& targetNumPoints);
  static boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > downSamplePointsDeterministic(
      const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > points,
      const int& targetNumPoints);
};

#endif /* DOWN_SAMPLER_H_ */
