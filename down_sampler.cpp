/*
 * down_sampler.cpp
 *
 *  Created on: May 12, 2014
 *      Author: davheld
 */

#include "down_sampler.h"

namespace {

// Whether to round up or down for deterministic downsampling when computing
// how many points to skip.
const bool kUseCeil = true;

} // namespace

DownSampler::DownSampler() {
  // TODO Auto-generated constructor stub

}

DownSampler::~DownSampler() {
  // TODO Auto-generated destructor stub
}

void DownSampler::downSamplePointsStochastic(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& points,
    const int targetNumPoints,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& downSampledPoints) {
  const size_t num_points = points->size();

  // Check if the points are already sufficiently down-sampled.
  if (targetNumPoints >= num_points * 0.8){
    *downSampledPoints = *points;
    return;
  }

  // Allocate space for the new points.
  downSampledPoints->reserve(targetNumPoints);

  //Just to ensure that we don't end up with 0 points, add 1 point to this
  downSampledPoints->push_back((*points)[0]);

  // Randomly select points with (targetNumPoints / num_points) probability.
  for (size_t i = 1; i < num_points; ++i){
    const pcl::PointXYZRGB& pt = (*points)[i];
    if (rand() % num_points < targetNumPoints){
      downSampledPoints->push_back(pt);
    }
  }
}

void DownSampler::downSamplePointsDeterministic(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& points,
    const int targetNumPoints,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& downSampledPoints) {
  const size_t num_points = points->size();

  // Check if the points are already sufficiently down-sampled.
  if (targetNumPoints >= num_points * 0.8){
    *downSampledPoints = *points;
    return;
  }

  // Select every N points to reach the target number of points.
  int everyN = 0;
  if (kUseCeil) {
    everyN = ceil(static_cast<double>(num_points) /
                  static_cast<double>(targetNumPoints));
  } else {
    everyN = static_cast<double>(num_points) /
        static_cast<double>(targetNumPoints);
  }

  // Allocate space for the new points.
  downSampledPoints->reserve(targetNumPoints);

  //Just to ensure that we don't end up with 0 points, add 1 point to this
  downSampledPoints->push_back((*points)[0]);

  // Select every N points to reach the target number of points.
  for (size_t i = 1; i < num_points; ++i) {
    if (i % everyN == 0){
      const pcl::PointXYZRGB& pt = (*points)[i];
      downSampledPoints->push_back(pt);
    }
  }
}
