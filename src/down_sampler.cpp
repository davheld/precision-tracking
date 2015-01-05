/*
 * down_sampler.cpp
 *
 *  Created on: May 12, 2014
 *      Author: davheld
 *
 */

#include <precision_tracking/down_sampler.h>

namespace precision_tracking {


DownSampler::DownSampler(const bool stochastic, const Params *params)
  : params_(params),
    stochastic_(stochastic)
{
}

DownSampler::~DownSampler()
{
  // TODO Auto-generated destructor stub
}

void DownSampler::downSamplePoints(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& points,
    const int target_num_points,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& down_sampled_points) const
{
  if (stochastic_) {
    downSamplePointsStochastic(points, target_num_points, down_sampled_points);
  } else {
    downSamplePointsDeterministic(points, target_num_points, down_sampled_points, params_->kUseCeil);
  }
}


void DownSampler::downSamplePointsStochastic(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& points,
    const int target_num_points,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& down_sampled_points)
{
  const size_t num_points = points->size();

  // Check if the points are already sufficiently down-sampled.
  if (target_num_points >= num_points * 0.8){
    *down_sampled_points = *points;
    return;
  }

  // Allocate space for the new points.
  down_sampled_points->reserve(target_num_points);

  //Just to ensure that we don't end up with 0 points, add 1 point to this
  down_sampled_points->push_back((*points)[0]);

  // Randomly select points with (targetNumPoints / num_points) probability.
  for (size_t i = 1; i < num_points; ++i){
    const pcl::PointXYZRGB& pt = (*points)[i];
    if (rand() % num_points < target_num_points){
      down_sampled_points->push_back(pt);
    }
  }
}

void DownSampler::downSamplePointsDeterministic(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& points,
    const int target_num_points,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& down_sampled_points,
    const bool use_ceil)
{
  const size_t num_points = points->size();

  // Check if the points are already sufficiently down-sampled.
  if (target_num_points >= num_points * 0.8){
    *down_sampled_points = *points;
    return;
  }

  // Select every N points to reach the target number of points.
  int everyN = 0;
  if (use_ceil) {
    everyN = ceil(static_cast<double>(num_points) /
                  static_cast<double>(target_num_points));
  } else {
    everyN = static_cast<double>(num_points) /
        static_cast<double>(target_num_points);
  }

  // Allocate space for the new points.
  down_sampled_points->reserve(target_num_points);

  //Just to ensure that we don't end up with 0 points, add 1 point to this
  down_sampled_points->push_back((*points)[0]);

  // Select every N points to reach the target number of points.
  for (size_t i = 1; i < num_points; ++i) {
    if (i % everyN == 0){
      const pcl::PointXYZRGB& pt = (*points)[i];
      down_sampled_points->push_back(pt);
    }
  }
}

} // namespace precision_tracking
