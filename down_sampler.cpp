/*
 * down_sampler.cpp
 *
 *  Created on: May 12, 2014
 *      Author: davheld
 */

#include "down_sampler.h"

namespace {

const bool useCeil = true; //getenv("USE_CEIL");

} // namespace

DownSampler::DownSampler() {
  // TODO Auto-generated constructor stub

}

DownSampler::~DownSampler() {
  // TODO Auto-generated destructor stub
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > DownSampler::downSamplePointsStochastic(
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > points,
    const int& targetNumPoints) {
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > downSampledPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

  if (targetNumPoints >= points->size() * 0.8){
    //printf("Target num points: %d, num points in cloud: %zu - close enough\n", targetNumPoints, points->size());
    return points;
  }

  //int n = 0;
  //int everyN = static_cast<double>(points->size()) / targetNumPoints;
  //PRINT_INFO("Taking every %d points", everyN);

  //Just to ensure that we don't end up with 0 points, add 1 point to this
  downSampledPoints->push_back(points->at(0));

  int numPoints = points->size();

  for (size_t i = 1; i < points->size(); i++){
    //extract the point so we can compute the distance to the nearest neighbors
    pcl::PointXYZRGB pt = points->at(i);
    //if (n % everyN == 0){
    if (rand() % numPoints < targetNumPoints){
      downSampledPoints->push_back(pt);
    }
    //n++;
  }

  //printf("Target was %d points, down-sampled from %zu to %zu points\n", targetNumPoints, points->size(), downSampledPoints->size());

  return downSampledPoints;
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > DownSampler::downSamplePointsDeterministic(
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > points,
    const int& targetNumPoints) {
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > downSampledPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

  const size_t num_points = points->size();

  if (targetNumPoints >= num_points * 0.8){
    //printf("Target num points: %d, num points in cloud: %zu - close enough\n", targetNumPoints, points->size());
    return points;
  }

  //PRINT_WARN("Use_ceil: %d", useCeil);

  int everyN = 0;
  if (useCeil) {
    everyN = ceil(static_cast<double>(num_points) / static_cast<double>(targetNumPoints));
  } else {
    everyN = static_cast<double>(num_points) / static_cast<double>(targetNumPoints);
  }

  //PRINT_INFO("Taking every %d points", everyN);

  // Allocate space.
  downSampledPoints->reserve(targetNumPoints);

  //Just to ensure that we don't end up with 0 points, add 1 point to this
  downSampledPoints->push_back(points->at(0));

  //int numPoints = points->size();

  int n = 0;
  for (size_t i = 1; i < num_points; ++i) {
    //extract the point so we can compute the distance to the nearest neighbors
    if (n % everyN == 0){
    //if (rand() % numPoints < targetNumPoints){
      const pcl::PointXYZRGB& pt = (*points)[i];
      downSampledPoints->push_back(pt);
    }
    n++;
  }

  //printf("Target was %d points, down-sampled from %zu to %zu points", targetNumPoints, num_points, downSampledPoints->size());

  return downSampledPoints;
}
