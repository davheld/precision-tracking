/*
 * aligner.cpp
 *
 *  Created on: Nov 20, 2011
 *      Author: davheld
 */

#include "model_builder.h"

#include <algorithm>
#include <set>
#include <vector>
#include <iostream>
#include <utility>
#include <cmath>

#include <Eigen/Dense>

#include <pcl/common/common.h>

using std::max;
using std::min;
using std::set;
using std::vector;
using std::cout;
using std::endl;
using std::pair;
using std::make_pair;

namespace{

const bool kUseMode = false;

const bool useCentroid = true;

}  // namespace

ModelBuilder::ModelBuilder(
    const bool visualize)
    : previousModel_(new pcl::PointCloud<pcl::PointXYZRGB>),
      curr_vlf_timestamp_(-1),
      prev_vlf_timestamp_(-1)
{
  motion_model_.reset(new MotionModel);
}

ModelBuilder::~ModelBuilder() {
  // TODO Auto-generated destructor stub
}

void ModelBuilder::clear(){
  motion_model_.reset(new MotionModel);
  previousModel_->clear();
}

void ModelBuilder::addPoints(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
    const double& timestamp,
    const Eigen::Vector3f velo_centroid,
    Eigen::Vector3f* estimated_velocity) {
  // Do not align if there are no points.
  if (current_points->size() == 0){
    printf("No points - cannot align.\n");
    *estimated_velocity = Eigen::Vector3f::Zero();
    return;
  }

  //save previous vlf_timestamp
  prev_vlf_timestamp_ = curr_vlf_timestamp_;
  curr_vlf_timestamp_ = timestamp;

  if (previousModel_->empty()) {
    // No previous points - just creating initial model.
    *estimated_velocity = Eigen::Vector3f::Zero();
  } else {
    const double vlf_time_stamp_diff = curr_vlf_timestamp_ - prev_vlf_timestamp_;

    // Propogate the motion model forward to estimate the new position.
    motion_model_->propagate(vlf_time_stamp_diff);

    // Get the distance to the tracked object.
    //Eigen::Vector4d velo_centroid = frame_helper.getVeloCentroid();
    const double horizontal_distance =
        sqrt(pow(velo_centroid(0), 2) + pow(velo_centroid(1), 2));

    if (useCentroid) {

      Eigen::Vector3f new_centroid;
      PrecisionTracker::computeCentroid(current_points, &new_centroid);

      Eigen::Vector3f old_centroid;
      PrecisionTracker::computeCentroid(previousModel_, &old_centroid);

      const Eigen::Vector3f& centroidDiff =  old_centroid - new_centroid;

      motion_model_->addCentroidDiff(centroidDiff, vlf_time_stamp_diff);

      Eigen::Vector3f mean_velocity = motion_model_->get_mean_velocity();
      *estimated_velocity = mean_velocity;
    }
    else {
      // Align.
      ScoredTransforms<ScoredTransformXYZ> scored_transforms;
      precision_tracker_.track(current_points, previousModel_,
          horizontal_distance, *motion_model_, &scored_transforms);

      motion_model_->addTransformsWeightedGaussian(scored_transforms,
                                                  vlf_time_stamp_diff);
      if (kUseMode) {
        ScoredTransformXYZ best_transform;
        scored_transforms.findBest(&best_transform);

        Eigen::Vector3f best_displacement;
        best_transform.getEigen(&best_displacement);

        *estimated_velocity = best_displacement / vlf_time_stamp_diff;
      } else {
        Eigen::Vector3f mean_velocity = motion_model_->get_mean_velocity();
        *estimated_velocity = mean_velocity;
      }
    }
  }
  *previousModel_ = *current_points;
}
