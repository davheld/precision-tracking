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

#include "transform_helper.h"

using std::max;
using std::min;
using std::set;
using std::vector;
using std::cout;
using namespace Eigen;
using std::endl;
using std::pair;
using std::make_pair;

namespace{

const bool kUseMode = false;

const bool useCentroid = false;

}  // namespace

ModelBuilder::ModelBuilder(
    const bool visualize)
    : previousModel(new pcl::PointCloud<pcl::PointXYZRGB>),
      curr_vlf_timestamp(-1),
      prev_vlf_timestamp(-1)
{
  motion_model_.reset(new MotionModel);
}

ModelBuilder::~ModelBuilder() {
  // TODO Auto-generated destructor stub
}

void ModelBuilder::clear(){
  motion_model_.reset(new MotionModel);
}

void ModelBuilder::addPoints(
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& current_points,
    const double& vlf_timestamp,
    const Eigen::Vector3f velo_centroid,
    Eigen::Vector3f* estimated_velocity) {
  // Do not align if there are no points.
  if (current_points->size() == 0){
    printf("No points - cannot align.\n");
    *estimated_velocity = Eigen::Vector3f::Zero();
    return;
  }

  //save previous vlf_timestamp
  prev_vlf_timestamp = curr_vlf_timestamp;
  curr_vlf_timestamp = vlf_timestamp;

  if (!previousModel->empty()) {

    	double vlf_time_stamp_diff = curr_vlf_timestamp - prev_vlf_timestamp;
    	vlf_time_stamp_diff_ = vlf_time_stamp_diff;

      // Propogate the motion model forward to estimate the new position.
      motion_model_->propagate(vlf_time_stamp_diff);

			Eigen::Affine3f full_alignment_to_prev;

      // All alignments from which to form a probability distribution.
      ScoredTransforms<ScoredTransformXYZ> scored_transforms;

      // Get the distance to the tracked object.
      //Eigen::Vector4d velo_centroid = frame_helper.getVeloCentroid();
      const double horizontal_distance =
          sqrt(pow(velo_centroid(0), 2) + pow(velo_centroid(1), 2));

			if (useCentroid) {

			  Eigen::Vector3f newCentroid;
			  PrecisionTracker::computeCentroid(current_points, &newCentroid);

			  Eigen::Vector3f oldCentroid;
        PrecisionTracker::computeCentroid(previousModel, &oldCentroid);

			  const Eigen::Vector3f& centroidDiff =  oldCentroid - newCentroid;

        motion_model_->addCentroidDiff(centroidDiff, vlf_time_stamp_diff);

        const Eigen::Vector3f& mean_displacement = motion_model_->mean_displacement();

        full_alignment_to_prev = Eigen::Translation<float, 3>(mean_displacement);

      }
      else {
        // Align.
        precision_tracker_.track(current_points, previousModel,
            horizontal_distance, *motion_model_, &scored_transforms);

        motion_model_->addTransformsWeightedGaussian(scored_transforms,
                                                    vlf_time_stamp_diff);
			}
			Eigen::Vector3f centroid;
			PrecisionTracker::computeCentroid(current_points, &centroid);
			Eigen::Vector4d centroid_4d;
			centroid_4d(0) = centroid(0);
      centroid_4d(1) = centroid(1);
      centroid_4d(2) = centroid(2);
      centroid_4d(3) = 1;
			Eigen::Affine3f real_transform_to_prev =
			    TransformHelper::computeRealTransform(
			        full_alignment_to_prev,
			        centroid_4d);

        Eigen::Vector3f mean_displacement = motion_model_->mean_displacement();
				Eigen::Affine3f mean_translation; mean_translation = Eigen::Translation3f(mean_displacement);

        Eigen::Vector3f mean_velocity = motion_model_->get_mean_velocity();

				Eigen::Affine3f estimated_full_alignment_to_prev;

				if (kUseMode) {
          //*estimated_velocity = real_transform_to_prev.matrix() / vlf_time_stamp_diff_;
				  estimated_full_alignment_to_prev = full_alignment_to_prev;
				} else {
          *estimated_velocity = mean_velocity;
          estimated_full_alignment_to_prev = mean_translation;
				}

  } else {
    // No previous points - just creating initial model.
    *estimated_velocity = Eigen::Vector3f::Zero();
  }

  *previousModel = *current_points;
}
