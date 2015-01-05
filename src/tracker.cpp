/*
 * tracker.cpp
 *
 *  Created on: Nov 20, 2011
 *      Author: davheld
 *
 */

#include <pcl/common/centroid.h>

#include <precision_tracking/tracker.h>


namespace precision_tracking {

Tracker::Tracker(const Params *params)
  : params_(params),
    previousModel_(new pcl::PointCloud<pcl::PointXYZRGB>),
    prev_timestamp_(-1)
{
  motion_model_.reset(new MotionModel(params_));
}

void Tracker::clear()
{
  motion_model_.reset(new MotionModel(params_));
  previousModel_->clear();
}

void Tracker::addPoints(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
    const double current_timestamp,
    const double sensor_horizontal_resolution,
    const double sensor_vertical_resolution,
    Eigen::Vector3f* estimated_velocity)
{
    double alignment_probability;
    addPoints(current_points, current_timestamp, sensor_horizontal_resolution,
              sensor_vertical_resolution, estimated_velocity,
              &alignment_probability);
}


void Tracker::addPoints(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
    const double current_timestamp,
    const double sensor_horizontal_resolution,
    const double sensor_vertical_resolution,
    Eigen::Vector3f* estimated_velocity,
    double* alignment_probability)
{
  // Do not align if there are no points.
  if (current_points->size() == 0){
    printf("No points - cannot align.\n");
    *estimated_velocity = Eigen::Vector3f::Zero();
    return;
  }

  if (previousModel_->empty()) {
    // No previous points - just creating initial model.
    *estimated_velocity = Eigen::Vector3f::Zero();
  } else {
    const double timestamp_diff = current_timestamp - prev_timestamp_;

    // Propogate the motion model forward to estimate the new position.
    motion_model_->propagate(timestamp_diff);

    // Always align the smaller points to the bigger points.
    const bool flip = previousModel_->size() > current_points->size();

    if (precision_tracker_) {
      // Align.
      ScoredTransforms<ScoredTransformXYZ> scored_transforms;
      if (!flip) {
          motion_model_->setFlip(false);
          // Previous points are smaller - align previous points to current.
          precision_tracker_->track(
                previousModel_, current_points, sensor_horizontal_resolution,
                sensor_vertical_resolution, *motion_model_, &scored_transforms);
      } else {
          motion_model_->setFlip(true);

          // Current points are smaller - align current points to previous.
          precision_tracker_->track(
                current_points, previousModel_, sensor_horizontal_resolution,
                sensor_vertical_resolution, *motion_model_, &scored_transforms);
      }


      motion_model_->addTransformsWeightedGaussian(scored_transforms,
                                                  timestamp_diff);
      ScoredTransformXYZ best_transform;
      scored_transforms.findBest(&best_transform, alignment_probability);

      if (params_->useMean) {
        Eigen::Vector3f mean_velocity = motion_model_->get_mean_velocity();
        *estimated_velocity = mean_velocity;
      } else {
        Eigen::Vector3f best_displacement;
        best_transform.getEigen(&best_displacement);

        *estimated_velocity = (flip ? -1 : 1) * best_displacement / timestamp_diff;
      }
    } else {
      // Track using the centroid-based Kalman filter.
      Eigen::Vector4f new_centroid;
      pcl::compute3DCentroid (*current_points, new_centroid);

      Eigen::Vector4f old_centroid;
      pcl::compute3DCentroid (*previousModel_, old_centroid);

      const Eigen::Vector4f& centroidDiff =  new_centroid - old_centroid;

      motion_model_->addCentroidDiff(centroidDiff, timestamp_diff);

      Eigen::Vector3f mean_velocity = motion_model_->get_mean_velocity();
      *estimated_velocity = mean_velocity;
    }
  }

  // Save mdoel and timestamp.
  *previousModel_ = *current_points;
  prev_timestamp_ = current_timestamp;
}

} // namespace precision_tracking
