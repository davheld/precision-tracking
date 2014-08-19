/*
*  Created on: Aug 18, 2014
*      Author: davheld
*/


#include "alignment_evaluator.h"

namespace {

// Factor to multiply the sensor resolution for our measurement model.
// We model each point as a Gaussian: exp(-x^2 / 2 sigma^2)
// With sigma^2 = (sensor_resolution * kSigmaFactor)^2 + other terms.
const double kSigmaFactor = 0.5;

// Factor to multiply the particle sampling resolution for our measurement  model.
// We model each point as a Gaussian: exp(-x^2 / 2 sigma^2)
// With sigma^2 = (sampling_resolution * kSigmaGridFactor)^2 + other terms.
const double kSigmaGridFactor = 1;

// The noise in our sensor which is independent of the distance to the tracked object.
// We model each point as a Gaussian: exp(-x^2 / 2 sigma^2)
// With sigma^2 = kMinMeasurementVariance^2 + other terms.
const double kMinMeasurementVariance = 0.03;

// We add this to our Gaussian so we don't give 0 probability to points
// that don't align.
// We model each point as a Gaussian: exp(-x^2 / 2 sigma^2) + kSmoothingFactor
const double kSmoothingFactor = 0.8;

// We multiply our log measurement probability by this factor, to decrease
// our confidence in the measurement model (e.g. to take into account
// dependencies between neighboring points).
const double kMeasurementDiscountFactor = 1;

} // namespace

AlignmentEvaluator::AlignmentEvaluator()
  : smoothing_factor_(kSmoothingFactor),
    measurement_discount_factor_(kMeasurementDiscountFactor)
{
}

AlignmentEvaluator::~AlignmentEvaluator() {
  // TODO Auto-generated destructor stub
}

void AlignmentEvaluator::setPrevPoints(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr prev_points) {
  prev_points_ = prev_points;
}

void AlignmentEvaluator::init(
    const double xy_sampling_resolution,
    const double z_sampling_resolution,
    const double sensor_horizontal_resolution,
    const double sensor_vertical_resolution) {
  xy_sampling_resolution_ = xy_sampling_resolution;
  z_sampling_resolution_ = z_sampling_resolution;

  // Compute the expected spatial variance in the x and y directions.
  const double error1_xy = kSigmaGridFactor * xy_sampling_resolution;
  const double error2_xy = sensor_horizontal_resolution * kSigmaFactor;
  const double sigma_xy = sqrt(pow(error1_xy, 2) + pow(error2_xy, 2) +
                               pow(kMinMeasurementVariance, 2));


  const double error1_z = kSigmaGridFactor * z_sampling_resolution_;
  const double error2_z = sensor_vertical_resolution * kSigmaFactor;
  const double sigma_z = sqrt(pow(error1_z, 2) + pow(error2_z, 2) +
                              pow(kMinMeasurementVariance, 2));


  // Convert the variance to a factor such that
  // exp(-x^2 / 2 sigma^2) = exp(x^2 * factor)
  // where x is the distance.
  xy_exp_factor_ = -1.0 / (2 * pow(sigma_xy, 2));
  z_exp_factor_ = -1.0 / (2 * pow(sigma_z, 2));
  xyz_exp_factor_ = -1.0 / (2 * (pow(sigma_xy, 2)) + pow(sigma_z, 2));
  isotropic_ = (xy_exp_factor_ == z_exp_factor_);
}

void AlignmentEvaluator::score3DTransforms(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
    const Eigen::Vector3f& current_points_centroid,
    const double xy_sampling_resolution,
    const double z_sampling_resolution,
    const double sensor_horizontal_resolution,
    const double sensor_vertical_resolution,
    const std::vector<XYZTransform>& transforms,
    const MotionModel& motion_model,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms) {
  // Initialize variables for tracking grid.
  init(xy_sampling_resolution, z_sampling_resolution,
       sensor_horizontal_resolution, sensor_vertical_resolution);

  const size_t num_transforms = transforms.size();

  // Compute scores for all of the transforms using the density grid.
  scored_transforms->clear();
  scored_transforms->resize(num_transforms);

  for(size_t i = 0; i < num_transforms; ++i){
    const XYZTransform& transform = transforms[i];
    const double delta_x = transform.x;
    const double delta_y = transform.y;
    const double delta_z = transform.z;
    const double volume = transform.volume;

    const double log_prob = getLogProbability(
          current_points, current_points_centroid, motion_model,
          delta_x, delta_y, delta_z);

    // Save the complete transform with its log probability.
    const ScoredTransformXYZ scored_transform(delta_x, delta_y, delta_z,
                                              log_prob, volume);
    scored_transforms->set(scored_transform, i);
  }
}
