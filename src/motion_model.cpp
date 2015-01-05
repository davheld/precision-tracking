/*
 * motion_model.cpp
 *
 *  Created on: Jan 23, 2013
 *      Author: davheld
 *
 */


#include <boost/math/constants/constants.hpp>

#include <precision_tracking/motion_model.h>

using std::max;

namespace precision_tracking {

namespace {

const double pi = boost::math::constants::pi<double>();

} // namespace


MotionModel::MotionModel(const Params *params)
  : params_(params),
    pdf_constant_(1),
    min_score_(params_->kMotionMinProb),
    valid_(false),
    flip_(1)
{
  // We assume that, at each time step, we independently sample an acceleration
  // with 0 mean and covariance of covariance_propagation_uncertainty_.
  covariance_propagation_uncertainty_ = Eigen::Matrix3d::Zero();
  covariance_propagation_uncertainty_(0,0) = params_->kPropagationVarianceXY;
  covariance_propagation_uncertainty_(1,1) = params_->kPropagationVarianceXY;
  covariance_propagation_uncertainty_(2,2) = params_->kPropagationVarianceZ;

  // For the centroid Kalman filter, we set the initial mean and covariance.
  // The precision tracker uses the first alignment to initialize the mean
  // and covariance.
  covariance_velocity_ =
      params_->kCentroidInitVelocityVariance * Eigen::Matrix3d::Identity();
  mean_velocity_ = Eigen::Vector3d::Zero();
}

MotionModel::~MotionModel()
{
  // TODO Auto-generated destructor stub
}

void MotionModel::addTransformsWeightedGaussian(
    const ScoredTransforms<ScoredTransformXYZ>& transforms,
    const double& recorded_time_diff)
{
  // If the recorded_time_diff is 0 or very small, we avoid numerical
  // issues by thresholding at 0.01.
  const double time_diff = max(0.01, recorded_time_diff);

  // Compute the mean velocity from the distribution of tranform probabilities.
  mean_velocity_ = computeMeanVelocity(transforms, time_diff);

  mean_delta_position_ = mean_velocity_ * recorded_time_diff;

  // Compute the covariance velocity from the distribution of tranform
  // probabilities.
  covariance_velocity_ = computeCovarianceVelocity(
        transforms, time_diff, mean_velocity_);

  // Because covariance is quadratic, we must multiply by the square of the
  // time difference.
  covariance_delta_position_ = covariance_velocity_ * pow(time_diff, 2);

  // The motion model is now valid.
  valid_ = true;
}


Eigen::Matrix3d MotionModel::computeCovarianceVelocity(
    const ScoredTransforms<ScoredTransformXYZ>& transforms,
    const double time_diff,
    const Eigen::Vector3d& mean_velocity) const
{
  double x_velocity_mean = mean_velocity(0);
  double y_velocity_mean = mean_velocity(1);
  double z_velocity_mean = mean_velocity(2);

  // Initialize the covariance velocity to zero.
  Eigen::Matrix3d covariance_velocity = Eigen::Matrix3d::Zero();

  const std::vector<ScoredTransformXYZ>& scored_transforms =
      transforms.getScoredTransforms();
  const std::vector<double>& probs = transforms.getNormalizedProbs();

  // Compute the upper-right half of the covariance matrix.
  const size_t num_transforms = scored_transforms.size();
  for (size_t i = 0; i < num_transforms; ++i) {
    const ScoredTransformXYZ& scored_transform = scored_transforms[i];

    const double prob = probs[i];

    // Compute the difference from the mean.
    const double x_velo_diff = (flip_ * scored_transform.getX() / time_diff) - x_velocity_mean;
    const double y_velo_diff = (flip_ * scored_transform.getY() / time_diff) - y_velocity_mean;
    const double z_velo_diff = (flip_ * scored_transform.getZ() / time_diff) - z_velocity_mean;

    // Compute the weighted contribution to the covaraince.
    covariance_velocity(0,0) += prob * pow(x_velo_diff, 2);
    covariance_velocity(1,1) += prob * pow(y_velo_diff, 2);
    covariance_velocity(2,2) += prob * pow(z_velo_diff, 2);
    covariance_velocity(0,1) += prob * x_velo_diff * y_velo_diff;
    covariance_velocity(0,2) += prob * x_velo_diff * z_velo_diff;
    covariance_velocity(1,2) += prob * y_velo_diff * z_velo_diff;
  }

  // Fill in the rest of the covariance matrix by symmetry.
  for (int i = 0; i < 2; i++){
    for (int j = i+1; j < 3; j++){
      covariance_velocity(j,i) = covariance_velocity(i,j);
    }
  }

  return covariance_velocity;
}

Eigen::Vector3d MotionModel::computeMeanVelocity(
    const ScoredTransforms<ScoredTransformXYZ>& transforms,
    const double time_diff) const
{
  // Compute the mean delta position.
  TransformComponents mean_delta_position;

  // Get the transforms and their probabilities.
  const std::vector<ScoredTransformXYZ>& scored_transforms =
      transforms.getScoredTransforms();

  const std::vector<double>& normalized_probs =
      transforms.getNormalizedProbs();

  const size_t num_transforms = scored_transforms.size();

  // Keep track of the sum so we can normalize.
  double prob_sum = 0;

  // Compute the mean components.
  for (size_t i = 0; i < num_transforms; ++i){
    const double prob = normalized_probs[i];

    const ScoredTransformXYZ& scored_transform = scored_transforms[i];

    prob_sum += prob;

    mean_delta_position.x += flip_ * scored_transform.getX() * prob;
    mean_delta_position.y += flip_ * scored_transform.getY() * prob;
    mean_delta_position.z += flip_ * scored_transform.getZ() * prob;
  }

  // Normalize, and divide by time to get the velocity.
  const double mean_velocity_x = mean_delta_position.x / (time_diff * prob_sum);
  const double mean_velocity_y = mean_delta_position.y / (time_diff * prob_sum);
  const double mean_velocity_z = mean_delta_position.z / (time_diff * prob_sum);

  const Eigen::Vector3d mean_velocity(
        mean_velocity_x, mean_velocity_y, mean_velocity_z);

  return mean_velocity;
}

double MotionModel::computeScore(const double x, const double y,
    const double z) const
{
  TransformComponents components;
  components.x = x;
  components.y = y;
  components.z = z;
  return computeScore(components);
}

double MotionModel::computeScore(const TransformComponents& components) const
{
  if (!valid_) {
		return 1;
  } else {
    Eigen::Vector3d x(components.x, components.y, components.z);

    // Compute the probability from the normal distribution.
    Eigen::Vector3d diff = flip_ * x - mean_delta_position_;
    double log_prob =
        -0.5 * diff.transpose() * covariance_delta_position_inv_ * diff;
    double prob = max(pdf_constant_ * exp(log_prob), min_score_);

    return prob;
  }
}

void MotionModel::addCentroidDiff(const Eigen::Vector4f& centroid_diff,
                                  const double recorded_time_diff)
{
  // If the recorded_time_diff is 0 or very small, we avoid numerical
  // issues by thresholding at 0.01.
  const double time_diff = max(0.01, recorded_time_diff);

  // Compute the position measurement vector.
  Eigen::Vector3d z(centroid_diff(0), centroid_diff(1), centroid_diff(2));

  // Convert to a velocity measurement.
  z = z / time_diff;

  // C is the identity because we have the velocity corrupted by noise.
	Eigen::Matrix3d C = Eigen::Matrix3d::Identity();

  // Compute the measurement covariance.
  Eigen::Matrix3d Q =
      params_->kCentroidMeasurementNoise * Eigen::Matrix3d::Identity();

  // Compute the Kalman gain.
  Eigen::MatrixXd K = covariance_velocity_ * C.transpose() *
      (C * covariance_velocity_ * C.transpose() + Q).inverse();

  // Update the mean velocity.
	mean_velocity_ += K * (z - C * mean_velocity_);

  // Update the covariance velocity.
  covariance_velocity_ =
      (Eigen::Matrix3d::Identity() - K * C) * covariance_velocity_;

  // After the first measurement, the motion model is valid.
	valid_ = true;
}

void MotionModel::propagate(const double& recorded_time_diff)
{
	if (!valid_){
    // Nothing to propagate!
		return;
	}

  // If the recorded_time_diff is 0 or very small, we avoid numerical
  // issues by thresholding the minimum time diff.
  const double time_diff = max(recorded_time_diff, 0.01);

  mean_delta_position_ = mean_velocity_ * time_diff;

  // Add uncertainty to the covariance velocity.
  // We assume that, at each time step, we independently sample an acceleration
  // with 0 mean and covariance of covariance_propagation_uncertainty_.  The
  // effect of the acceleration uncertainty on the velocity covariance is then
  // given by the below formula.
  covariance_velocity_ +=
      covariance_propagation_uncertainty_ * pow(time_diff,2);

  // Similarly to the above, the uncertainty in velocity has a quadratic
  // effect with time on the uncertainty on the position.
  covariance_delta_position_ += covariance_velocity_ * pow(time_diff,2);

  // If we are not estimating the vertical motion, we could end up with a
  // covariance of 0, which could lead to numerical instability.  Instead,
  // we set a minimum threshold the vertical uncertainty.
  covariance_delta_position_(2,2) =
      std::max(covariance_delta_position_(2,2), 0.1);

  covariance_delta_position_inv_ = covariance_delta_position_.inverse();

  // Compute the constant in front of the multivariate Guassian.
  const double k = mean_delta_position_.size();
  const double determinant = covariance_delta_position_.determinant();
  pdf_constant_ = 1 / (pow(2 * pi, k/2) * pow(determinant, 0.5));
}

} // namespace precision_tracking
