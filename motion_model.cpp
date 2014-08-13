/*
 * motion_model.cpp
 *
 *  Created on: Jan 23, 2013
 *      Author: davheld
 */

#include "motion_model.h"

#include <iostream>
#include <algorithm>

#include <boost/math/constants/constants.hpp>

#include "transform_helper.h"

using std::endl;
using std::vector;
using std::max;
using std::cout;


const double pi = boost::math::constants::pi<double>();

// How much noise to add to the velocity covariance per 0.1 milliseconds of
// propagation.
const double kPropagationVarianceXY = getenv("PROP_VARIANCE") ? atof(getenv("PROP_VARIANCE")) : 0.1;
const double kPropagationVarianceZ = getenv("PROP_VARIANCE_Z") ? atof(getenv("PROP_VARIANCE_Z")) : 0.1;

const double kMeasurementVariance = getenv("MEASUREMENT_VARIANCE") ? atof(getenv("MEASUREMENT_VARIANCE")) : 0.5;



namespace {

bool computeCovarianceVelocity(
    const ScoredTransforms<ScoredTransformXYZ>& transforms,
    const double& time_diff,
    const Eigen::Vector3d& mean_velocity,
    Eigen::Matrix3d& covariance_velocity){

	double x_velocity_mean = mean_velocity(0);
	double y_velocity_mean = mean_velocity(1);
	double z_velocity_mean = mean_velocity(2);

	covariance_velocity = Eigen::Matrix3d::Zero();

  const std::vector<ScoredTransformXYZ>& scored_transforms =
	    transforms.getScoredTransforms();

	const std::vector<double>& probs = transforms.getNormalizedProbs();

  // Compute half of the covariance matrix.
	const size_t num_transforms = scored_transforms.size();
	for (size_t i = 0; i < num_transforms; ++i) {
    const ScoredTransformXYZ& scored_transform = scored_transforms[i];

	  const double& prob = probs[i];

	  if (prob > 0) {
      const double x_velo_diff = (scored_transform.getX() / time_diff) - x_velocity_mean;
      const double y_velo_diff = (scored_transform.getY() / time_diff) - y_velocity_mean;
      const double z_velo_diff = (scored_transform.getZ() / time_diff) - z_velocity_mean;

      covariance_velocity(0,0) += prob * pow(x_velo_diff, 2);
      covariance_velocity(1,1) += prob * pow(y_velo_diff, 2);
      covariance_velocity(2,2) += prob * pow(z_velo_diff, 2);
      covariance_velocity(0,1) += prob * x_velo_diff * y_velo_diff;
      covariance_velocity(0,2) += prob * x_velo_diff * z_velo_diff;
      covariance_velocity(1,2) += prob * y_velo_diff * z_velo_diff;
	  }
	}

	// Fill in the rest of the covariance matrix by symmetry.
	for (int i = 0; i < 2; i++){
		for (int j = i+1; j < 3; j++){
			covariance_velocity(j,i) = covariance_velocity(i,j);
		}
	}
	return true;
}

} //namespace

Eigen::Vector3d MotionModel::computeMeanVelocity(
    const ScoredTransforms<ScoredTransformXYZ>& transforms_orig,
    const double& time_diff) const {

  // Keep track of the mean velocity for each parameter.
  TransformComponents mean_velocity_components;

  ScoredTransforms<ScoredTransformXYZ> transforms = transforms_orig;

  // Get the transforms and their probabilities.
  const std::vector<ScoredTransformXYZ>& scored_transforms =
      transforms.getScoredTransforms();

  //const std::vector<double>& normalized_probs = transforms.getNormalizedProbsApprox();
  const std::vector<double>& normalized_probs =
      transforms.getNormalizedProbs();

  const size_t num_transforms = scored_transforms.size();

  // Keep track of the sum so we can normalize.
  double prob_sum = 0;

  // Compute the mean components.
  for (size_t i = 0; i < num_transforms; ++i){
    const double& prob = normalized_probs[i];

    if (prob > 0) {

      const ScoredTransformXYZ& scored_transform = scored_transforms[i];

      prob_sum += prob;

      mean_velocity_components.x += scored_transform.getX() * prob;
      mean_velocity_components.y += scored_transform.getY() * prob;
      mean_velocity_components.z += scored_transform.getZ() * prob;
    }
  }

  const double mean_velocity_x = mean_velocity_components.x / (time_diff * prob_sum);
  const double mean_velocity_y = mean_velocity_components.y / (time_diff * prob_sum);
  const double mean_velocity_z = mean_velocity_components.z / (time_diff * prob_sum);

  const Eigen::Vector3d mean_velocity(mean_velocity_x, mean_velocity_y, mean_velocity_z);

  return mean_velocity;

}

double MotionModel::computeScore(const double x, const double y,
    const double z, const double xy_gridStep,
    const double z_gridStep) const {
  if (!valid_){
    return 1;
  }

  TransformComponents components;
  components.x = x;
  components.y = y;
  components.z = z;
  const double new_score = computeScore(components);
  return new_score;
}

double MotionModel::computeScore(const double x, const double y,
    const double z) const {
  if (!valid_){
    return 1;
  }

  TransformComponents components;
  components.x = x;
  components.y = y;
  components.z = z;
  return computeScore(components);
}

double MotionModel::computeScore(const TransformComponents& components) const{
	if (!valid_){
		return 1;
	} else {
		Eigen::Vector3d x(components.x, components.y, components.z);

    Eigen::Vector3d diff = x-mean_delta_position_;
		Eigen::Vector3d a = covariance_delta_position_inv_ * diff;
		double b = -0.5 * diff.transpose() * a;
		double p_translation = max(pdf_constant_ * exp(b), min_score_);


		/*Eigen::Vector2d r(components.roll, components.pitch);
		double b_rotation = -0.5 * r.transpose() * covariance_delta_rotation_inv_ * r;
		double p_rotation = max(exp(b_rotation), 1e-10);
		*/
		//return p_translation * p_rotation;

		return p_translation;
		//double p = max(exp(b), min_score_);
	}
}

void MotionModel::addCentroidDiff(const Eigen::Vector3f& centroid_diff, const double& recorded_time_diff){

  const double time_diff = max(0.01, recorded_time_diff);

	time_diff_ = time_diff;

	//measurement vector
	Eigen::Vector3d z(centroid_diff(0), centroid_diff(1), centroid_diff(2));

	//convert this to a velocity
	z = z / time_diff_;

	//PRINT_WARN_STREAM("z: " << endl << z);

	//C is the identity
	Eigen::Matrix3d C = Eigen::Matrix3d::Identity();

	//x is (v_x, v_y, v_z)

	//a variance of 1
	Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();

	Eigen::MatrixXd K = covariance_velocity_ * C.transpose() * (C * covariance_velocity_ * C.transpose() + Q).inverse();

	mean_velocity_ += K * (z - C * mean_velocity_);

	covariance_velocity_ = (Eigen::Matrix3d::Identity() - K * C) * covariance_velocity_;

	valid_ = true;

}


void MotionModel::propagate(const double& recorded_time_diff){

	if (!valid_){
		//nothing to propagate!
		return;
	}
	time_diff_ = max(recorded_time_diff, 0.01);

  //covariance_delta_position_inv_ = covariance_delta_position_.inverse();

	//add uncertainty to the covariance velocity.
	//the longer the time, the more uncertainty we add (variance adds linearly)
	double min_time = 0.05; //the minimum perceptible time
	covariance_velocity_ +=
			(covariance_propagation_uncertainty_ * (time_diff_ + min_time) / 0.1);
	//covariance_velocity_ += covariance_propagation_uncertainty_ * time_diff / 0.1;
	//PRINT_INFO_STREAM("After propagating, new covariance velocity: " << endl << covariance_velocity_);

	mean_delta_position_ = mean_velocity_ * time_diff_;

  //covariance_delta_position_ = covariance_velocity_ * pow(time_diff_,2);

  covariance_delta_position_ += covariance_velocity_ * pow(time_diff_,2);
  //covariance_delta_position_ /= 2;
  covariance_delta_position_(2,2) = std::max(covariance_delta_position_(2,2), 0.1);

  covariance_delta_position_inv_ = covariance_delta_position_.inverse();

	int k = mean_delta_position_.size();
	pdf_constant_ = 1 / (pow(2 * pi, static_cast<double>(k)/2) * pow(covariance_delta_position_.determinant(), 0.5));

	//compute max score
	TransformComponents mean_components;
	mean_components.x = mean_delta_position_(0);
	mean_components.y = mean_delta_position_(1);
	mean_components.z = mean_delta_position_(2);

	double max_score = computeScore(mean_components);

	//compute eigenvalues
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance_delta_position_);
	if (eigensolver.info() != Eigen::Success){
		min_score_ = max_score / 100;

		printf("Cannot compute eigenvalues - instead chose min_score as %lf", min_score_);

	} else {
		Eigen::Vector3d eigen_values = eigensolver.eigenvalues();
		double max_eigenvalue = eigen_values(2);

		// Save eigenvectors.
		eigen_vectors_ = eigensolver.eigenvectors();
		valid_eigen_vectors_ = true;

		Eigen::Vector3d max_eigenvector = eigensolver.eigenvectors().col(2);

		double sigma = 5;

		TransformComponents sigma_components;
		sigma_components.x = mean_delta_position_(0) + max_eigenvector(0) * sqrt(max_eigenvalue) * sigma;
		sigma_components.y = mean_delta_position_(1) + max_eigenvector(1) * sqrt(max_eigenvalue) * sigma;
		sigma_components.z = mean_delta_position_(2) + max_eigenvector(2) * sqrt(max_eigenvalue) * sigma;

		double sigma_score = computeScore(sigma_components);

		min_score_ = sigma_score;
	}
}

void MotionModel::addTransformsKalman(
		const vector<TransformProposal>& transformProposals,
		const Eigen::Vector4d& centroid,
		const double& recorded_time_diff) {

  const double time_diff = max(0.01, recorded_time_diff);

	time_diff_ = time_diff;

	//make our C_t matrix to hold one row for every measurement in every dimension
	//Eigen::MatrixXf C(3 * transformProposals.size(), 3);
	Eigen::MatrixXd C = Eigen::Matrix3d::Identity().replicate(transformProposals.size(), 1);

	printf("Computing normalizer");
	double prob_sum = 0;
	for (size_t i = 0; i < transformProposals.size(); ++i) {
		prob_sum += transformProposals[i].measurement_prob;
	}

	printf("Computing Q_vect and z");
	Eigen::MatrixXd Q_vect(3 * transformProposals.size(), 1);
	Eigen::MatrixXd z(3 * transformProposals.size(), 1);
	for (size_t i = 0; i < transformProposals.size(); i++) {
		TransformProposal transform_proposal = transformProposals[i];
		TransformComponents components = TransformHelper::computeComponents(
				transform_proposal.full_transform, centroid);

		double normalized_prob = transform_proposal.measurement_prob / prob_sum;
		double var = -log(normalized_prob);

		z(i*3) = (components.x / time_diff);
		z(i*3+1) = (components.y / time_diff);
		z(i*3+2) = (components.z / time_diff);

		for (int j = 0; j < 3; j++){
			Q_vect(i*3 + j) = var;
		}
	}

	printf("Computing Q");
	Eigen::MatrixXd Q = Q_vect.asDiagonal();

	printf("Computing K");
	Eigen::MatrixXd K =
			covariance_velocity_ * C.transpose() *
			(C * covariance_velocity_ * C.transpose() + Q).inverse();

	printf("Computing mean");
	mean_velocity_ += K * (z - C * mean_velocity_);

	printf("Computing covariance");
	covariance_velocity_ = (Eigen::Matrix3d::Identity() - K * C) * covariance_velocity_;

	valid_ = true;
}

void MotionModel::addTransformsWeightedGaussian(
    const ScoredTransforms<ScoredTransformXYZ>& transforms,
		const double& recorded_time_diff) {
	const double time_diff = max(0.01, recorded_time_diff);

	time_diff_ = time_diff;

  mean_velocity_ = computeMeanVelocity(transforms, time_diff);

  mean_delta_position_ = mean_velocity_ * recorded_time_diff;

  bool computed_covariance = computeCovarianceVelocity(transforms, time_diff, mean_velocity_, covariance_velocity_);

	covariance_delta_position_ = covariance_velocity_ * pow(time_diff, 2);

	valid_ = computed_covariance;

	if (!valid_){
		printf("Motion model is not valid\n");
	}
}

Eigen::Vector3f MotionModel::mean_displacement() const {
	return mean_velocity_.cast<float>() * time_diff_;
}


MotionModel::MotionModel()
	:pdf_constant_(1),
	 min_score_(1e-4),
   valid_eigen_vectors_(false),
	 valid_(false)
{

	//R_t in the kalman filter - the amount of uncertainty propagation in 0.1 s
  double xy_variance = kPropagationVarianceXY; //1; //0.25;
	double z_variance = kPropagationVarianceZ; //0.1
	covariance_propagation_uncertainty_ = Eigen::Matrix3d::Zero();
	covariance_propagation_uncertainty_(0,0) = xy_variance;
	covariance_propagation_uncertainty_(1,1) = xy_variance;
	covariance_propagation_uncertainty_(2,2) = z_variance;

	//used for centroid kalman filter
	covariance_velocity_ = 100 * Eigen::Matrix3d::Identity();
	//covariance_velocity_(2,2) = z_variance;
	mean_velocity_ = Eigen::Vector3d::Zero();

	//double roll_variance = 0.001;
	//double pitch_variance = 0.001;

	/*Eigen::Matrix2d covariance_delta_rotation = Eigen::Matrix2d::Zero();
	covariance_delta_rotation(0,0) = roll_variance;
	covariance_delta_rotation(1,1) = pitch_variance;
	covariance_delta_rotation_inv_ = covariance_delta_rotation.inverse();*/

  //PRINT_INFO_STREAM("Adding covariance_propagation_uncertainty_: " << endl
   //   << covariance_propagation_uncertainty_);
}

MotionModel::~MotionModel() {
	// TODO Auto-generated destructor stub
}

