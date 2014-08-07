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
//#include "helper.h"
//#include "../../ros-pkg/bag_of_tricks/include/bag_of_tricks/high_res_timer.h"

using std::endl;
using std::vector;
using std::max;
using std::cout;


namespace{
  //const char* logger = ROSCONSOLE_DEFAULT_NAME ".motion_model";
}

const int kNumTransformsMean = getenv("NUM_TRANSFORMS_MEAN") ? atoi(getenv("NUM_TRANSFORMS_MEAN")) : 1;

const double pi = boost::math::constants::pi<double>();

//const bool kUse_annealing = getenv("USE_ANNEALING");

// How far to spill over in the density grid (number of sigmas).
const double kPropagationVariance = getenv("PROP_VARIANCE") ? atof(getenv("PROP_VARIANCE")) : 0.5;
const double kPropagationVarianceZ = getenv("PROP_VARIANCE_Z") ? atof(getenv("PROP_VARIANCE_Z")) : 0.1;

const double kSigmaGridFactorMotion = getenv("SIGMA_GRID_FACTOR_MOTION") ? atof(getenv("SIGMA_GRID_FACTOR_MOTION")) : 2;

const bool kInflateMotion = getenv("INFLATE_MOTION");

const double kMeasurementVariance = getenv("MEASUREMENT_VARIANCE") ? atof(getenv("MEASUREMENT_VARIANCE")) : 1;



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

  //const std::vector<double>& probs = transforms.getNormalizedProbsApprox();
	const std::vector<double>& probs = transforms.getNormalizedProbs();

  // Compute half of the covariance matrix.
  //printf("Computing covariance\n");
	const size_t num_transforms = scored_transforms.size();
	for (size_t i = 0; i < num_transforms; ++i) {
		//const TransformProposal transform_proposal = transformProposals[i];
		//const TransformComponents components = TransformHelper::computeComponents(
	//			transform_proposal.full_transform, centroid);

		//const double prob = transform_proposal.prob_with_motion;

    const ScoredTransformXYZ& scored_transform = scored_transforms[i];

	  const double& prob = probs[i];

	  if (prob > 0) {
      const double x_velo_diff = (scored_transform.getX() / time_diff) - x_velocity_mean;
      const double y_velo_diff = (scored_transform.getY() / time_diff) - y_velocity_mean;
      const double z_velo_diff = (scored_transform.getZ() / time_diff) - z_velocity_mean;

      /*printf("prob: %lf, x diff: %lf, %lf, y_diff: %lf, %lf\n",
          prob,
          x_velo_diff, prob * pow(x_velo_diff, 2),
          y_velo_diff, prob * pow(y_velo_diff, 2));*/

      /*if (prob * pow(y_velo_diff, 2) > 0.01) {
        printf("Prob: %lf, y: %lf, y_velo_diff: %lf, prob * pow(y_velo_diff, 2): %lf\n",
            prob, scored_transform.getY(), y_velo_diff, prob * pow(y_velo_diff, 2));
      }*/

      /*if (prob > 0.01) {
        printf("Prob: %lf, x: %lf, y: %lf\n",
            prob, scored_transform.getX(), scored_transform.getY());
      }*/

      covariance_velocity(0,0) += prob * pow(x_velo_diff, 2);
      covariance_velocity(1,1) += prob * pow(y_velo_diff, 2);
      covariance_velocity(2,2) += prob * pow(z_velo_diff, 2);
      covariance_velocity(0,1) += prob * x_velo_diff * y_velo_diff;
      covariance_velocity(0,2) += prob * x_velo_diff * z_velo_diff;
      covariance_velocity(1,2) += prob * y_velo_diff * z_velo_diff;
	  }
	}

	//printf("After iterating: x: %lf, y: %lf\n", covariance_velocity(0,0),
	//    covariance_velocity(1,1));

	// Fill in the rest of the covariance matrix by symmetry.
	for (int i = 0; i < 2; i++){
		for (int j = i+1; j < 3; j++){
			covariance_velocity(j,i) = covariance_velocity(i,j);
		}
	}

	//PRINT_WARN_STREAM("Computed covariance velocity to be " << endl << covariance_velocity);

	return true;

}

/*bool compareTransforms(const ScoredTransformXYZ& transform_i,
                       const ScoredTransformXYZ& transform_j)
{
  const double score_i = transform_i.getUnnormalizedLogProb() -
      fast_functions1_.getFastLog(transform_i.getVolume());
  const double score_j = transform_j.getUnnormalizedLogProb() -
      fast_functions1_.getFastLog(transform_j.getVolume());
  return score_i > score_j;
}*/

} //namespace

Eigen::Vector3d MotionModel::computeMeanVelocity(
    const ScoredTransforms<ScoredTransformXYZ>& transforms_orig,
    const double& time_diff) const {

  // Keep track of the mean velocity for each parameter.
  TransformComponents mean_velocity_components;

  ScoredTransforms<ScoredTransformXYZ> transforms = transforms_orig;

  //transforms.sortDescending();

  // Get the transforms and their probabilities.
  const std::vector<ScoredTransformXYZ>& scored_transforms =
      transforms.getScoredTransforms();

  //const std::vector<double>& normalized_probs = transforms.getNormalizedProbsApprox();
  const std::vector<double>& normalized_probs =
      transforms.getNormalizedProbs();

  //std::vector<ScoredTransformXYZ> sorted_transforms = scored_transforms;

  //std::sort(scored_transforms.begin(), scored_transforms.end(),
  //          compareTransforms);

  //std::sort(normalized_probs.begin(), normalized_probs.end(),
  //    std::greater<double>());

  const size_t num_transforms = scored_transforms.size();

  //const double min_prob = 1.0 / num_transforms;

  //HighResTimer hrt("Get normalized probs");
  //hrt.start();


  //hrt.stop();
  //hrt.printMilliseconds();

  // Keep track of the sum so we can normalize.
  double prob_sum = 0;

  //const int num_transforms_mean = kNumTransformsMean == 0 ? num_transforms :
  //  std::min(static_cast<int>(num_transforms), kNumTransformsMean);
  //printf("num_transforms_mean: %d, %d\n", num_transforms_mean, kNumTransformsMean);

  // Compute the mean components.
  for (size_t i = 0; i < num_transforms; ++i){
    //const TransformProposal& transform_proposal = transforms[i];
    //const TransformComponents components = TransformHelper::computeComponents(transform_proposal.full_transform, centroid);

    //const double prob = transform_proposal.prob_with_motion;
    const double& prob = normalized_probs[i];

    if (prob > 0) {

      const ScoredTransformXYZ& scored_transform = scored_transforms[i];

      prob_sum += prob;

      mean_velocity_components.x += scored_transform.getX() * prob;
      mean_velocity_components.y += scored_transform.getY() * prob;
      mean_velocity_components.z += scored_transform.getZ() * prob;

      /*mean_velocity_components.x += components.x * prob;
      mean_velocity_components.y += components.y * prob;
      mean_velocity_components.z += components.z * prob;*/

      //if (prob > 0.001) {

        /*PRINT_INFO("Prob: %lf, components: %lf, %lf, %lf",
            prob, scored_transform.getX(), scored_transform.getY(),
            scored_transform.getZ());*/
      //}
    }
  }

  //PRINT_INFO("Prob_sum: %lf", prob_sum);

  /*const double mean_velocity_x = mean_velocity_components.x / time_diff;
  const double mean_velocity_y = mean_velocity_components.y / time_diff;
  const double mean_velocity_z = mean_velocity_components.z / time_diff;*/

  const double mean_velocity_x = mean_velocity_components.x / (time_diff * prob_sum);
  const double mean_velocity_y = mean_velocity_components.y / (time_diff * prob_sum);
  const double mean_velocity_z = mean_velocity_components.z / (time_diff * prob_sum);

  const Eigen::Vector3d mean_velocity(mean_velocity_x, mean_velocity_y, mean_velocity_z);

  return mean_velocity;

}

double MotionModel::computeScoreXY(const double x, const double y) const {
	TransformComponents components;
	components.x = x;
	components.y = y;
	components.z = mean_delta_position_(2);
	return computeScore(components);
}


double MotionModel::computeScoreZ(const double z) const {
	TransformComponents components;
	components.x = mean_delta_position_(0);
	components.y = mean_delta_position_(1);
	components.z = z;
	return computeScore(components);
}

double MotionModel::computeScoreZ(const double z,
    const double z_gridStep) const {
  // Move up to half of the grid step towards the mean.
  Eigen::Vector3d pos(mean_delta_position_(0), mean_delta_position_(1), z);
  Eigen::Vector3d diff = mean_delta_position_ - pos;

  double new_x, new_y, new_z;
  new_x = mean_delta_position_(0);
  new_y = mean_delta_position_(1);

  if (fabs(diff(2)) < z_gridStep / 2) {
    // If the correct answer is within range, use that.
    new_z = mean_delta_position_(2);
  } else {
    // Move as much as we can towards the correct answer.
    new_z = z + (diff(2) / fabs(diff(2))) * z_gridStep / 2;
  }

  TransformComponents components;
  components.x = new_x;
  components.y = new_y;
  components.z = new_z;
  return computeScore(components);
}

double MotionModel::computeScoreXY(const double x, const double y,
    const double xy_gridStep) const {
  if (!valid_){
    return 1;
  }

  // Move up to half of the grid step towards the mean.
  const Eigen::Vector3d pos(x, y, mean_delta_position_(2));
  const Eigen::Vector3d diff = mean_delta_position_ - pos;

  Eigen::Vector3d new_pos = pos;
  if (!valid_eigen_vectors_) {
    return computeScoreXY(x, y);
  } else {
    for (int i = 0; i < eigen_vectors_.cols(); ++i) {
      const Eigen::Vector3d& eigen_vect = eigen_vectors_.col(i);
      const double dot_prod = diff.dot(eigen_vect);
      if (dot_prod != 0) {
        const double distance = std::min(xy_gridStep / 2, fabs(dot_prod));
        new_pos += eigen_vect * distance * dot_prod / fabs(dot_prod);
      }
    }
  }

  const double new_x = new_pos(0);
  const double new_y = new_pos(1);
  const double new_z = new_pos(2);

  TransformComponents components;
  components.x = new_x;
  components.y = new_y;
  components.z = new_z;
  const double new_score = computeScore(components);

  return new_score;


  // Move up to half of the grid step towards the mean.
  /*Eigen::Vector3d pos(x, y, mean_delta_position_(2));
  Eigen::Vector3d diff = mean_delta_position_ - pos;

  double new_x, new_y, new_z;
  if (fabs(diff(0)) < xy_gridStep / 2) {
    // If the correct answer is within range, use that.
    new_x = mean_delta_position_(0);
  } else {
    // Move as much as we can towards the correct answer.
    new_x = x + (diff(0) / fabs(diff(0))) * xy_gridStep / 2;
  }
  //printf("%lf, %lf, %lf, %lf\n", x, diff(0), xy_gridStep / 2, new_x);

  if (fabs(diff(1)) < xy_gridStep / 2) {
    // If the correct answer is within range, use that.
    new_y = mean_delta_position_(1);
  } else {
    // Move as much as we can towards the correct answer.
    new_y = y + (diff(1) / fabs(diff(1))) * xy_gridStep / 2;
  }

  new_z = mean_delta_position_(2);

  TransformComponents components;
  components.x = new_x;
  components.y = new_y;
  components.z = new_z;
  return computeScore(components);*/
}

double MotionModel::computeScore(const double x, const double y,
    const double z, const double xy_gridStep,
    const double z_gridStep) const {
  if (!valid_){
    return 1;
  }

  //if (!kUse_annealing) {
    TransformComponents components;
    components.x = x;
    components.y = y;
    components.z = z;
    const double new_score = computeScore(components);
    return new_score;
  //}

  //printf("Annealing motion model\n");

  /*if (kInflateMotion) {
    TransformComponents components;
    components.x = x;
    components.y = y;
    components.z = z;

    return computeScore(components, xy_gridStep, z_gridStep);
  } else {
    // Move up to half of the grid step towards the mean.
    const Eigen::Vector3d pos(x, y, z);
    const Eigen::Vector3d diff = mean_delta_position_ - pos;

    Eigen::Vector3d new_pos = pos;
    if (!valid_eigen_vectors_) {
      return computeScore(x, y, z);
    } else {
      for (int i = 0; i < eigen_vectors_.cols(); ++i) {
        const Eigen::Vector3d& eigen_vect = eigen_vectors_.col(i);
        const double dot_prod = diff.dot(eigen_vect);
        if (dot_prod != 0) {
          const double distance = std::min(xy_gridStep / 2, fabs(dot_prod));
          new_pos += eigen_vect * distance * dot_prod / fabs(dot_prod);
        }
      }
    }

    const double new_x = new_pos(0);
    const double new_y = new_pos(1);
    const double new_z = new_pos(2);

    const double old_score = computeScore(x, y, z);

    TransformComponents components;
    components.x = new_x;
    components.y = new_y;
    components.z = new_z;
    const double new_score = computeScore(components);

    return new_score;
  }*/
  /*printf("Pos: %lf, %lf, %lf\n", x, y, z);
  printf("Mean delta: %lf, %lf, %lf\n", mean_delta_position_(0),
      mean_delta_position_(1), mean_delta_position_(2));
  printf("New Pos: %lf, %lf, %lf\n", new_x, new_y, new_z);*/


  /*double new_x, new_y, new_z;
  if (fabs(diff(0)) < xy_gridStep / 2) {
    // If the correct answer is within range, use that.
    new_x = mean_delta_position_(0);
  } else {
    // Move as much as we can towards the correct answer.
    new_x = x + (diff(0) / fabs(diff(0))) * xy_gridStep / 2;
  }
  //printf("%lf, %lf, %lf, %lf\n", x, diff(0), xy_gridStep / 2, new_x);

  if (fabs(diff(1)) < xy_gridStep / 2) {
    // If the correct answer is within range, use that.
    new_y = mean_delta_position_(1);
  } else {
    // Move as much as we can towards the correct answer.
    new_y = y + (diff(1) / fabs(diff(1))) * xy_gridStep / 2;
  }

  if (fabs(diff(2)) < z_gridStep / 2) {
    // If the correct answer is within range, use that.
    new_z = mean_delta_position_(2);
  } else {
    // Move as much as we can towards the correct answer.
    new_z = z + (diff(2) / fabs(diff(2))) * z_gridStep / 2;
  }*/
}

double MotionModel::computeScore(const double x, const double y,
    const double z) const {
  TransformComponents components;
  components.x = x;
  components.y = y;
  components.z = z;
  return computeScore(components);
}

double MotionModel::computeScore(const TransformComponents& components,
    const double xy_stepSize, const double z_stepSize) const{
  if (!valid_){
    return 1;
  } else {
    //PRINT_WARN("Computing score");

    printf("Annealing motion model\n");

    const double error1_xy = kSigmaGridFactorMotion * xy_stepSize;
    const double error1_z = kSigmaGridFactorMotion * z_stepSize;

    Eigen::Matrix3d covariance_delta_position_inflated =
        covariance_delta_position_;

    covariance_delta_position_inflated(0,0) += error1_xy;
    covariance_delta_position_inflated(1,1) += error1_xy;
    covariance_delta_position_inflated(2,2) += error1_z;

    const Eigen::Matrix3d covariance_delta_position_inflated_inv =
        covariance_delta_position_inflated.inverse();

    //PRINT_INFO_STREAM("covariance_delta_position_inflated: " << endl << covariance_delta_position_inflated);


    Eigen::Vector3d x(components.x, components.y, components.z);
    //PRINT_INFO_STREAM("position: " << endl << x);

    //printf("mean_delta_position_: %lf, %lf, %lf\n", mean_delta_position_(0),
    //    mean_delta_position_(1), mean_delta_position_(2));
    Eigen::Vector3d diff = x-mean_delta_position_;
    //PRINT_INFO_STREAM("mean_delta_position_: " << endl << mean_delta_position_);

    Eigen::Vector3d a = covariance_delta_position_inflated_inv * diff;
    double b = -0.5 * diff.transpose() * a;
    double p_translation = max(pdf_constant_ * exp(b), min_score_);

    //printf("Score: %lf\n", p_translation);

    /*Eigen::Vector2d r(components.roll, components.pitch);
    //PRINT_INFO_STREAM("R: " << r);
    double b_rotation = -0.5 * r.transpose() * covariance_delta_rotation_inv_ * r;
    //PRINT_INFO_STREAM("B_rotation: " << b_rotation);
    double p_rotation = max(exp(b_rotation), 1e-10);
    PRINT_INFO("roll: %lf, pitch: %lf, p_rotation: %lf", components.roll, components.pitch, p_rotation);*/
    //return p_translation * p_rotation;

    return p_translation;
    //double p = max(exp(b), min_score_);
  }
}

double MotionModel::computeScore(const TransformComponents& components) const{
	if (!valid_){
		return 1;
	} else {
		Eigen::Vector3d x(components.x, components.y, components.z);
    //printf("mean_delta_position_: %lf, %lf, %lf\n", mean_delta_position_(0),
    //    mean_delta_position_(1), mean_delta_position_(2));
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

// The state x is (v_x, v_y, v_z)
void MotionModel::addAlignment(
    Eigen::Affine3f full_alignment_to_prev,
    const double& recorded_time_diff,
    const double& horizontal_distance) {
  const double time_diff = max(0.01, recorded_time_diff);

  time_diff_ = time_diff;

  //PRINT_WARN_STREAM("full_alignment_to_prev: " << endl << full_alignment_to_prev.matrix());

  //convert measurement to a linear velocity.
  Eigen::Vector3d z =
      full_alignment_to_prev.translation().cast<double>() / time_diff_;

  //PRINT_WARN_STREAM("z: " << endl << z);

  //C is the identity (the state is the measurement)
  Eigen::Matrix3d C = Eigen::Matrix3d::Identity();

  // Compute the sensor resolution
  const double velodyne_horizontal_res =
      2 * horizontal_distance * tan(.18 / 2.0 * pi / 180.0);
  const double velodyne_vertical_res = 2.2 * velodyne_horizontal_res;

  // Compute the measurement variance based on the sensor resolution.
  Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
  Q(0,0) = kMeasurementVariance; //velodyne_horizontal_res;
  Q(1,1) = kMeasurementVariance; //velodyne_horizontal_res;
  Q(2,2) = kMeasurementVariance; //velodyne_vertical_res;

  Eigen::MatrixXd K =
      covariance_velocity_ * C.transpose() * (C * covariance_velocity_ * C.transpose() + Q).inverse();
  //PRINT_WARN_STREAM("Computed K" << endl << K);

  mean_velocity_ += K * (z - C * mean_velocity_);
  //PRINT_WARN_STREAM("Computed mean" << endl << mean_velocity_);

  covariance_velocity_ = (Eigen::Matrix3d::Identity() - K * C) * covariance_velocity_;
  //PRINT_WARN_STREAM("Computing covariance" << endl << covariance_velocity_);

  valid_ = true;
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
	//PRINT_WARN_STREAM("Computed K" << endl << K);

	mean_velocity_ += K * (z - C * mean_velocity_);
	//PRINT_WARN_STREAM("Computed mean" << endl << mean_velocity_);

	covariance_velocity_ = (Eigen::Matrix3d::Identity() - K * C) * covariance_velocity_;
	//PRINT_WARN_STREAM("Computing covariance" << endl << covariance_velocity_);

	valid_ = true;

}

void MotionModel::addCentroidDiffNoUpdate(
    const Eigen::Vector3f& centroid_diff,
    const double& recorded_time_diff,
    Eigen::Vector3d* mean_velocity,
    Eigen::Matrix3d* covariance_velocity) const {
  const double time_diff = max(0.01, recorded_time_diff);

  //measurement vector
  Eigen::Vector3d z(centroid_diff(0), centroid_diff(1), centroid_diff(2));

  //cout << "z: " << z << endl;

  //convert this to a velocity
  z = z / time_diff;

  //cout << "z/time_diff: " << z << endl;

  //C is the identity
  Eigen::Matrix3d C = Eigen::Matrix3d::Identity();

  //x is (v_x, v_y, v_z)

  //a variance of 1
  Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();

  Eigen::MatrixXd K = covariance_velocity_ * C.transpose() * (C * covariance_velocity_ * C.transpose() + Q).inverse();
  //PRINT_WARN_STREAM("Computed K" << endl << K);

  //cout << "mean_velocity_: " << mean_velocity_ << endl;

  *mean_velocity = mean_velocity_ + K * (z - C * mean_velocity_);

  //cout << "mean_velocity_: " << mean_velocity_ << endl;

  *covariance_velocity = (Eigen::Matrix3d::Identity() - K * C) * covariance_velocity_;
  //PRINT_WARN_STREAM("Computing covariance" << endl << covariance_velocity_);
}


void MotionModel::propagate(const double& recorded_time_diff){

	if (!valid_){
		//nothing to propagate!
		return;
	}
	time_diff_ = max(recorded_time_diff, 0.01);

  //PRINT_INFO_STREAM("covariance_delta_position_: " << endl << covariance_delta_position_);

  //covariance_delta_position_inv_ = covariance_delta_position_.inverse();


	//printf("Propagating with time diff: %lf\n", time_diff_);

  //PRINT_INFO_STREAM("Before propagating, covariance velocity: " << endl << covariance_velocity_);

  //PRINT_INFO_STREAM("Adding covariance_propagation_uncertainty_: " << endl << covariance_propagation_uncertainty_);
  //cout << "Adding covariance_propagation_uncertainty_: " << endl << covariance_propagation_uncertainty_;

	//add uncertainty to the covariance velocity.
	//the longer the time, the more uncertainty we add (variance adds linearly)
	double min_time = 0.05; //the minimum perceptible time
	covariance_velocity_ +=
			(covariance_propagation_uncertainty_ * (time_diff_ + min_time) / 0.1);
	//covariance_velocity_ += covariance_propagation_uncertainty_ * time_diff / 0.1;
	//PRINT_INFO_STREAM("After propagating, new covariance velocity: " << endl << covariance_velocity_);

	mean_delta_position_ = mean_velocity_ * time_diff_;
  //PRINT_INFO_STREAM("mean_delta_position_: " << endl << mean_delta_position_);

  //PRINT_INFO_STREAM("covariance_delta_position_inv_: " << endl << covariance_delta_position_inv_);

	//PRINT_INFO_STREAM("covariance_delta_position_: " << endl << covariance_delta_position_);

  //covariance_delta_position_ = covariance_velocity_ * pow(time_diff_,2);

  covariance_delta_position_ += covariance_velocity_ * pow(time_diff_,2);
  //covariance_delta_position_ /= 2;
  covariance_delta_position_(2,2) = std::max(covariance_delta_position_(2,2), 0.1);

  covariance_delta_position_inv_ = covariance_delta_position_.inverse();

	int k = mean_delta_position_.size();
	pdf_constant_ = 1 / (pow(2 * pi, static_cast<double>(k)/2) * pow(covariance_delta_position_.determinant(), 0.5));

	//PRINT_INFO("pdf_constant_: %lf", pdf_constant_);

	//compute max score
	TransformComponents mean_components;
	mean_components.x = mean_delta_position_(0);
	mean_components.y = mean_delta_position_(1);
	mean_components.z = mean_delta_position_(2);

	double max_score = computeScore(mean_components);

	//PRINT_WARN("Max score: %lf", max_score);

	//compute eigenvalues
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance_delta_position_);
	if (eigensolver.info() != Eigen::Success){
		min_score_ = max_score / 100;

		printf("Cannot compute eigenvalues - instead chose min_score as %lf", min_score_);

	} else {
		Eigen::Vector3d eigen_values = eigensolver.eigenvalues();
		//PRINT_WARN_STREAM("Eigen values: " << eigen_values);
		double max_eigenvalue = eigen_values(2);

		// Save eigenvectors.
		eigen_vectors_ = eigensolver.eigenvectors();
		valid_eigen_vectors_ = true;

		Eigen::Vector3d max_eigenvector = eigensolver.eigenvectors().col(2);

		//PRINT_INFO("Max eigenvalue: %lf", max_eigenvalue);

		double sigma = 5;

		TransformComponents sigma_components;
		sigma_components.x = mean_delta_position_(0) + max_eigenvector(0) * sqrt(max_eigenvalue) * sigma;
		sigma_components.y = mean_delta_position_(1) + max_eigenvector(1) * sqrt(max_eigenvalue) * sigma;
		sigma_components.z = mean_delta_position_(2) + max_eigenvector(2) * sqrt(max_eigenvalue) * sigma;

		double sigma_score = computeScore(sigma_components);

		//PRINT_WARN_STREAM("Max eigenvector: " << std::endl << max_eigenvector << std::endl << " with eigenvalue: " << max_eigenvalue);
		//PRINT_WARN("Computed %lf sigma score as %lf", sigma, sigma_score);

		min_score_ = sigma_score;
		//PRINT_INFO("Computed %lf sigma score as %lf", sigma, min_score_);
	}

	//PRINT_WARN("Debug*************");
	//min_score_ = 0;

	//PRINT_WARN("Done propagating - pdf_constant: %lf, motion model is valid: %d", pdf_constant_, valid_);
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

	//HighResTimer hrt("Compute mean velocity");
	//hrt.start();

  mean_velocity_ = computeMeanVelocity(transforms, time_diff);

	/*hrt.stop();
	hrt.printMilliseconds();
	hrt.reset("Compute covariance");
	hrt.start();*/

  //PRINT_WARN_STREAM("Computed mean velocity to be: " << endl << mean_velocity_);

  mean_delta_position_ = mean_velocity_ * recorded_time_diff;

  bool computed_covariance = computeCovarianceVelocity(transforms, time_diff, mean_velocity_, covariance_velocity_);

	covariance_delta_position_ = covariance_velocity_ * pow(time_diff, 2);

	//hrt.stop();
	//hrt.printMilliseconds();

	valid_ = computed_covariance;

	if (!valid_){
		printf("Motion model is not valid\n");
	}
}

Eigen::Vector3f MotionModel::mean_displacement() const {
  //printf("Time diff: %lf\n", time_diff_);
	return mean_velocity_.cast<float>() * time_diff_;
}


MotionModel::MotionModel()
	:pdf_constant_(1),
	 min_score_(1e-4),
   valid_eigen_vectors_(false),
	 valid_(false)
{

	//R_t in the kalman filter - the amount of uncertainty propagation in 0.1 s
	double xy_variance = kPropagationVariance; //1; //0.25;
	double z_variance = kPropagationVarianceZ; //0.1
	covariance_propagation_uncertainty_ = Eigen::Matrix3d::Zero();
	covariance_propagation_uncertainty_(0,0) = xy_variance;
	covariance_propagation_uncertainty_(1,1) = xy_variance;
	covariance_propagation_uncertainty_(2,2) = z_variance;

  //PRINT_INFO_STREAM("Initial covariance propagation uncertainty: " << endl << covariance_propagation_uncertainty_);

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

