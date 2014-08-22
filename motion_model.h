/*
 * motion_model.h
 *
 *  Created on: Jan 23, 2013
 *      Author: davheld
 */

#ifndef MOTION_MODEL_H_
#define MOTION_MODEL_H_

#include <vector>

#include <Eigen/Eigen>

#include "scored_transform.h"

struct TransformComponents{
  TransformComponents()
    :x(0), y(0), z(0), roll(0), pitch(0), yaw(0)
  {
  }

  float x,y,z,roll,pitch,yaw;
};

class MotionModel {
public:
  MotionModel();
	virtual ~MotionModel();

  void addTransformsWeightedGaussian(
      const ScoredTransforms<ScoredTransformXYZ>& transforms,
      const double& time_diff);

	// Add centroid diff and update the Kalman filter.
  void addCentroidDiff(const Eigen::Vector3f& centroid_diff,
                       const double time_diff);

	void propagate(const double& time_diff);

	double computeScore(const TransformComponents& components) const;

  // Compute the score given the x,y, and z components.
  double computeScore(const double x, const double y, const double z) const;

  Eigen::Vector3f get_mean_velocity() const {
    return mean_velocity_.cast<float>();
  }

	bool valid() const { return valid_; }

	const Eigen::Vector3d& get_mean_delta_position() const {
	  return mean_delta_position_;
	}

	const Eigen::Matrix3d get_covariance_delta_position() const {
	  return covariance_delta_position_;
	}

  const Eigen::Matrix3d get_covariance_delta_position_inv() const {
    return covariance_delta_position_inv_;
  }

	double get_min_score() {
	  return min_score_;
	}

private:
  Eigen::Vector3d computeMeanVelocity(
      const ScoredTransforms<ScoredTransformXYZ>& transforms,
      const double time_diff) const;

  Eigen::Matrix3d computeCovarianceVelocity(
      const ScoredTransforms<ScoredTransformXYZ>& transforms,
      const double time_diff,
      const Eigen::Vector3d& mean_velocity) const;

	Eigen::Vector3d mean_velocity_;
	Eigen::Matrix3d covariance_velocity_;

	Eigen::Vector3d mean_delta_position_;
	Eigen::Matrix3d covariance_delta_position_;
	Eigen::Matrix3d covariance_delta_position_inv_;

  double pdf_constant_;
	double min_score_;

	bool valid_eigen_vectors_;
	Eigen::Matrix3d eigen_vectors_;

	//R_t in the kalman filter - the amount of uncertainty propagation in 0.1 s
	Eigen::Matrix3d covariance_propagation_uncertainty_;

	bool valid_;
};

#endif /* MOTION_MODEL_H_ */
