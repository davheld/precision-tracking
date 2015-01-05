/*
 * motion_model.h
 *
 *  Created on: Jan 23, 2013
 *      Author: davheld
 *
 * Motion model used to predict the next location of a given object,
 * given a previously estimated motion.
 *
 */

#ifndef __PRECISION_TRACKING__MOTION_MODEL_H_
#define __PRECISION_TRACKING__MOTION_MODEL_H_

#include <vector>

#include <Eigen/Eigen>

#include <precision_tracking/scored_transform.h>
#include <precision_tracking/params.h>

namespace precision_tracking {

struct TransformComponents{
  TransformComponents()
    :x(0), y(0), z(0), roll(0), pitch(0), yaw(0)
  {
  }

  float x, y, z, roll, pitch, yaw;
};

class MotionModel {
public:
  explicit MotionModel(const Params *params);
	virtual ~MotionModel();

  void addTransformsWeightedGaussian(
      const ScoredTransforms<ScoredTransformXYZ>& transforms,
      const double& time_diff);

	// Add centroid diff and update the Kalman filter.
  void addCentroidDiff(const Eigen::Vector4f& centroid_diff,
                       const double time_diff);

	void propagate(const double& time_diff);

	double computeScore(const TransformComponents& components) const;

  // Compute the score given the x,y, and z components.
  double computeScore(const double x, const double y, const double z) const;

  Eigen::Vector3f get_mean_velocity() const {
    return mean_velocity_.cast<float>();
  }

  const Eigen::Matrix3d get_covariance_velocity() const {
    return covariance_velocity_;
  }

	bool valid() const { return valid_; }

    Eigen::Vector3d get_mean_delta_position() const {
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

  void setFlip(const bool flip) { if (flip) { flip_ = -1; } else { flip_ = 1; } }

private:
  Eigen::Vector3d computeMeanVelocity(
      const ScoredTransforms<ScoredTransformXYZ>& transforms,
      const double time_diff) const;

  Eigen::Matrix3d computeCovarianceVelocity(
      const ScoredTransforms<ScoredTransformXYZ>& transforms,
      const double time_diff,
      const Eigen::Vector3d& mean_velocity) const;

  const Params *params_;

	Eigen::Vector3d mean_velocity_;
	Eigen::Matrix3d covariance_velocity_;

	Eigen::Vector3d mean_delta_position_;
	Eigen::Matrix3d covariance_delta_position_;
	Eigen::Matrix3d covariance_delta_position_inv_;

  double pdf_constant_;
	double min_score_;

	//R_t in the kalman filter - the amount of uncertainty propagation in 0.1 s
	Eigen::Matrix3d covariance_propagation_uncertainty_;

	bool valid_;

    // Whether to flip the output of the motion model (-1 to flip, 1 not to flip).
    int flip_;
};

} // namespace precision_tracking

#endif /* __PRECISION_TRACKING__MOTION_MODEL_H_ */
