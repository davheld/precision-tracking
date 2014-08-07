/*
 * ScoredTransform.h
 *
 *  Created on: Sep 17, 2013
 *      Author: davheld
 */

#ifndef SCOREDTRANSFORM_H_
#define SCOREDTRANSFORM_H_

#include <cstdio>
#include <vector>

#include <Eigen/Eigen>
#include "fast_functions.h"

namespace {

FastFunctions& fast_functions1_ = FastFunctions::getInstance();

} //namespace

class ScoredTransform {
public:
  ScoredTransform() {
  }

  ScoredTransform(const double log_prob)
    : unnormalized_log_prob_(log_prob)
  {
  }

  double getUnnormalizedLogProb() const  { return unnormalized_log_prob_; }

  void setUnnormalizedLogProb(const double new_log_prob) {
    unnormalized_log_prob_ = new_log_prob;
  }

protected:
  // Unnormalized Log probability.
  double unnormalized_log_prob_;

};

// A transform in the ground plane and its associated log probability score.
class ScoredTransformXYZ : public ScoredTransform {
public:
  ScoredTransformXYZ(
      const double x,
      const double y,
      const double z,
      const double log_prob,
      const double volume)
    : ScoredTransform(log_prob),
      x_(x),
      y_(y),
      z_(z),
      volume_(volume)
  {
  }

  ScoredTransformXYZ() {
  }

  virtual ~ScoredTransformXYZ();

  // Getters.
  double getX() const       { return x_;    }
  double getY() const       { return y_;    }
  double getZ() const       { return z_;    }
  double getVolume() const    { return volume_; }

protected:
  // Translation parameters (meters).
  double x_, y_, z_;
  double volume_;
};

// A transform and its associated log probability score.
// We assume that we rotate first about the centroid, and then translate.
class ScoredTransform6D : public ScoredTransformXYZ {
public:
  ScoredTransform6D(
      const double x,
      const double y,
      const double z,
      const double roll,
      const double pitch,
      const double yaw,
      const double log_prob,
      const double volume)
    : ScoredTransformXYZ(x, y, z, log_prob, volume),
      roll_(roll),
      pitch_(pitch),
      yaw_(yaw)
  {
  }

  ScoredTransform6D() {
  }

  virtual ~ScoredTransform6D();

  void print() const {
    printf("x,y,z,roll,pitch,yaw: %lf, %lf, %lf, %lf, %lf, %lf\n", x_, y_, z_,
        roll_, pitch_, yaw_);
  }

  // Getters.
  double getRoll() const    { return roll_;    }
  double getPitch() const  { return pitch_;    }
  double getYaw() const    { return yaw_;    }

private:
  // Rotation parameters (radians?).
  double roll_, pitch_, yaw_;
};

namespace {

bool compareTransforms(const ScoredTransformXYZ& transform_i,
                       const ScoredTransformXYZ& transform_j)
{
  const double score_i = transform_i.getUnnormalizedLogProb() -
      fast_functions1_.getFastLog(transform_i.getVolume());
  const double score_j = transform_j.getUnnormalizedLogProb() -
      fast_functions1_.getFastLog(transform_j.getVolume());
  return score_i > score_j;
}

} // namespace

// Collection of Scored Transforms.
template <class TransformType>
class ScoredTransforms {
public:
  ScoredTransforms(const std::vector<TransformType>& scored_transforms)
    : scored_transforms_(scored_transforms)
    {}

  ScoredTransforms() {}

  void findBest(TransformType* best_transform) const;

  void setScoredTransforms(const std::vector<TransformType>& scored_transforms) {
    scored_transforms_ = scored_transforms;
  }

  void setScoredTransforms(const ScoredTransforms& scored_transforms) {
    scored_transforms_ = scored_transforms.getScoredTransforms();
  }

  void addScoredTransforms(const ScoredTransforms& scored_transforms) {
    scored_transforms_.insert(
        scored_transforms_.end(),
        scored_transforms.getScoredTransforms().begin(),
        scored_transforms.getScoredTransforms().end());
  }


  const std::vector<double>& getNormalizedProbs() const;

  const std::vector<TransformType>& getScoredTransforms() const {
    return scored_transforms_;
  }

  // Non-const version.
  std::vector<TransformType>& getScoredTransforms() {
    return scored_transforms_;
  }

  void clear() {
    scored_transforms_.clear();
    normalized_probs_.clear();
    normalized_probs_approx_.clear();
  }

  void reserve(const size_t n) {
    scored_transforms_.reserve(n);
    normalized_probs_.reserve(n);
    normalized_probs_approx_.reserve(n);
  }

  void addScoredTransform(const TransformType& transform) {
    scored_transforms_.push_back(transform);
  }

  void resize(size_t n) {
    scored_transforms_.resize(n);
    normalized_probs_.reserve(n);
    normalized_probs_approx_.reserve(n);
  }

  void set(const TransformType& scored_transform, const int i) {
    scored_transforms_[i] = scored_transform;
  }

  void sortDescending() {
    std::sort(scored_transforms_.begin(), scored_transforms_.end(),
              compareTransforms);
  }

private:
  // A collection of transforms and their scores.
  std::vector<TransformType> scored_transforms_;

  // A normalized probability for each transform.
  mutable std::vector<double> normalized_probs_;

  // A normalized probability for each transform.
  mutable std::vector<double> normalized_probs_approx_;

};

template <class TransformType>
void ScoredTransforms<TransformType>::findBest(
    TransformType* best_transform) const {
  int best_transform_index = -1;
  double best_score = -std::numeric_limits<double>::max();

  for (size_t i = 0; i < scored_transforms_.size(); ++i) {
    const double score = scored_transforms_[i].getUnnormalizedLogProb() -
        fast_functions1_.getFastLog(scored_transforms_[i].getVolume());

    if (score > best_score) {
      best_score = score;
      best_transform_index = i;
    }
  }

  if (best_transform_index == -1) {
    printf("Error - no best transform!");
    exit(1);
  }

  *best_transform = scored_transforms_[best_transform_index];
}


template <class TransformType>
const std::vector<double>& ScoredTransforms<TransformType>::getNormalizedProbs() const {
  const size_t num_transforms = scored_transforms_.size();

  if (normalized_probs_.size() != num_transforms) {
    // We need to recompute the normalized probabilities.

    // First obtain a list of all probabilities.
    std::vector<double> log_probabilities(num_transforms);
    for (size_t i = 0; i < num_transforms; ++i) {
      log_probabilities[i] = scored_transforms_[i].getUnnormalizedLogProb();
    }

    // Make all the scores positive and normalized.

    // Find the max log probablity.
    const double max_log_prob = *std::max_element(
        log_probabilities.begin(), log_probabilities.end());

    // Find the value such that max_log_prob + n_factor * log(10) = max_x.
    // This is equivalent to: prob * 10 ^ n_factor = max_x.
    const double max_x = 0;
    const double n_factor = (max_x - max_log_prob) / log(10);

    // Multiply all probabilities by 10 ^ n_factor, and then find the
    // (not log) probability.
    normalized_probs_.clear();
    normalized_probs_.reserve(num_transforms);
    for (size_t i = 0; i < num_transforms; ++i) {
      const double prob = exp(log_probabilities[i] + n_factor * log(10));
      normalized_probs_.push_back(prob);
    }

    // Normalize.
    //const double sum_prob =
  //      std::accumulate(probabilities->begin(), probabilities->end(), 0);
    double sum_prob2 = 0;
    for (size_t i = 0; i < num_transforms; ++i) {
      sum_prob2 += normalized_probs_[i];
    }

    // Verfify that something bad didn't happen.
    if (sum_prob2 == 0) {
      printf("Log probs:\n");
      for (size_t i = 0; i < num_transforms; ++i) {
        printf("%lf\n", log_probabilities[i]);
      }
      printf("Probs:\n");
      for (size_t i = 0; i < num_transforms; ++i) {
        printf("%lf\n", normalized_probs_[i]);
      }
      exit(1);
    }

    // Normalize the probabilities.
    for (size_t i = 0; i < num_transforms; ++i) {
      normalized_probs_[i] /= sum_prob2;
    }
  }

  return normalized_probs_;
}


#endif /* SCOREDTRANSFORM_H_ */
