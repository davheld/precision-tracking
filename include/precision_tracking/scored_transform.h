/*
 * scored_transform.h
 *
 *  Created on: Sep 17, 2013
 *      Author: davheld
 *
 * Classes representing point cloud transforms and associated metadata.
 *
 */

#ifndef __PRECISION_TRACKING__SCORED_TRANSFORM_H_
#define __PRECISION_TRACKING__SCORED_TRANSFORM_H_

#include <cstdio>
#include <vector>
#include <numeric>

#include <Eigen/Eigen>

namespace precision_tracking {

// A pure translation, which represents a proposed alignment between
// the points in the current frame to the previuos frame.
// The volume is the size of the discretized region of the
// state space represented by this transform.
struct XYZTransform {
public:
  double x, y, z;
  double volume;
  XYZTransform(
      const double& x,
      const double& y,
      const double& z,
      const double& volume)
    :x(x),
     y(y),
     z(z),
     volume(volume)
  {  }
};

// Base class for a scored transform, which stores the unnormalized log
// probability of a given alignment.
class ScoredTransform {
public:
  ScoredTransform() {
  }

  ScoredTransform(const double log_prob,
                  const double volume)
    : unnormalized_log_prob_(log_prob),
      volume_(volume)
  {
  }

  double getUnnormalizedLogProb() const  { return unnormalized_log_prob_; }

  void setUnnormalizedLogProb(const double new_log_prob) {
    unnormalized_log_prob_ = new_log_prob;
  }

  double getVolume() const { return volume_; }


protected:
  // Unnormalized Log probability.
  double unnormalized_log_prob_;

  // The volume is the size of the discretized region of the
  // state space represented by this transform.
  double volume_;

};

// A translation and its associated log probability.
class ScoredTransformXYZ : public ScoredTransform {
public:
  ScoredTransformXYZ(
      const double x,
      const double y,
      const double z,
      const double log_prob,
      const double volume)
    : ScoredTransform(log_prob, volume),
      x_(x),
      y_(y),
      z_(z)
  {
  }

  ScoredTransformXYZ() {
  }

  virtual ~ScoredTransformXYZ();

  // Convert the translation to an Eigen vector.
  void getEigen(Eigen::Vector3f* translation);

  // Getters.
  double getX() const       { return x_;    }
  double getY() const       { return y_;    }
  double getZ() const       { return z_;    }

protected:
  // Translation parameters (meters).
  double x_, y_, z_;
};

// A transform and its associated log probability score.
// We assume that we rotate the object first about the centroid and
// then translate.
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
  // Rotation parameters (in radians).
  double roll_, pitch_, yaw_;
};

// Helper function for sorting.
bool compareTransforms(const ScoredTransform& transform_i,
                       const ScoredTransform& transform_j);

// A collection of Scored Transforms.
template <class TransformType>
class ScoredTransforms {
public:
  ScoredTransforms(const std::vector<TransformType>& scored_transforms)
    : scored_transforms_(scored_transforms)
    {}

  ScoredTransforms() {}

  // Sort the transforms in descending order of probability density
  // (probability per unit volume).
  void sortDescending() {
    std::sort(scored_transforms_.begin(), scored_transforms_.end(),
              compareTransforms);
  }

  // Finds the highest scoring transform in terms of probability density
  // (probability per unit volume), which is the mode of the distribution.
  void findBest(TransformType* best_transform,
                double* best_probability_density) const;

  // Append scored transforms to the end of the current list.
  void appendScoredTransforms(const ScoredTransforms& scored_transforms) {
    scored_transforms_.insert(
        scored_transforms_.end(),
        scored_transforms.getScoredTransforms().begin(),
        scored_transforms.getScoredTransforms().end());
  }

  // Normalize the probabilities for all of the scored transforms so they
  // sum to 1, and return the resulting list of probabilities.
  const std::vector<double> getNormalizedProbs() const;

  const std::vector<TransformType>& getScoredTransforms() const {
    return scored_transforms_;
  }

  // Non-const version.
  std::vector<TransformType>& getScoredTransforms() {
    return scored_transforms_;
  }

  void setScoredTransforms(
      const std::vector<TransformType>& scored_transforms) {
    scored_transforms_ = scored_transforms;
  }

  void setScoredTransforms(const ScoredTransforms& scored_transforms) {
    scored_transforms_ = scored_transforms.getScoredTransforms();
  }

  void clear() {
    scored_transforms_.clear();
  }

  void reserve(const size_t n) {
    scored_transforms_.reserve(n);
  }

  void addScoredTransform(const TransformType& transform) {
    scored_transforms_.push_back(transform);
  }

  void resize(size_t n) {
    scored_transforms_.resize(n);
  }

  void set(const TransformType& scored_transform, const int i) {
    scored_transforms_[i] = scored_transform;
  }

private:
  // A collection of transforms and their scores.
  std::vector<TransformType> scored_transforms_;
};

template <class TransformType>
void ScoredTransforms<TransformType>::findBest(
    TransformType* best_transform, double* best_probability_density) const {
  int best_transform_index = -1;
  double best_score = -std::numeric_limits<double>::max();

  const std::vector<double>& normalized_probs = getNormalizedProbs();

  for (size_t i = 0; i < scored_transforms_.size(); ++i) {
    const double prob = normalized_probs[i];

    // Compute the unnormalized log probability density of this transform.
    const double prob_density = prob / scored_transforms_[i].getVolume();

    // Find the transform with the highest unnormalized log probability density.
    if (prob_density > best_score) {
      best_score = prob_density;
      best_transform_index = i;
    }
  }

  if (best_transform_index == -1) {
    printf("Error - no best transform!");
    exit(1);
  }

  *best_transform = scored_transforms_[best_transform_index];
  *best_probability_density = best_score;
}

struct KahanAccumulation
{
    double sum;
    double correction;
};

KahanAccumulation KahanSum(KahanAccumulation accumulation, double value);

template <class TransformType>
const std::vector<double> ScoredTransforms<TransformType>::getNormalizedProbs() const {
  // Allocate vector to store the normalized probabilities.
  const size_t num_transforms = scored_transforms_.size();
  std::vector<double> normalized_probs;
  normalized_probs.reserve(num_transforms);

  // First obtain a list of all probabilities.
  std::vector<double> log_probabilities(num_transforms);
  for (size_t i = 0; i < num_transforms; ++i) {
    log_probabilities[i] = scored_transforms_[i].getUnnormalizedLogProb();
  }

  // Make all the scores positive and normalized.

  // Find the max log probablity.
  const double max_log_prob = *std::max_element(
      log_probabilities.begin(), log_probabilities.end());

  // Subtract the max to bring the highest probabilities up to a reasonable
  // level, to avoid issues of numerical instability.  Then convert
  // from log prob to prob.
  for (size_t i = 0; i < num_transforms; ++i) {
    const double prob = exp(log_probabilities[i] - max_log_prob);
    normalized_probs.push_back(prob);
  }

  // Compute the normalization constant - for details see
  // http://stackoverflow.com/questions/10330002/sum-of-small-double-numbers-c
  const KahanAccumulation init = {0};
  const KahanAccumulation result =
      std::accumulate(normalized_probs.begin(), normalized_probs.end(), init,
                      KahanSum);
  const double sum_prob = result.sum;

  // Verfify that something bad didn't happen.
  if (sum_prob == 0) {
    // Something bad happened - print lots of info and quit.
    printf("Log probs:\n");
    for (size_t i = 0; i < num_transforms; ++i) {
      printf("%lf\n", log_probabilities[i]);
    }
    printf("Probs:\n");
    for (size_t i = 0; i < num_transforms; ++i) {
      printf("%lf\n", normalized_probs[i]);
    }
    exit(1);
  }

  // Normalize the probabilities.
  for (size_t i = 0; i < num_transforms; ++i) {
    normalized_probs[i] /= sum_prob;
  }

  return normalized_probs;
}

} // namespace precision_tracking

#endif /* __PRECISION_TRACKING__SCORED_TRANSFORM_H_ */
