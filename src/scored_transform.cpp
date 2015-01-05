/*
 * scored_transform.cpp
 *
 *  Created on: Sep 17, 2013
 *      Author: davheld
 *
 */

#include <precision_tracking/scored_transform.h>

using std::vector;

namespace precision_tracking {

ScoredTransform6D::~ScoredTransform6D() {
  // TODO Auto-generated destructor stub
}

ScoredTransformXYZ::~ScoredTransformXYZ() {
  // TODO Auto-generated destructor stub
}


void ScoredTransformXYZ::getEigen(Eigen::Vector3f* translation) {
  (*translation)(0) = x_;
  (*translation)(1) = y_;
  (*translation)(2) = z_;
}

// Helper function.
bool compareTransforms(const ScoredTransform& transform_i,
                       const ScoredTransform& transform_j)
{
  const double score_i = transform_i.getUnnormalizedLogProb() -
      log(transform_i.getVolume());
  const double score_j = transform_j.getUnnormalizedLogProb() -
      log(transform_j.getVolume());
  return score_i > score_j;
}

KahanAccumulation KahanSum(KahanAccumulation accumulation, double value)
{
    KahanAccumulation result;
    double y = value - accumulation.correction;
    double t = accumulation.sum + y;
    result.correction = (t - accumulation.sum) - y;
    result.sum = t;
    return result;
}

} // namespace precision_tracking
