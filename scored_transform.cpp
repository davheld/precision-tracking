/*
 * ScoredTransform.cpp
 *
 *  Created on: Sep 17, 2013
 *      Author: davheld
 */

#include <cstdio>

#include "scored_transform.h"

using std::vector;

namespace {

// Return the transform with the larger score.
bool transformComparator(
    const ScoredTransform& transform1,
    const ScoredTransform& transform2) {
  return transform1.getUnnormalizedLogProb() >
          transform2.getUnnormalizedLogProb();
}

}  // namespace

ScoredTransform6D::~ScoredTransform6D() {
  // TODO Auto-generated destructor stub
}

ScoredTransformXYZ::~ScoredTransformXYZ() {
  // TODO Auto-generated destructor stub
}


