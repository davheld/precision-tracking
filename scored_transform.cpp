/*
 * ScoredTransform.cpp
 *
 *  Created on: Sep 17, 2013
 *      Author: davheld
 */

#include "scored_transform.h"

using std::vector;

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
