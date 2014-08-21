/*
 * transform_helper.cpp
 *
 *  Created on: Jan 16, 2013
 *      Author: davheld
 */

#include "transform_helper.h"

#include <stdint.h>
#include <pcl/common/eigen.h>

using namespace Eigen;

Eigen::Affine3f TransformHelper::computeRealTransform(
		const Eigen::Affine3f& full_transform, const Eigen::Vector4d& centroid) {

	Transform<float, 3, Affine> addCentroid;
	addCentroid = Translation<float, 3>(centroid(0), centroid(1), centroid(2));

	return addCentroid.inverse() * full_transform * addCentroid;

}

TransformComponents TransformHelper::computeComponents(const Eigen::Affine3f& real_transform){

	TransformComponents components;

	pcl::getTranslationAndEulerAngles(real_transform,
			components.x, components.y, components.z,
			components.roll, components.pitch, components.yaw);

	return components;

}


TransformComponents TransformHelper::computeComponents(
		const Eigen::Affine3f& full_transform, const Eigen::Vector4d& centroid) {
	Eigen::Affine3f real_transform  =
			computeRealTransform(full_transform, centroid);

	return computeComponents(real_transform);

}

TransformHelper::TransformHelper() {
	// TODO Auto-generated constructor stub

}

TransformHelper::~TransformHelper() {
	// TODO Auto-generated destructor stub
}
