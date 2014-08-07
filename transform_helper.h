/*
 * transform_helper.h
 *
 *  Created on: Jan 16, 2013
 *      Author: davheld
 */

#ifndef TRANSFORM_HELPER_H_
#define TRANSFORM_HELPER_H_

#include <Eigen/Eigen>

struct TransformComponents{
	TransformComponents()
		:x(0), y(0), z(0), roll(0), pitch(0), yaw(0)
	{
	}

	float x,y,z,roll,pitch,yaw;
};

class TransformHelper {
public:
	TransformHelper();
	virtual ~TransformHelper();

	static TransformComponents computeComponents(const Eigen::Affine3f& full_transform, const Eigen::Vector4d& centroid);
	static TransformComponents computeComponents(const Eigen::Affine3f& real_transform);
	static Eigen::Affine3f computeRealTransform(const Eigen::Affine3f& full_transform, const Eigen::Vector4d& centroid);

};

#endif /* TRANSFORM_HELPER_H_ */
