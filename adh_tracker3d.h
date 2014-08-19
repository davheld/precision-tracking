/*
 * adh_tracker3d.h
 *
 *  Created on: Sep 1, 2013
 *      Author: davheld
 *
 * Annealed dynamic histogram tracker
 * Starts by coarsely sampling from the state space, while inflating
 * the measurement model to avoid local minima.  We then sample more
 * finely, and anneal the measurement model accordingly, until
 * we are sampling finely and using the true measurement model.
 *
 */

#ifndef ADH_TRACKER_3D_H_
#define ADH_TRACKER_3D_H_

#include <utility>

#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "motion_model.h"
#include "scored_transform.h"
#include "density_grid_tracker.h"
#include "lf_discrete_3d.h"
#include "lf_rgbd_6d.h"
#include "fast_functions.h"

class ADHTracker3d {
public:
  ADHTracker3d();
  virtual ~ADHTracker3d();

	void track(
      const double initial_xy_sampling_resolution,
      const double initial_z_sampling_resolution,
      const std::pair <double, double>& xRange,
	    const std::pair <double, double>& yRange,
	    const std::pair <double, double>& zRange,
	    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > current_points,
	    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > prev_points,
	    const Eigen::Vector3f &current_points_centroid,
	    const MotionModel& motion_model,
	    const double horizontal_distance,
	    const double down_sample_factor_prev,
      const double xy_sensor_resolution,
      const double z_sensor_resolution,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms);

private:
  // Create a list of candidate xyz transforms.
  void createCandidateXYZTransforms(
      const double xy_sampling_resolution,
      const double z_sampling_resolution,
      const std::pair <double, double>& xRange,
      const std::pair <double, double>& yRange,
      const std::pair <double, double>& zRange_orig,
      std::vector<XYZTransform>* transforms) const;

	void recomputeProbs(
	    const double prior_prob,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms) const;

  void makeNewTransforms3D(
      const double new_xy_resolution, const double new_z_resolution,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms,
      std::vector<XYZTransform>* new_xyz_transforms,
      double* total_recomputing_prob) const;

  LFDiscrete3d& lf_discrete_3d_;
  LF_RGBD_6D lf_rgbd_6d_;
  DensityGridTracker& density_grid_tracker_;
  const FastFunctions& fast_functions_;
};

#endif /* ADH_TRACKER_3D_H_ */
