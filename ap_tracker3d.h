/* Probabilistic Filtering Tracker
 *
 * ap_tracker.h
 *
 *  Created on: Sep 1, 2013
 *      Author: davheld
 */

#ifndef AP_TRACKER_3D_H_
#define AP_TRACKER_3D_H_

#include <utility>

#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "motion_model.h"
#include "scored_transform.h"
#include "density_grid_tracker.h"
//#include "NNTracker3d.h"
#include "lf_discrete_3d.h"
#include "fast_functions.h"

// Approximate probabilistic tracker - successively computes approximations
// to the posterior probability of each alignment.
class APTracker3d {
public:
  APTracker3d();
	virtual ~APTracker3d();

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
	void recomputeProbs(
	    const double prior_prob,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms) const;

  void makeNewTransforms3D(
      const double new_xy_resolution, const double new_z_resolution,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms,
      std::vector<XYZTransform>* new_xyz_transforms,
      double* total_recomputing_prob) const;

  LFDiscrete3d& lf_discrete_3d_;
  DensityGridTracker& density_grid_tracker_;
  const FastFunctions& fast_functions_;
};

#endif /* AP_TRACKER_3D_H_ */
