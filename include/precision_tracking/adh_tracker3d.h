/*
 * adh_tracker3d.h
 *
 *  Created on: Sep 1, 2013
 *      Author: davheld
 *
 * Annealed dynamic histogram tracker.
 * Starts by coarsely sampling from the state space, while inflating
 * the measurement model to avoid local minima.  We then sample more
 * finely and anneal the measurement model accordingly, until
 * we are sampling finely and using the true measurement model.
 *
 */

#ifndef __PRECISION_TRACKING__ADH_TRACKER_3D_H_
#define __PRECISION_TRACKING__ADH_TRACKER_3D_H_

#include <utility>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <precision_tracking/alignment_evaluator.h>
#include <precision_tracking/motion_model.h>
#include <precision_tracking/scored_transform.h>
#include <precision_tracking/params.h>

namespace precision_tracking {

class ADHTracker3d {
public:
  explicit ADHTracker3d(const Params *params);
  virtual ~ADHTracker3d();

  // Estimate the posterior distribution over alignments sampled from the
  // proposed range in xRange, yRange, zRange.
	void track(
      const double initial_xy_sampling_resolution,
      const double initial_z_sampling_resolution,
      const std::pair <double, double>& xRange,
	    const std::pair <double, double>& yRange,
	    const std::pair <double, double>& zRange,
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
      const Eigen::Vector3f& current_points_centroid,
	    const MotionModel& motion_model,
      const double xy_sensor_resolution,
      const double z_sensor_resolution,
      boost::shared_ptr<AlignmentEvaluator> alignment_evaluator,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms) const;

private:
  const Params *params_;

  // Compute the joint probability of each cell and the region, given
  // the prior region probability.
	void recomputeProbs(
      const double prior_region_prob,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms) const;

  // Sample more finely in all regions above a certain threshold probability.
  void makeNewTransforms3D(
      const double new_xy_resolution, const double new_z_resolution,
      const double old_xy_sampling_resolution,
      const double old_z_sampling_resolution,
      ScoredTransforms<ScoredTransformXYZ>* scored_transforms,
      std::vector<XYZTransform>* new_xyz_transforms,
      double* total_recomputing_prob) const;

  // Create a list of candidate xyz transforms.
  void createCandidateXYZTransforms(
      const double xy_sampling_resolution,
      const double z_sampling_resolution,
      const std::pair <double, double>& xRange,
      const std::pair <double, double>& yRange,
      const std::pair <double, double>& zRange_orig,
      std::vector<XYZTransform>* transforms) const;
};

} // namespace precision_tracking

#endif /* __PRECISION_TRACKING__ADH_TRACKER_3D_H_ */
