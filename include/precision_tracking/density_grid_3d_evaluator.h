/*
 * density_grid_3d_evaluator.h
 *
 *  Created on: Sep 1, 2013
 *      Author: davheld
 *
 * Compute the probability of a given set of alignments.
 * To do this quickly, we pre-cache probability values in a density grid for
 * fast lookups.  Because of the caching in the grid-structure, this
 * evaluator works best for translations only (no rotations) and does
 * not make use of color information.  If rotations or color are required,
 * use a different evaluator class.
 *
 */

#ifndef __PRECISION_TRACKING__DENSITY_GRID_3D_EVALUATOR_H_
#define __PRECISION_TRACKING__DENSITY_GRID_3D_EVALUATOR_H_

#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <precision_tracking/scored_transform.h>
#include <precision_tracking/alignment_evaluator.h>


struct ScoredTransform;
struct XYZTransform;
class MotionModel;

namespace precision_tracking {

class DensityGrid3dEvaluator : public AlignmentEvaluator {
public:
  DensityGrid3dEvaluator(const Params *params);
  virtual ~DensityGrid3dEvaluator();

private:
  void init(const double xy_sampling_resolution,
            const double z_sampling_resolution,
            const double sensor_horizontal_resolution,
            const double sensor_vertical_resolution,
            const size_t num_current_points);

  // Get the probability of this transform.
  double getLogProbability(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
      const Eigen::Vector3f& current_points_centroid,
      const MotionModel& motion_model,
      const double delta_x, const double delta_y, const double delta_z);

  void computeDensityGridParameters(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
      const double xy_sampling_resolution,
      const double z_sampling_resolution,
      const double xy_sensor_resolution,
      const double z_sensor_resolution);

  // Pre-cache probability values in a density grid for fast lookups.
  void computeDensityGrid(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points);

  // A grid used to pre-cache probability values for fast lookups.
  std::vector<std::vector<std::vector<double> >  > density_grid_;

  // The size of the resulting grid.
  int xSize_;
  int ySize_;
  int zSize_;

  // The step size of the density grid.
  double xy_grid_step_;
  double z_grid_step_;

  // The minimum point of the previous set of points used for tracking.
  pcl::PointXYZRGB min_pt_;

  // In our discrete grid, we want to compute the Gaussian probability
  // for this many grid cells away from each point.  As we get farther
  // away from the point, the probability of the Guassian goes to 0
  // so we only need to compute the probability at a limited number
  // of grid cells for each point.
  int num_spillover_steps_xy_;
  int num_spillover_steps_z_;
};

} // namespace precision_tracking

#endif /* __PRECISION_TRACKING__DENSITY_GRID_3D_EVALUATOR_H_ */

