/*
 * precision_tracker.cpp
 *
 *  Created on: May 12, 2014
 *      Author: davheld
 *
 */


#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <precision_tracking/down_sampler.h>
#include <precision_tracking/density_grid_2d_evaluator.h>
#include <precision_tracking/density_grid_3d_evaluator.h>
#include <precision_tracking/lf_rgbd_6d_evaluator.h>
#include <precision_tracking/precision_tracker.h>


namespace precision_tracking {

namespace {

using std::pair;
using std::max;

} // namespace

PrecisionTracker::PrecisionTracker(const Params *params)
  : params_(params),
    adh_tracker3d_(params_),
    down_sampler_(params_->stochastic_downsample, params_)
{
  if (params_->useColor) {
    alignment_evaluator_.reset(new LF_RGBD_6D_Evaluator(params_));
  } else if (params_->use3D){
    alignment_evaluator_.reset(new DensityGrid3dEvaluator(params_));
  } else {
    alignment_evaluator_.reset(new DensityGrid2dEvaluator(params_));
  }
}


PrecisionTracker::~PrecisionTracker()
{
  // TODO Auto-generated destructor stub
}

void PrecisionTracker::track(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
    const double sensor_horizontal_resolution_actual,
    const double sensor_vertical_resolution_actual,
    const MotionModel& motion_model,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms)
{
  // Estimate the search range for alignment.
  std::pair <double, double> xRange;
  std::pair <double, double> yRange;
  std::pair <double, double> zRange;
  estimateRange(current_points, prev_points, &xRange, &yRange, &zRange);

  // Compute the centroid.
  Eigen::Vector4f current_points_centroid_4d;
  pcl::compute3DCentroid (*current_points, current_points_centroid_4d);
  const Eigen::Vector3f current_points_centroid =
      current_points_centroid_4d.head(3);

  // Down-sample the previous points.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr previous_model_downsampled(
        new pcl::PointCloud<pcl::PointXYZRGB>);
  down_sampler_.downSamplePoints(
        prev_points, params_->kPrevFrameDownsample, previous_model_downsampled);

  // Compute the ratio by which we down-sampled, which decreases the effective
  // resolution.
  const double down_sample_factor_prev =
      static_cast<double>(previous_model_downsampled->size()) /
      static_cast<double>(prev_points->size());

  // Down-sample the current points.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr down_sampled_current(
        new pcl::PointCloud<pcl::PointXYZRGB>);
  down_sampler_.downSamplePoints(
        current_points, params_->kCurrFrameDownsample, down_sampled_current);

  // The effective resolution = resolution / downsample factor.
  const double sensor_horizontal_res =
      sensor_horizontal_resolution_actual / down_sample_factor_prev;
  const double sensor_vertical_res =
      sensor_vertical_resolution_actual / down_sample_factor_prev;

  // Align the current points to the previous points using the annealed
  // dynamic histogram tracker.
  adh_tracker3d_.track(
        params_->kInitialXYSamplingResolution, params_->kInitialZSamplingResolution,
        xRange, yRange, zRange,
        down_sampled_current, previous_model_downsampled,
        current_points_centroid, motion_model,
        sensor_horizontal_res, sensor_vertical_res,
        alignment_evaluator_, scored_transforms);
}

void PrecisionTracker::estimateRange(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
    std::pair <double, double>* xRange,
    std::pair <double, double>* yRange,
    std::pair <double, double>* zRange) const
{
  // Compute the displacement of the centroid.
  Eigen::Matrix4f centroidDiffTransform =
      estimateAlignmentCentroidDiff(current_points, prev_points);

  // Find the min and max of the previous points.
  pcl::PointXYZRGB max_pt_prev;
  pcl::PointXYZRGB min_pt_prev;
  pcl::getMinMax3D(*prev_points, min_pt_prev, max_pt_prev);

  // Compute the size of the previous points.
  const double x_diff_prev = max_pt_prev.x - min_pt_prev.x;
  const double y_diff_prev = max_pt_prev.y - min_pt_prev.y;

  // Find the min and max of the previous points.
  pcl::PointXYZRGB max_pt_curr;
  pcl::PointXYZRGB min_pt_curr;
  pcl::getMinMax3D(*current_points, min_pt_curr, max_pt_curr);

  // Compute the size of the current points.
  const double x_diff_curr = max_pt_curr.x - min_pt_curr.x;
  const double y_diff_curr = max_pt_curr.y - min_pt_curr.y;

  // Compute the maximum size of the object.
  const double x_diff = max(x_diff_prev, x_diff_curr);
  const double y_diff = max(y_diff_prev, y_diff_curr);
  const double size = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

  // The object can have moved a maximum of size / 2 from the displacement
  // of the centroid.
  const double maxX = ceil(size / 2);
  const double maxY = ceil(size / 2);

  // We center our search window on an alignment of the centroids
  // of the points from the previous and current frames.
  const double x_init = centroidDiffTransform(0,3);
  const double y_init = centroidDiffTransform(1,3);

  // Assume that the vertical motion is minimal.
  const double z_init = 0;

  *xRange = std::make_pair(-maxX + x_init, maxX + x_init);
  *yRange = std::make_pair(-maxY + y_init, maxY + y_init);
  *zRange = std::make_pair(-params_->maxZ + z_init, params_->maxZ + z_init);
}

Eigen::Matrix4f PrecisionTracker::estimateAlignmentCentroidDiff(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& curr_points,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points) const
{
  Eigen::Vector4f new_centroid;
  pcl::compute3DCentroid (*curr_points, new_centroid);

  Eigen::Vector4f old_centroid;
  pcl::compute3DCentroid (*prev_points, old_centroid);

  const Eigen::Vector4f centroidDiff =  old_centroid - new_centroid;

  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();

  guess(0,3) = centroidDiff(0);
  guess(1,3) = centroidDiff(1);

  // The vertical motion is assumed to be small.
  guess(2,3) = 0;

  return guess;
}

} // namespace precision_tracking
