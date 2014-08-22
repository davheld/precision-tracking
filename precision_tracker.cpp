/*
 * precision_tracker.cpp
 *
 *  Created on: May 12, 2014
 *      Author: davheld
 */

#include "precision_tracker.h"

#include <boost/math/constants/constants.hpp>
#include <pcl/common/common.h>

#include "down_sampler.h"
#include "density_grid_tracker.h"
#include "lf_rgbd_6d.h"

namespace {

// Whether to include color probabilities when performing the alignment.
const bool use_color = false;

// We downsample the current frame of the tracked object to have this many
// points.
const int kCurrFrameDownsample = 150;

// We downsample the previous frame of the tracked object to have this many
// points.
const int kPrevFrameDownsample = 2000;

// Whether to deterministically or stochastically downsample points from
// the tracked object.
const bool stochastic_downsample = false;

// Set to 0 so we do not search over z (for objects moving in urban settings,
// the vertical motion is small).
const double maxZ = 0;

// We expect the actual motion of the previous centroid to be
// this far off from the measured displacement of the centroid (due to
// occlusions and viewpoint changes).  Roughly, we set this to be equal
// to half of the expected size of the tracked object.
const double maxXY = 2;

// Start with a very coarse xy sampling for efficiency.
const double kInitialXYSamplingResolution = 1;

// Do not sample in the z-direction - assume minimal vertical motion.
const double kInitialZSamplingResolution = 0;

const double pi = boost::math::constants::pi<double>();

using std::pair;
using std::max;

} // namespace

PrecisionTracker::PrecisionTracker()
  : down_sampler_(stochastic_downsample)
{
  if (use_color) {
    alignment_evaluator_.reset(new LF_RGBD_6D);
  } else {
    alignment_evaluator_.reset(new DensityGridTracker);
  }
}

PrecisionTracker::~PrecisionTracker() {
  // TODO Auto-generated destructor stub
}

void PrecisionTracker::track(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
    const double horizontal_distance,
    const MotionModel& motion_model,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms) {
  // Estimate the search range for alignment.
  std::pair <double, double> xRange;
  std::pair <double, double> yRange;
  std::pair <double, double> zRange;
  estimateRange(current_points, prev_points, &xRange, &yRange, &zRange);

  // Compute the centroid.
  Eigen::Vector3f current_points_centroid;
  computeCentroid(current_points, &current_points_centroid);

  // Down-sample the previous points.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr previous_model_downsampled(
        new pcl::PointCloud<pcl::PointXYZRGB>);
  down_sampler_.downSamplePoints(
        prev_points, kPrevFrameDownsample, previous_model_downsampled);

  // Compute the ratio by which we down-sampled, which decreases the effective
  // resolution.
  const double down_sample_factor_prev =
      static_cast<double>(previous_model_downsampled->size()) /
      static_cast<double>(prev_points->size());

  // Down-sample the current points.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr down_sampled_current(
        new pcl::PointCloud<pcl::PointXYZRGB>);
  down_sampler_.downSamplePoints(
        current_points, kCurrFrameDownsample, down_sampled_current);

  // Compute the sensor horizontal resolution
  const double velodyne_horizontal_res_actual =
      2 * horizontal_distance * tan(.18 / 2.0 * pi / 180.0);

  // The effective resolution = resolution / downsample factor.
  const double velodyne_horizontal_res =
      velodyne_horizontal_res_actual / down_sample_factor_prev;

  // The vertical resolution for the Velodyne is 2.2 * horizontal resolution.
  const double velodyne_vertical_res = 2.2 * velodyne_horizontal_res;

  // Align the previous points to the current points using the annealed
  // dynamic histogram trakcer.
  adh_tracker3d_.track(
        kInitialXYSamplingResolution, kInitialZSamplingResolution,
        xRange, yRange, zRange,
        down_sampled_current, previous_model_downsampled,
        current_points_centroid, motion_model,
        velodyne_horizontal_res, velodyne_vertical_res,
        alignment_evaluator_, scored_transforms);
}

void PrecisionTracker::estimateRange(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
    std::pair <double, double>* xRange,
    std::pair <double, double>* yRange,
    std::pair <double, double>* zRange) const {
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
  *zRange = std::make_pair(-maxZ + z_init, maxZ + z_init);
}

Eigen::Matrix4f PrecisionTracker::estimateAlignmentCentroidDiff(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& curr_points,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points) const {
  Eigen::Vector3f newCentroid;
  computeCentroid(curr_points, &newCentroid);

  Eigen::Vector3f oldCentroid;
  computeCentroid(prev_points, &oldCentroid);

  Eigen::Vector3f centroidDiff =  oldCentroid - newCentroid;

  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();

  guess(0,3) = centroidDiff(0);
  guess(1,3) = centroidDiff(1);

  // The vertical motion is assumed to be small.
  guess(2,3) = 0;

  return guess;
}

void PrecisionTracker::computeCentroid(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& points,
    Eigen::Vector3f* centroid) {
  *centroid = Eigen::Vector3f::Zero();

  for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = points->begin();
       it < points->end(); it++) {
    pcl::PointXYZRGB p = *it;
    (*centroid)(0) += p.x;
    (*centroid)(1) += p.y;
    (*centroid)(2) += p.z;
  }

  *centroid /= static_cast<double>(points->size());
}
