/*
 * precision_tracker.cpp
 *
 *  Created on: May 12, 2014
 *      Author: davheld
 */

#include "precision_tracker.h"

#include <boost/math/constants/constants.hpp>
#include "down_sampler.h"

namespace {

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

Eigen::Matrix4f makeRotationMatrix(const double& roll, const double& pitch, const double& yaw){
  Eigen::Matrix4f shift = Eigen::Matrix4f::Identity();

  //http://planning.cs.uiuc.edu/node102.html
  shift(0,0) = cos(yaw)*cos(pitch);
  shift(0,1) = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
  shift(0,2) = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll);

  shift(1,0) = sin(yaw)*cos(pitch);
  shift(1,1) = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);
  shift(1,2) = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);

  shift(2,0) = -sin(pitch);
  shift(2,1) = cos(pitch)*sin(roll);
  shift(2,2) = cos(pitch)*cos(roll);

  return shift;
}

Eigen::Matrix4f makeTranslationMatrix(const double& x, const double& y, const double& z){
  Eigen::Matrix4f translationMatrix = Eigen::Matrix4f::Identity();

  translationMatrix(0,3) = x;
  translationMatrix(1,3) = y;
  translationMatrix(2,3) = z;

  return translationMatrix;
}

Eigen::Matrix4f makeTransformationMatrix(const double& x, const double& y, const double& z,
    const double& roll, const double& pitch, const double& yaw,
    const Eigen::Vector3f& centroid){

  Eigen::Matrix4f centerMatrix = makeTranslationMatrix(-centroid(0), -centroid(1), -centroid(2));
  Eigen::Matrix4f rotationMatrix = makeRotationMatrix(roll, pitch, yaw);
  Eigen::Matrix4f uncenterMatrix = makeTranslationMatrix(centroid(0), centroid(1), centroid(2));
  Eigen::Matrix4f translationMatrix = makeTranslationMatrix(x, y, z);

  Eigen::Matrix4f transformationMatrix = translationMatrix*uncenterMatrix*rotationMatrix*centerMatrix;

  return transformationMatrix;
}

} // namespace

PrecisionTracker::PrecisionTracker()
{
  alignment_evaluator_.reset(new DensityGridTracker);
}

PrecisionTracker::~PrecisionTracker() {
  // TODO Auto-generated destructor stub
}

//for each x, y, z:
//best, min, max, stepsize, numsteps?
void PrecisionTracker::track(
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > current_points,
    const double initial_xy_sampling_resolution,
    const double initial_z_sampling_resolution,
    pair <double, double> xRange,
    pair <double, double> yRange,
    pair <double, double> zRange,
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > prev_points,
    const Eigen::Vector3f &current_points_centroid,
    const MotionModel& motion_model,
    const double down_sample_factor_prev,
    const double horizontal_distance,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms) {
  //down-sample to 150 pts
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > downSampledPoints1(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (stochastic_downsample){
    DownSampler::downSamplePointsStochastic(current_points, kCurrFrameDownsample, downSampledPoints1);
  } else {
    DownSampler::downSamplePointsDeterministic(current_points, kCurrFrameDownsample, downSampledPoints1);
  }

  // Compute the sensor horizontal resolution
  const double velodyne_horizontal_res_actual = 2 * horizontal_distance * tan(.18 / 2.0 * pi / 180.0);

  // The effective resolution = resolution / downsample factor.
  const double velodyne_horizontal_res = velodyne_horizontal_res_actual / down_sample_factor_prev;

  // The vertical resolution for the Velodyne is 2.2 * the horizontal resolution.
  const double velodyne_vertical_res = 2.2 * velodyne_horizontal_res;

  //boost::shared_ptr<AlignmentEvaluator> alignment_evaluator(new LF_RGBD_6D);
  //boost::shared_ptr<AlignmentEvaluator> alignment_evaluator(new DensityGridTracker);

  adh_tracker3d_.track(
        initial_xy_sampling_resolution, initial_z_sampling_resolution,
        xRange, yRange, zRange,
        downSampledPoints1, prev_points, current_points_centroid,
        motion_model,
        velodyne_horizontal_res, velodyne_vertical_res,
        alignment_evaluator_,
        scored_transforms);
}


void PrecisionTracker::track(
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > current_points,
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previousModel,
    const double horizontal_distance,
    const MotionModel& motion_model,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms) {
  // Down-sample the previous points.
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previous_model_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (stochastic_downsample){
    DownSampler::downSamplePointsStochastic(previousModel, kPrevFrameDownsample, previous_model_downsampled);
  } else {
    DownSampler::downSamplePointsDeterministic(previousModel, kPrevFrameDownsample, previous_model_downsampled);
  }

  // Compute the ratio by which we down-sampled, which decreases the effective
  // resolution.
  const double down_sample_factor_prev =
      static_cast<double>(previous_model_downsampled->size()) /
      static_cast<double>(previousModel->size());

  Eigen::Matrix4f centroidDiffTransform = estimateAlignmentCentroidDiff(current_points, previousModel);

  double maxX = maxXY;
  double maxY = maxXY;

  const double x_init = centroidDiffTransform(0,3);
  const double y_init = centroidDiffTransform(1,3);

  // TODO - set this to zero.
  // Assume that the vertical motion is minimal.
  const double z_init = 0; // centroidDiffTransform(2,3);

  pair <double, double> xRange (-maxX + x_init, maxX + x_init);
  pair <double, double> yRange (-maxY + y_init, maxY + y_init);
  pair <double, double> zRange (-maxZ + z_init, maxZ + z_init);

  Eigen::Vector3f current_points_centroid;
  computeCentroid(current_points, &current_points_centroid);

  track(current_points, kInitialXYSamplingResolution, kInitialZSamplingResolution, xRange, yRange, zRange,
                   previous_model_downsampled,
      current_points_centroid, motion_model, down_sample_factor_prev,
      horizontal_distance, scored_transforms);

}

Eigen::Matrix4f PrecisionTracker::estimateAlignmentCentroidDiff(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& interpolatedColoredPointsPtr,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& previousModelPtr) {
  Eigen::Vector3f newCentroid;
  computeCentroid(interpolatedColoredPointsPtr, &newCentroid);

  Eigen::Vector3f oldCentroid;
  computeCentroid(previousModelPtr, &oldCentroid);

  Eigen::Vector3f centroidDiff =  oldCentroid - newCentroid;

  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();

  guess(0,3) = centroidDiff(0);
  guess(1,3) = centroidDiff(1);
  //do not account for any elevation changes - these will be small!
  guess(2,3) = 0; // centroidDiff(2);

  return guess;
}

void PrecisionTracker::computeCentroid(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& points,
    Eigen::Vector3f* centroid) {
  *centroid = Eigen::Vector3f::Zero();

  for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = points->begin(); it < points->end(); it++) {
    pcl::PointXYZRGB p = *it;
    (*centroid)(0) += p.x;
    (*centroid)(1) += p.y;
    (*centroid)(2) += p.z;
  }

  *centroid /= static_cast<double>(points->size());
}
