/*
 * precision_tracker.cpp
 *
 *  Created on: May 12, 2014
 *      Author: davheld
 */

#include "precision_tracker.h"

#include <boost/math/constants/constants.hpp>
#include "down_sampler.h"
#include "high_res_timer.h"

namespace {

const int kCurrFrameDownsample =
    getenv("PROJECTION_SEARCH_NUM_POINTS") ? atoi(getenv("PROJECTION_SEARCH_NUM_POINTS")) : 150;

const double minXYStep = getenv("MIN_XY_STEP") ? atof(getenv("MIN_XY_STEP")) : 0.05;

const bool stochastic_downsample = false;

const int kPrevFrameDownsample = getenv("FINE_SEARCH_NUM_POINTS") ? atoi(getenv("FINE_SEARCH_NUM_POINTS")) : 2000;

// Set to 0 so we do not search over z (for objects moving in urban settings,
// the vertical motion is small).
const double maxZ = getenv("MAX_Z") ? atof(getenv("MAX_Z")) : 1;

const double maxXY = getenv("MAX_XY") ? atof(getenv("MAX_XY")) : 2;

const double stepSize =
    getenv("STEP_SIZE") ? atof(getenv("STEP_SIZE")) : 1;

const double kZStepSize =
    getenv("Z_STEP_SIZE") ? atof(getenv("Z_STEP_SIZE")) : 0.5;

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
  // TODO Auto-generated constructor stub
}

PrecisionTracker::~PrecisionTracker() {
  // TODO Auto-generated destructor stub
}

//for each x, y, z:
//best, min, max, stepsize, numsteps?
void PrecisionTracker::findBestLocation(
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > current_points,
    const double max_xy_stepSize,
    const double max_z_stepSize,
    pair <double, double> xRange,
    pair <double, double> yRange,
    pair <double, double> zRange,
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > prev_points,
    const Eigen::Vector3f &current_points_centroid,
    const MotionModel& motion_model,
    const double down_sample_factor_prev,
    const double point_ratio,
    const double horizontal_distance,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms) {
  //down-sample to 150 pts
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > downSampledPoints1;
  if (stochastic_downsample){
    downSampledPoints1 = DownSampler::downSamplePointsStochastic(current_points, kCurrFrameDownsample);
  } else {
    downSampledPoints1 = DownSampler::downSamplePointsDeterministic(current_points, kCurrFrameDownsample);
  }

  const int total_num_points = 0;

  ap_tracker3d_.track(max_xy_stepSize, max_z_stepSize, xRange, yRange, zRange,
      downSampledPoints1, prev_points, current_points_centroid,
      motion_model, total_num_points, horizontal_distance,
      down_sample_factor_prev, point_ratio, scored_transforms);
}


void PrecisionTracker::track(
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > current_points,
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previousModel,
    const double horizontal_distance,
    const MotionModel& motion_model,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms) {
  // Ratio of the previous model size to the current model size.
  // If less than 1, then this indicates the chance of a match to the current
  // model.
  // Example: If the previous model had 50 points and the current model has
  // 200, the chance of a match is 50 / 200.  The chance of no match is 150 / 200.
  const double point_ratio =
      static_cast<double>(previousModel->size()) / current_points->size();
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previous_model_downsampled;
  if (stochastic_downsample){
    previous_model_downsampled = DownSampler::downSamplePointsStochastic(previousModel, kPrevFrameDownsample);
  } else {
    previous_model_downsampled = DownSampler::downSamplePointsDeterministic(previousModel, kPrevFrameDownsample);
  }
  const double down_sample_factor_prev =
      static_cast<double>(previous_model_downsampled->size()) /
      static_cast<double>(previousModel->size());

  Eigen::Matrix4f centroidDiffTransform = estimateAlignmentCentroidDiff(current_points, previousModel);

  double maxX = maxXY;
  double maxY = maxXY;

  const double x_init = centroidDiffTransform(0,3);
  const double y_init = centroidDiffTransform(1,3);

  // TODO - set this to zero.
  const double z_init = centroidDiffTransform(2,3);

  pair <double, double> xRange (-maxX + x_init, maxX + x_init);
  pair <double, double> yRange (-maxY + y_init, maxY + y_init);
  pair <double, double> zRange (-maxZ + z_init, maxZ + z_init);

  Eigen::Vector3f current_points_centroid;
  computeCentroid(current_points, &current_points_centroid);

  findBestLocation(current_points, stepSize, kZStepSize, xRange, yRange, zRange,
                   previous_model_downsampled,
      current_points_centroid, motion_model, down_sample_factor_prev,
      point_ratio, horizontal_distance, scored_transforms);

}

Eigen::Matrix4f PrecisionTracker::estimateAlignmentCentroidDiff(
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > interpolatedColoredPointsPtr,
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previousModelPtr) {
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
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > points,
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
