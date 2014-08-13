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

const int projectionSearchNumPoints =
    getenv("PROJECTION_SEARCH_NUM_POINTS") ? atoi(getenv("PROJECTION_SEARCH_NUM_POINTS")) : 150;

const double minXYStep = getenv("MIN_XY_STEP") ? atof(getenv("MIN_XY_STEP")) : 0.05;

const bool stochastic_downsample = false;

const int fineSearchNumPoints = getenv("FINE_SEARCH_NUM_POINTS") ? atoi(getenv("FINE_SEARCH_NUM_POINTS")) : 2000;

// Set to 0 so we do not search over z (for objects moving in urban settings,
// the vertical motion is small).
const double maxZ = getenv("MAX_Z") ? atof(getenv("MAX_Z")) : 1;

const bool kUseAbsoluteRef = false;

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
 //: //lf_discrete_3d_(LFDiscrete3d::getInstance())
      //density_grid_tracker3d_(DensityGridTracker3d::getInstance())
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
    double& bestX,
    double& bestY,
    double& bestZ,
    double& bestRoll,
    double& bestPitch,
    double& bestYaw,
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > prev_points,
    const Eigen::Vector3f &current_points_centroid,
    const MotionModel& motion_model,
    const double down_sample_factor_prev,
    const double point_ratio,
    const double horizontal_distance,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms) {
  //coarse search
  double maxYaw = 0;
  pair <double, double> rollRange (0,0);
  pair <double, double> pitchRange (0,0);
  pair <double, double> yawRange (-maxYaw,maxYaw);
  double angleStep = 0.05;

  //down-sample to 150 pts

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > downSampledPoints1;
  if (stochastic_downsample){
    downSampledPoints1 = DownSampler::downSamplePointsStochastic(current_points, projectionSearchNumPoints);
  } else {
    downSampledPoints1 = DownSampler::downSamplePointsDeterministic(current_points, projectionSearchNumPoints);
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
    Eigen::Affine3f* full_alignment_to_prev,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms) {

  //HighResTimer hrt("Grid search align setup");
  //hrt.start();

  //PRINT_INFO("Performing Grid search alignment");

  // Ratio of the previous model size to the current model size.
  // If less than 1, then this indicates the chance of a match to the current
  // model.
  // Example: If the previous model had 50 points and the current model has
  // 200, the chance of a match is 50 / 200.  The chance of no match is 150 / 200.
  const double point_ratio =
      static_cast<double>(previousModel->size()) / current_points->size();
  //printf("Point ratio: %lf\n", point_ratio);


  //PRINT_INFO("Model size: %d points", static_cast<int>(accumulatedInterpolatedPoints->size()));

  //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > current_points(new pcl::PointCloud<pcl::PointXYZRGB>);
  //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previousModel(new pcl::PointCloud<pcl::PointXYZRGB>);

  //*current_points = *current_points_orig;
  //*previousModel = *previousModel_orig;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previous_model_downsampled;
  if (stochastic_downsample){
    previous_model_downsampled = DownSampler::downSamplePointsStochastic(previousModel, fineSearchNumPoints);
  } else {
    previous_model_downsampled = DownSampler::downSamplePointsDeterministic(previousModel, fineSearchNumPoints);
  }
  const double down_sample_factor_prev =
      static_cast<double>(previous_model_downsampled->size()) /
      static_cast<double>(previousModel->size());

  //remove centroid so we can exactly see the translation (otherwise we see the translation + rotation)
  //Eigen::Vector3f centroid = computeCentroid(*current_points);
  //Eigen::Vector3f centroid = computeCentroid(*previousModel); - this was most recently used
  //Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > currentPointsCenteredPtr = removeCentroid(*current_points, centroid);
  //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > modelCenteredPtr = removeCentroid(*previousModel, centroid);
  //*current_points = *currentPointsCenteredPtr;
  //*previousModel = *modelCenteredPtr;

  //TODO align using prev points instead of accumulated
  //TODO take the centroid diff into account when determining rotation/translation
  Eigen::Matrix4f centroidDiffTransform = estimateAlignmentCentroidDiff(current_points, previousModel);
  //Eigen::Matrix4f centroidDiffTransform_inv = centroidDiffTransform.inverse();

  //PRINT_WARN_STREAM("Centroid diff: " << std::endl << centroidDiffTransform);
  //PRINT_WARN("Centroid velocity: %lf", sqrt(pow(centroidDiffTransform(0,3), 2) + pow(centroidDiffTransform(1,3), 2)));

  //start by aligning the centroids - thus searching only over the range for which the points actually align
  //pcl::transformPointCloud(*accumulatedInterpolatedPoints, *accumulatedInterpolatedPoints, centroidDiffTransform_inv);
  //pcl::transformPointCloud(*accumulatedInterpolatedPointsFull, *accumulatedInterpolatedPointsFull, centroidDiffTransform_inv);
  //pcl::transformPointCloud(*previousModel, *previousModel, centroidDiffTransform_inv);

  //add centroid offset to occlusion checker
  /*Eigen::Vector3f offset;
  offset(0) += centroidDiffTransform(0,3);
  offset(1) += centroidDiffTransform(1,3);
  offset(2) += centroidDiffTransform(2,3);
  occlusion_checker.addOffset(offset);*/

  //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > current_points_initial_alignment(new pcl::PointCloud<pcl::PointXYZRGB>);

  //const double xy_stepsize = stepSize;
  //const double z_stepsize = 2.2 * stepSize;

  double maxX = maxXY;
  double maxY = maxXY;

  const double x_init = centroidDiffTransform(0,3);
  const double y_init = centroidDiffTransform(1,3);

  double z_init;
  if (kUseAbsoluteRef) {
    z_init = 0;
  } else {
    z_init = centroidDiffTransform(2,3);
  }

  pair <double, double> xRange (-maxX + x_init, maxX + x_init);
  pair <double, double> yRange (-maxY + y_init, maxY + y_init);
  pair <double, double> zRange (-maxZ + z_init, maxZ + z_init);

  Eigen::Vector3f current_points_centroid;
  computeCentroid(current_points, &current_points_centroid);

  //find best shift x,y,z
  //double bestX = centroidDiffTransform(0), bestY = centroidDiffTransform(1), bestZ = centroidDiffTransform(2), bestRoll = 0, bestPitch = 0, bestYaw = 0;
  double bestX = 0, bestY = 0, bestZ = 0, bestRoll = 0, bestPitch = 0, bestYaw = 0;
  findBestLocation(current_points, stepSize, kZStepSize, xRange, yRange, zRange, bestX,
      bestY, bestZ, bestRoll, bestPitch, bestYaw, previous_model_downsampled,
      current_points_centroid, motion_model, down_sample_factor_prev,
      point_ratio, horizontal_distance, scored_transforms);

  /*ScoredTransformXYZ best_transform;
  double best_prob;
  scored_transforms->findBest(&best_transform, &best_prob);

  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
  transform_matrix(0,3) = best_transform.getX();
  transform_matrix(1,3) = best_transform.getY();
  transform_matrix(2,3) = best_transform.getZ();
  *full_alignment_to_prev = transform_matrix;*/



  //show info about best alignment
  /*PRINT_INFO("Best alignment:");
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > shiftedCurrentPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
  *shiftedCurrentPoints = *shiftPCL(current_points, bestX, bestY, bestZ, bestRoll, bestPitch, bestYaw, current_points_centroid);
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > matchedPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
  int foundNeighbors;
  int missingNeighbors;
  vector<int> numNeighbors;
  double searchSize = 0.05;
  AlignmentScoreComputer::computeScore(shiftedCurrentPoints, searchTree, searchSize, matchedPoints, foundNeighbors, missingNeighbors, numNeighbors, true, occlusion_checker);*/

  //PRINT_INFO("Best: x: %lf, y: %lf, z: %lf, roll: %lf, pitch: %lf, yaw: %lf", bestX, bestY, bestZ, bestRoll, bestPitch, bestYaw);

  //cout << endl << "centroidDiffTransform: " << centroidDiffTransform << endl;
  //cout << endl << "centroidDiffTransform_inv: " << centroidDiffTransform_inv << endl;

  /*double totalDeltaX = bestX + centroidDiffTransform(0,3);
  double totalDeltaY = bestY + centroidDiffTransform(1,3);
  double totalMag = sqrt(pow(totalDeltaX, 2) + pow(totalDeltaY, 2));
  PRINT_INFO("Centroid diff: x: %lf, y: %lf", centroidDiffTransform(0,3), centroidDiffTransform(1,3));
  PRINT_INFO("Total velocity: x: %lf, y: %lf, mag: %lf", totalDeltaX, totalDeltaY, totalMag);*/

  //apply best shift
  //TODO take the centroid diff into account when determining rotation/translation
  /*Eigen::Matrix4f transformation_matrix =
      makeTransformationMatrix(bestX, bestY, bestZ, bestRoll, bestPitch,
          bestYaw, current_points_centroid);*/
   //Eigen::Matrix4f transformation_matrix_inverse = transformation_matrix.inverse();

  //full_alignment_to_prev = transformation_matrix_inverse * centroidDiffTransform_inv;
  //full_alignment_to_prev = full_alignment_to_prev.inverse();
  //full_alignment_to_prev = centroidDiffTransform * transformation_matrix;
  //*full_alignment_to_prev = transformation_matrix;



  //Eigen::Matrix4f transformation_matrix = makeTransformationMatrix(bestX, bestY, bestZ, bestRoll, bestPitch, bestYaw, current_points_centroid);
  //Eigen::Affine3f transform; transform = centroidDiffTransform * transformation_matrix;

  /*Eigen::Vector4d centroid(current_points_centroid(0), current_points_centroid(1), current_points_centroid(2), 1);
  TransformComponents components = TransformHelper::computeComponents(*full_alignment_to_prev, centroid);
  double motionModelProbability = motion_model_.computeScore(components);*/
  /*printf("Components: %lf, %lf, %lf\n", components.x, components.y, components.z);
  const Eigen::Vector3d& mean_delta_position =
      motion_model_.get_mean_delta_position();
  printf("Mean delta position: %lf, %lf, %lf\n", mean_delta_position(0),
      mean_delta_position(1), mean_delta_position(2));*/

  //PRINT_INFO("motionModelProbability: %lf", motionModelProbability);

  /*double motionModelProbability2 = motion_model_.computeScore(
      mean_delta_position(0), components.y, components.z);
  //double motionModelProbability2 = motion_model_.computeScore(mean_delta_position(0),
  //    mean_delta_position(1), mean_delta_position(2));
  printf("Motion model prob2: %lf\n", motionModelProbability2);

  printf("Min score: %lf\n", motion_model_.get_min_score());*/



  //Eigen::Vector4d centroid(current_points_centroid(0), current_points_centroid(1), current_points_centroid(2), 0);
  //Eigen::Affine3f real_transform_to_prev = TransformHelper::computeRealTransform(full_alignment_to_prev, centroid);

  /*TransformComponents components = TransformHelper::computeComponents(real_transform_to_prev.inverse());
  double totalMag2 = sqrt(pow(components.x, 2) + pow(components.y, 2));
  PRINT_INFO("Total velocity: x: %lf, y: %lf, mag: %lf", components.x, components.y, totalMag2);*/


  //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > bestShiftedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::transformPointCloud(*accumulatedInterpolatedPoints, *bestShiftedCloud, transformation_matrix_inverse);
  //*accumulatedInterpolatedPoints = *bestShiftedCloud;

  //pcl::transformPointCloud(*accumulatedInterpolatedPoints, *accumulatedInterpolatedPoints, transformation_matrix_inverse);

  //pcl::transformPointCloud(*previousModel, *previousModel, transformation_matrix_inverse);

  //undo effect of removing centroid:
  /*Eigen::Matrix4f centroidTransform = Eigen::Matrix4f::Identity();
  centroidTransform(0,3) = centroid(0);
  centroidTransform(1,3) = centroid(1);
  centroidTransform(2,3) = centroid(2);
  //cout << "Centroid transform:\n" << centroidTransform << endl;
  //Eigen::Matrix4f centroidTransform_inv = centroidTransform.inverse();
  //cout << "Centroid transform inverse:\n" << centroidTransform_inv << endl;
  pcl::transformPointCloud(*previousModel, *previousModel, centroidTransform);
  pcl::transformPointCloud(*current_points, *current_points, centroidTransform);*/

  /*if (getenv("SHOW_MATCH_DEBUG")){
    pcl::visualization::PCLVisualizer debugViewer;
    debugViewer.removePointCloud("previousPointsColor");
    debugViewer.removePointCloud("currentPointsColor");
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previousPointsColor = makeColor<pcl::PointXYZRGB>(previousModel, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb3(previousPointsColor);
    debugViewer.addPointCloud (previousPointsColor, rgb3, "previousPointsColor");

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > current_pointsColor = makeNonColor2(current_points);
    debugViewer.addPointCloud (current_pointsColor, "currentPointsColor");
    debugViewer.spin();
  }*/


  //*previousModel = *current_points;

  //accumulate points filtered points
  //*accumulatedInterpolatedPoints = *current_points;
  //*accumulatedInterpolatedPoints += *current_points;

  //*accumulatedInterpolatedPoints = *previousModel;
  //*accumulatedInterpolatedPoints += *current_points;

  //accumulate full points
  //pcl::transformPointCloud(*accumulatedInterpolatedPointsFull, *accumulatedInterpolatedPointsFull, transformation_matrix_inverse);
  //*accumulatedInterpolatedPointsFull = *current_points;
  //*accumulatedInterpolatedPointsFull += *current_points;

  //create the filtered object
  //*accumulatedInterpolatedPoints = *downSamplePoints(points);

  /*Eigen::Matrix4f bestTransformPrevToCurrent = transformation_matrix_inverse * centroidDiffTransform_inv;

  transformations_.push_back(bestTransformPrevToCurrent.inverse());//transformation_matrix);
  inverse_transformations_.push_back(bestTransformPrevToCurrent);//transformation_matrix_inverse);

  transformations_full_.push_back(bestTransformPrevToCurrent.inverse()); //transformation_matrix);
  inverse_transformations_full_.push_back(bestTransformPrevToCurrent); //transformation_matrix_inverse);*/


  //PRINT_INFO("Done with grid search align");

  //hrt.stop();
  //hrt.printMilliseconds();
}

Eigen::Matrix4f PrecisionTracker::estimateAlignmentCentroidDiff(
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > interpolatedColoredPointsPtr,
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previousModelPtr) {
  Eigen::Vector3f newCentroid;
  computeCentroid(interpolatedColoredPointsPtr, &newCentroid);

  Eigen::Vector3f oldCentroid;
  computeCentroid(previousModelPtr, &oldCentroid);

  Eigen::Vector3f centroidDiff =  oldCentroid - newCentroid;

  //do not account for any elevation changes - these will be small!
  //centroidDiff(2) = 0;

  //Eigen::Vector3f centroidDiff =  newCentroid - oldCentroid;

  //std::cout << "New centroid: \n" << newCentroid << std::endl;
  //std::cout << "Old centroid: \n" << oldCentroid << std::endl;
  //std::cout << "centroidDiff: \n" << centroidDiff << std::endl;


  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();

  guess(0,3) = centroidDiff(0);
  guess(1,3) = centroidDiff(1);
  guess(2,3) = 0; // centroidDiff(2);

  //std::cout << "Guess: \n" << guess << std::endl;

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
