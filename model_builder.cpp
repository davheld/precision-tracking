/*
 * aligner.cpp
 *
 *  Created on: Nov 20, 2011
 *      Author: davheld
 */

#include "model_builder.h"

#include <algorithm>
#include <set>
#include <vector>
#include <iostream>
#include <utility>
#include <cmath>
#include <omp.h>

#include <Eigen/Dense>

#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
//#include <pcl/common/transformation_from_correspondences.h> // Can't put this in scene_alignment_node.h

//necessary to fix openCV / PCL naming conflict - see http://www.pcl-users.org/Working-with-Code-Blocks-td3220580.html
#ifdef True
#undef True
#endif

#ifdef False
#undef False
#endif

#include "density_grid_tracker.h"
#include "scored_transform.h"
#include "ap_tracker3d.h"
#include "transform_helper.h"

using std::max;
using std::min;
using std::set;
using std::vector;
using std::cout;
using namespace Eigen;
using std::endl;
using std::pair;
using std::make_pair;

namespace{

const bool align_to_accumulated = false;
    //getenv("ALIGN_TO_ACCUMULATED") ? true : false;

const double valueWeight =
		getenv("VALUE_WEIGHT") ? atof(getenv("VALUE_WEIGHT")) : 10;

const bool accumulate_all = getenv("ACCUMULATE_ALL");

const bool use_motion_model = true; //getenv("USE_MOTION_MODEL");

const bool useCentroid = getenv("USE_CENTROID");


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

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > makeColor2(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > coloredCloud, vector<int> numNeighbors){
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > newColorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  int pointNum = 0;

  for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = coloredCloud->begin(); it < coloredCloud->end(); it++){
  	pointNum++;

    pcl::PointXYZRGB p = *it;

    int g = 20 * numNeighbors.at(pointNum);

    pcl::PointXYZRGB p2;
    p2.x = p.x;
    p2.y = p.y;
    p2.z = p.z;
    p2.r = 255 - g;
    p2.g = 255;
    p2.b = 255 - g;
    newColorCloud->points.push_back(p2);
  }

  return newColorCloud;
}

/*boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > makeColor(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > coloredCloud, float r, float g, float b){
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > newColorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = coloredCloud->begin(); it < coloredCloud->end(); it++){
    pcl::PointXYZRGB p = *it;

    pcl::PointXYZRGB p2;
    p2.x = p.x;
    p2.y = p.y;
    p2.z = p.z;
    p2.r = r;
    p2.g = g;
    p2.b = b;
    newColorCloud->points.push_back(p2);
  }

  return newColorCloud;
}*/

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > makeNonColor2(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > coloredCloud){
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > nonColorCloud(new pcl::PointCloud<pcl::PointXYZ>);

  for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = coloredCloud->begin(); it < coloredCloud->end(); it++){
    pcl::PointXYZRGB p = *it;
    pcl::PointXYZ p2;
    p2.x = p.x;
    p2.y = p.y;
    p2.z = p.z;
    nonColorCloud->points.push_back(p2);
  }

  return nonColorCloud;
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > downSamplePointsVoxel(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > points, const double& leafSize){
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > downSampledPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
//  /double leafSize = 0.1; //0.05;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (points);
  sor.setLeafSize (leafSize, leafSize, leafSize);
  sor.filter (*downSampledPoints);

  printf("Down-sampled using voxel grid from %zu points to %zu points", points->size(), downSampledPoints->size());

  return downSampledPoints;
}

}  // namespace

ModelBuilder::ModelBuilder(
    const bool visualize)
    : accumulatedInterpolatedPoints(new pcl::PointCloud<pcl::PointXYZRGB>),
      accumulatedInterpolatedPointsFull(new pcl::PointCloud<pcl::PointXYZRGB>),
      accumulatedRawColoredPoints(new pcl::PointCloud<pcl::PointXYZRGB>),
      previousModel(new pcl::PointCloud<pcl::PointXYZRGB>),
      interpolatedPts(new pcl::PointCloud<pcl::PointXYZRGB>),
      coloredPts(new pcl::PointCloud<pcl::PointXYZRGB>),
      curr_vlf_timestamp(-1),
      prev_vlf_timestamp(-1),
      visualize_(visualize)//,
      //fast_functions_(FastFunctions::getInstance()),
{
  motion_model_.reset(new MotionModel);
}

ModelBuilder::~ModelBuilder() {
  // TODO Auto-generated destructor stub
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > ModelBuilder::shiftPCL(
		const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > cloud,
    const double& x, const double& y, const double& z,
    const double& roll, const double& pitch, const double& yaw,
    const Eigen::Vector3f& centroid) {
	// Make a new cloud to store the shifted cloud.
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > shiftedCloud(
  		new pcl::PointCloud<pcl::PointXYZRGB>);

  // Shift the cloud.
  Eigen::Matrix4f transformation_matrix = makeTransformationMatrix(
  		x, y, z, roll, pitch, yaw, centroid);
  pcl::transformPointCloud(*cloud, *shiftedCloud, transformation_matrix);

  return shiftedCloud;
}

void ModelBuilder::clear(){
  accumulatedInterpolatedPoints->clear();
  accumulatedRawColoredPoints->clear();
  motion_model_.reset(new MotionModel);
}

std::vector<int> ModelBuilder::getPointsPerFrameInterpolated() const{
  return pointsPerFrame_interpolated;
}

std::vector<int> ModelBuilder::getPointsPerFrameRaw() const{
  return pointsPerFrame_raw;
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > ModelBuilder::removeCentroid(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, Eigen::Vector3f centroid){
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > newcloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

  //pcl::PointCloud<pcl::PointXYZRGB> newcloud;

  for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = cloud.begin(); it < cloud.end(); it++){
    pcl::PointXYZRGB p = *it;
    //pcl::PointXYZRGB newPoint(p.x + x, p.y + y, p.z + z);
    pcl::PointXYZRGB newPoint;
    newPoint.x = p.x - centroid.x();
    newPoint.y = p.y - centroid.y();
    newPoint.z = p.z - centroid.z();

    newPoint.rgb = p.rgb;
    newcloudPtr->push_back(newPoint);
  }

  return newcloudPtr;
}

/*Eigen::Matrix4f ModelBuilder::estimateAlignment(
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > interpolatedColoredPointsPtr,
    const int& numTries) {
  if (transformations_.size() > 0){
    Eigen::Matrix4f prev_transformation = transformations_.at(transformations_.size() - 1 - numTries);

    std::cout << "Previous transformation: \n" << prev_transformation << std::endl;*/

    //guess using just the previous translation - assume roll, pitch, yaw are 0
    /*Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    guess(0,3) = prev_transformation(0,3);
    guess(1,3) = prev_transformation(1,3);
    guess(2,3) = prev_transformation(2,3);*/

    //std::cout << "Guess: \n" << guess << std::endl;

    /*return prev_transformation;
  } else{
    return estimateAlignmentCentroidDiff(interpolatedColoredPointsPtr,
        accumulatedInterpolatedPoints);
  }
}*/

const bool kUseMode = getenv("USE_MODE");
const bool kKalmanICP = getenv("KALMAN_ICP_ALIGN");
const bool kUseICP = getenv("ICP_ALIGN");

void ModelBuilder::addPoints(
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& current_points,
		const double& vlf_timestamp,
    const Eigen::Vector3f velo_centroid,
    Eigen::Vector3f* estimated_velocity) {
  // Do not align if there are no points.
  if (current_points->size() == 0){
    printf("No points - cannot align.\n");
    *estimated_velocity = Eigen::Vector3f::Zero();
    return;
  }

  //points are in Velodyne coordinates (centered at the Velodyne) - shift to be centered at the Applanix
  //Eigen::Matrix4f veloToApplanix = dgcToEigen(velodyne_offset);
  //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > interpolatedColoredPointsPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
  //*interpolatedColoredPointsPtr = *interpolatedColoredPointsVeloPtr;
  //pcl::transformPointCloud(*interpolatedColoredPointsVeloPtr, *interpolatedColoredPointsPtr, veloToApplanix);

  //save last-time interpolated points to the previous model for alignment
  //*interpolatedPts = *interpolatedColoredPointsVeloPtr;

  //update prev occlusion checker
  //prev_occlusion_checker = curr_occlusion_checker;
  //curr_occlusion_checker = occlusion_checker;

  //save previous vlf_timestamp
  prev_vlf_timestamp = curr_vlf_timestamp;
  curr_vlf_timestamp = vlf_timestamp;

  if (visualize_) {
    *previousModel = *interpolatedPts;
    *interpolatedPts = *current_points;
  }

  //save previous alignment
  //prev_full_alignment_to_prev_ = curr_full_alignment_to_prev_;

  //save current and previous interpolated points
  //printf("Saving interpolated points\n");
  /*if (interpolatedPts != NULL){

    *prevInterpolatedPts = *interpolatedPts;
    //prevInterpolatedPts->swap(*interpolatedPts);
    //prevInterpolatedPts->swap(*currInterpolatedPts);
    //printf("Prev interpolated points: %d\n", static_cast<int>(prevInterpolatedPts->size()));
  }*/
  //*interpolatedPts = *interpolatedColoredPointsPtr;


  //PRINT_WARN("Align points: %d", alignPoints);

  //if (accumulatedInterpolatedPoints->empty()){

    //*accumulatedInterpolatedPoints = *interpolatedPts;
    //*accumulatedInterpolatedPointsFull = *interpolatedPts;
    //*previousModel = *interpolatedPts;
    //accumulatedInterpolatedPoints->swap(*interpolatedPts);
    //TODO - add raw points?

    //just add the frame to ransac
    /*if (useRANSACalignment){
    	ransacAligner_.addFrame(frame_helper, interpolater);
    }*/

    //curr_full_alignment_to_prev_ = Eigen::Affine3f::Identity();
  //} else {

  const bool alignPoints = true;

  if (!previousModel->empty()) {

    if (alignPoints) {

    	double vlf_time_stamp_diff = curr_vlf_timestamp - prev_vlf_timestamp;
    	vlf_time_stamp_diff_ = vlf_time_stamp_diff;

    	vlf_time_stamp_diffs_.push_back(vlf_time_stamp_diff);
      estimated_vlf_time_stamps_.push_back(curr_vlf_timestamp);
      if (use_motion_model) {
        // Propogate the motion model forward to estimate the new position.
        motion_model_->propagate(vlf_time_stamp_diff);
      }
      //const Eigen::Matrix3d& covariance_inv = motion_model_.get_covariance_delta_position_inv();
      //PRINT_INFO_STREAM("covariance_delta_position_inv_1: " << endl << covariance_inv);

      //PRINT_WARN("Velo timestamp diff: %lf", vlf_time_stamp_diff);

			//PRINT_WARN_STREAM("prev_full_alignment_to_prev: " << prev_full_alignment_to_prev_.matrix());

			//use optical flow to compute initial alignment to previous frame
			Eigen::Affine3f full_alignment_to_prev;

			//full_alignment_to_prev =

			// Choose the previous model for alignment.
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previous_model_for_alignment(new pcl::PointCloud<pcl::PointXYZRGB>);
      if (align_to_accumulated){
        previous_model_for_alignment = accumulatedInterpolatedPoints;
      } else {
        previous_model_for_alignment = previousModel;
      }

      // All alignments from which to form a probability distribution.
      ScoredTransforms<ScoredTransformXYZ> scored_transforms;

      // Get the distance to the tracked object.
      //Eigen::Vector4d velo_centroid = frame_helper.getVeloCentroid();
      const double horizontal_distance =
          sqrt(pow(velo_centroid(0), 2) + pow(velo_centroid(1), 2));

			if (useCentroid) {

			  Eigen::Vector3f newCentroid;
			  PrecisionTracker::computeCentroid(current_points, &newCentroid);

			  Eigen::Vector3f oldCentroid;
			  PrecisionTracker::computeCentroid(previous_model_for_alignment, &oldCentroid);

			  const Eigen::Vector3f& centroidDiff =  oldCentroid - newCentroid;

				if (use_motion_model){
					//implement kalman filter here

				  //Eigen::Matrix4f centroidDiffTransform = estimateAlignmentCentroidDiff(interpolatedPts, previousModel);
          motion_model_->addCentroidDiff(centroidDiff, vlf_time_stamp_diff);

          const Eigen::Vector3f& mean_displacement = motion_model_->mean_displacement();
					//PRINT_WARN_STREAM("mean_displacement: " << endl << mean_displacement);

					full_alignment_to_prev = Eigen::Translation<float, 3>(mean_displacement);

				} else {
					full_alignment_to_prev = Eigen::Translation<float, 3>(centroidDiff);
				}
      }
      else {
			  // Track using my method.

        // Align.
        precision_tracker_.track(current_points, previous_model_for_alignment,
            horizontal_distance, *motion_model_, &full_alignment_to_prev, &scored_transforms);

        motion_model_->addTransformsWeightedGaussian(scored_transforms,
                                                    vlf_time_stamp_diff);
			}
			Eigen::Vector3f centroid;
			PrecisionTracker::computeCentroid(current_points, &centroid);
			Eigen::Vector4d centroid_4d;
			centroid_4d(0) = centroid(0);
      centroid_4d(1) = centroid(1);
      centroid_4d(2) = centroid(2);
      centroid_4d(3) = 1;
			Eigen::Affine3f real_transform_to_prev =
			    TransformHelper::computeRealTransform(
			        full_alignment_to_prev,
			        centroid_4d);


				bool debug_print = false;

				if (debug_print) {
				  printf("Debug print - will be slow");
					TransformComponents components_curr_to_prev= TransformHelper::computeComponents(real_transform_to_prev);
					double totalMag_curr_to_prev = sqrt(pow(components_curr_to_prev.x, 2) + pow(components_curr_to_prev.y, 2));
					printf("Total disp (curr to prev): x: %lf, y: %lf, z: %lf, roll: %lf, pitch: %lf, yaw: %lf", components_curr_to_prev.x, components_curr_to_prev.y, components_curr_to_prev.z, components_curr_to_prev.roll, components_curr_to_prev.pitch, components_curr_to_prev.yaw);
					printf("Total disp (curr to prev): x: %lf, y: %lf, mag: %lf", components_curr_to_prev.x, components_curr_to_prev.y, totalMag_curr_to_prev);

					TransformComponents components_prev_to_curr = TransformHelper::computeComponents(real_transform_to_prev.inverse());
					double totalMag_prev_to_curr = sqrt(pow(components_prev_to_curr.x, 2) + pow(components_prev_to_curr.y, 2));
					printf("Total disp (prev to curr): x: %lf, y: %lf, mag: %lf", components_prev_to_curr.x, components_prev_to_curr.y, totalMag_prev_to_curr);
				}

				transformations_.push_back(real_transform_to_prev.matrix());
				inverse_transformations_.push_back(real_transform_to_prev.matrix().inverse());


        Eigen::Vector3f mean_displacement = motion_model_->mean_displacement();
				Eigen::Affine3f mean_translation; mean_translation = Eigen::Translation3f(mean_displacement);
				transformations_mean_.push_back(mean_translation.matrix());

        Eigen::Vector3f mean_velocity = motion_model_->get_mean_velocity();

				Eigen::Affine3f estimated_full_alignment_to_prev;

				if (kUseMode) {
          //*estimated_velocity = real_transform_to_prev;
				  estimated_full_alignment_to_prev = full_alignment_to_prev;
				} else {
          *estimated_velocity = mean_velocity;
          estimated_full_alignment_to_prev = mean_translation;
				}

				if (accumulate_all){
				  printf("Align to accumulate and accumulate all - will be slow");
					pcl::transformPointCloud(
					    *accumulatedInterpolatedPoints,
					    *accumulatedInterpolatedPoints,
					    estimated_full_alignment_to_prev.inverse());

					*accumulatedInterpolatedPoints += *current_points;

          //keep track of points per frame
          pointsPerFrame_interpolated.push_back(current_points->size());
          pointsPerFrame_raw.push_back(current_points->size());

				} else if (visualize_) {
				  //keep track of points per frame
				  pointsPerFrame_interpolated.push_back(interpolatedPts->size());
				  pointsPerFrame_raw.push_back(current_points->size());

				  printf("Applying transform: %lf, %lf, %lf\n",
				      (*estimated_velocity)(0,3),
				      (*estimated_velocity)(1,3),
				      (*estimated_velocity)(2,3));

				  printf("Visualizing - will be slow");
					typedef pcl::PointXYZRGB PointRGB;
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pts_aligned_to_prev (
					    new pcl::PointCloud<pcl::PointXYZRGB>);
					pcl::transformPointCloud(*interpolatedPts,
					    *current_pts_aligned_to_prev, estimated_full_alignment_to_prev);

					for (int i = 0; i < previousModel->size(); ++i){
					  (*previousModel)[i].r = 255;
            (*previousModel)[i].g = 0;
            (*previousModel)[i].b = 0;
					}
					*accumulatedInterpolatedPoints = *previousModel;

          for (int i = 0; i < current_pts_aligned_to_prev->size(); ++i){
            (*current_pts_aligned_to_prev)[i].r = 0;
            (*current_pts_aligned_to_prev)[i].g = 0;
            (*current_pts_aligned_to_prev)[i].b = 255;
          }
					*accumulatedInterpolatedPoints += *current_pts_aligned_to_prev;
				}

			//pcl::transformPointCloud(*centeredColoredPointCloudPtr, *centeredColoredPointCloudPtr, full_alignment_to_prev);
			//*accumulatedInterpolatedPoints += *current_points_initial_alignment;
			//*accumulatedRawColoredPoints += *centeredColoredPointCloudPtr;

    		//TODO - compute alignment to model using stored previous transforms
      /*} else {
    		*current_points_initial_alignment = *interpolatedPts;
    	}*/

    } else { //end doICP

    	printf("Accumulating points (no alignment)\n");
      //accumulating with no alignment`
      // *accumulatedInterpolatedPoints += *interpolatedColoredPointsPtr;
      // *accumulatedRawColoredPoints += *centeredColoredPointCloudPtr;

      //no alignment
    	printf("*************Warning: Just accumulating points with no alignment***********\n");


    	/*printf("*************Warning: Just estimating alignment using centroid diff***********\n");

      Eigen::Matrix4f centroidDiffAlignment =  (current_points_initial_alignment);
      std::cout << "Centroid diff alignment:\n" << centroidDiffAlignment << std::endl;

      Eigen::Matrix4f centroidDiffAlignment_inv = centroidDiffAlignment.inverse();

      std::cout << "Inverse transformation:\n" << centroidDiffAlignment_inv << std::endl;

      transformations_.push_back(centroidDiffAlignment);
      inverse_transformations_.push_back(centroidDiffAlignment_inv);
      estimated_vlf_time_stamps_.push_back(vlfTimestamp);
      transformations_full_.push_back(centroidDiffAlignment);
      inverse_transformations_full_.push_back(centroidDiffAlignment_inv);

      //inverse transform the model to align with the current frame
      pcl::transformPointCloud(*accumulatedInterpolatedPoints, *accumulatedInterpolatedPoints, centroidDiffAlignment_inv);
			*/

      //*accumulatedInterpolatedPoints += *interpolatedPts;
      //*accumulatedRawColoredPoints += *centeredColoredPointCloudPtr;

    }
  } else {
    // No previous points - just creating initial model.
    *accumulatedInterpolatedPoints = *current_points;
    *estimated_velocity = Eigen::Vector3f::Zero();
  }

	if (!visualize_) {
	  *previousModel = *current_points;
	}
}

std::vector<Eigen::Matrix4f> ModelBuilder::getTransformations() const {
  return transformations_;
}

std::vector<Eigen::Matrix4f> ModelBuilder::getInverseTransformationsFull() const {
  return inverse_transformations_full_;
}

std::vector<Eigen::Matrix4f> ModelBuilder::getTransformationsFull() const {
  return transformations_full_;
}

std::vector<Eigen::Matrix4f> ModelBuilder::getInverseTransformations() const {
  return inverse_transformations_;
}

std::vector<double> ModelBuilder::getEstimatedVLFTimestamps() const{
  return estimated_vlf_time_stamps_;
}

std::vector<double> ModelBuilder::getVLFTimestampDiffs() const{
	return vlf_time_stamp_diffs_;
}
