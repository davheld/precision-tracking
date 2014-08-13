/*
 * aligner.h
 *
 *  Created on: Nov 20, 2011
 *      Author: davheld
 */

#ifndef MOVING_SYNCHRONIZER_MODEL_BUILDER_H_
#define MOVING_SYNCHRONIZER_MODEL_BUILDER_H_

#include <vector>

#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//necessary to fix openCV / PCL naming conflict - see http://www.pcl-users.org/Working-with-Code-Blocks-td3220580.html
#ifdef True
#undef True
#endif

#ifdef False
#undef False
#endif

//#include <cv.h>
//#include <highgui.h>

//#include "transform.h"
#include "motion_model.h"
//#include "frame_helper.h"
#include "ap_tracker3d.h"
#include "fast_functions.h"
//#include "ap_tracker.h"
//#include "NNTracker3d.h"
#include "precision_tracker.h"
//#include "ransac_aligner.h"

class PointsInterpolater;

class ModelBuilder {
public:
  ModelBuilder(const bool visualize);
  virtual ~ModelBuilder();

  static boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > shiftPCL(
  		const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > cloud,
      const double& x, const double& y, const double& z,
      const double& roll, const double& pitch, const double& yaw,
      const Eigen::Vector3f& centroid);

  void clear();

  void addPoints(
      const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& current_points,
      const double& vlfTimestamp,
      const Eigen::Vector3f velo_centroid,
      //const dgc_transform_t velodyne_offset,
      //bool alignPoints,
      //const FrameHelper& frame_helper,
      Eigen::Vector3f* estimated_velocity);
  //void addImage(IplImage* currentImage);

  // Getters.
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > getAccumulatedInterpolatedPoints() const {
    return accumulatedInterpolatedPoints;
  }

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > getAccumulatedRawPoints() const {
    return accumulatedRawColoredPoints;
  }

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > getInterpolatedPoints() const {
    return interpolatedPts;
  }

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > getColoredPoints() const {
    return coloredPts;
  }

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > getFullModel() const {
    return accumulatedInterpolatedPointsFull;
  }
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > getPreviousModel() const {
    return previousModel;
  }

  std::vector<int> getPointsPerFrameInterpolated() const;
  std::vector<int> getPointsPerFrameRaw() const;

  std::vector<Eigen::Matrix4f> getInverseTransformations() const;
  std::vector<Eigen::Matrix4f> getTransformations() const;
  std::vector<Eigen::Matrix4f> getTransformationsMean() const { return transformations_mean_; }

  std::vector<Eigen::Matrix4f> getInverseTransformationsFull() const;
  std::vector<Eigen::Matrix4f> getTransformationsFull() const;

  std::vector<double> getEstimatedVLFTimestamps() const;
  std::vector<double> getVLFTimestampDiffs() const;

  void repairFrames();

  void icpAlign(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > current_points_initial_alignment, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > centeredColoredPointCloudPtr, const double& vlfTimestamp);
  void gridsearchAlign(
  		const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > current_points,
  		const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previousModel,
      const double horizontal_distance,
  		Eigen::Affine3f* full_alignment_to_prev,
  		ScoredTransforms<ScoredTransform>* scored_transforms);

  static boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > removeCentroid(
  		const pcl::PointCloud<pcl::PointXYZRGB>& cloud, Eigen::Vector3f centroid);
  static Eigen::Matrix4f estimateAlignmentCentroidDiff(
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > interpolatedColoredPointsPtr,
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previousModelPtr);

private:
  //Eigen::Matrix4f estimateAlignment(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > interpolatedColoredPointsPtr, const int& numTries);
  Eigen::Affine3f computeOpticalFlowAlignment();
  void sampleCorrespondences(std::vector<pcl::PointXYZRGB>* prev,
             std::vector<pcl::PointXYZRGB>* curr) const;
  double scoreTransform(const Eigen::Affine3f& trans,
              std::vector<pcl::PointXYZRGB>* curr_inliers,
              std::vector<pcl::PointXYZRGB>* prev_inliers) const;
  void gridsearchFine(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > current_points_orig, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previousModel_orig, Eigen::Affine3f& full_alignment_to_prev, std::vector<TransformProposal>& transformProposals);

  void findBestLocation(
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > current_points,
      const double xy_stepSize,
      const double z_stepSize,
      std::pair <double, double> xRange,
      std::pair <double, double> yRange,
      std::pair <double, double> zRange,
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
      ScoredTransforms<ScoredTransform>* scored_transforms);

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > accumulatedInterpolatedPoints;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > accumulatedInterpolatedPointsFull;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > accumulatedRawColoredPoints;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previousModel;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > previousModelRaw;

  //OcclusionChecker prev_occlusion_checker;
  //OcclusionChecker curr_occlusion_checker;

  //TODO - save intensity

  //IplImage* currImageOriginalSize;

  //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > prevInterpolatedPts;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > interpolatedPts;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > coloredPts;

  //cv::Mat1i* curr_index_;
  //cv::Mat1i* prev_index_;

  double curr_vlf_timestamp;
  double prev_vlf_timestamp;

  Eigen::Affine3f prev_full_alignment_to_prev_;
  Eigen::Affine3f curr_full_alignment_to_prev_;

  std::vector<int> pointsPerFrame_interpolated;
  std::vector<int> pointsPerFrame_raw;

  std::vector<Eigen::Matrix4f> transformations_;
  std::vector<Eigen::Matrix4f> inverse_transformations_;

  std::vector<Eigen::Matrix4f> transformations_full_;
  std::vector<Eigen::Matrix4f> inverse_transformations_full_;

  std::vector<Eigen::Matrix4f> transformations_mean_;
  //std::vector<Eigen::Matrix4f> inverse_transformations_mean_;

  std::vector<double> estimated_vlf_time_stamps_;
  std::vector<double> vlf_time_stamp_diffs_;

  double vlf_time_stamp_diff_;

  boost::shared_ptr<MotionModel> motion_model_;

  bool visualize_;

  /*APTracker ap_tracker2d_;
  APTracker3d ap_tracker3d_;
  const FastFunctions& fast_functions_;

  DensityGridTracker density_grid_tracker2d_;
  DensityGridTracker3d& density_grid_tracker3d_;
  NNTracker3d nn_tracker_;*/

  PrecisionTracker precision_tracker_;

  //RansacAligner ransacAligner_;

};

//public helper functions
template <class Point>
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > makeColor(const boost::shared_ptr<const pcl::PointCloud<Point> > coloredCloud, float r, float g, float b){
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > newColorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  for (typename pcl::PointCloud<Point>::const_iterator it = coloredCloud->begin(); it < coloredCloud->end(); it++){
    Point p = *it;

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
}

#endif /* MOVING_SYNCHRONIZER_MODEL_BUILDER_H_ */
