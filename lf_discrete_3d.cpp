/*
 * lf_discrete_3d.cpp
 *
 * Created on: May 1, 2014
 *      Author: davheld
 *
 * Discretized version of the likelihood field tracker from
 * Probabilistic Robotics, Thrun, et al, 2005.
 *
 * Parameters are taken based on the latent surface model from RSS 2014,
 * Held, et al.
 *
 * The 3d volume is discretized and the probability values are cached
 * after they are computed.
 */

#include "lf_discrete_3d.h"

#include <utility>
#include <vector>

#include <algorithm>

#include <boost/math/constants/constants.hpp>

#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

using std::max;
using std::min;
using std::pair;
using std::vector;

namespace {

// If true, we add a term to the measurement covariance based on the spacing between
// particles.
const bool kUse_annealing = true;

// Factor to multiply the sensor resolution for our measurement model.
// We model each point as a Gaussian: exp(-x^2 / 2 sigma^2)
// With sigma^2 = (sensor_resolution * kSigmaFactor)^2 + other terms.
const double kSigmaFactor = getenv("SIGMA_FACTOR") ? atof(getenv("SIGMA_FACTOR")) : 0.5;

// Factor to multiply the particle sampling resolution for our measurement  model.
// We model each point as a Gaussian: exp(-x^2 / 2 sigma^2)
// With sigma^2 = (sampling_resolution * kSigmaGridFactor)^2 + other terms.
const double kSigmaGridFactor = getenv("SIGMA_GRID_FACTOR") ? atof(getenv("SIGMA_GRID_FACTOR")) : 1;

// The noise in our sensor which is independent of the distance to the tracked object.
// We model each point as a Gaussian: exp(-x^2 / 2 sigma^2)
// With sigma^2 = kMinMeasurementVariance^2 + other terms.
const double kMinMeasurementVariance = getenv("MIN_MEASUREMENT_VAR") ? atof(getenv("MIN_MEASUREMENT_VAR")) : 0.03;

// We add this to our Gaussian so we don't give 0 probability to points
// that don't align.
// We model each point as a Gaussian: exp(-x^2 / 2 sigma^2) + kSmoothingFactor
const double kSmoothingFactor = getenv("SMOOTHING_FACTOR") ? atof(getenv("SMOOTHING_FACTOR")) : 0.8;

// We multiply our log measurement probability by this factor, to decrease
// our confidence in the measurement model (e.g. to take into account
// dependencies between neighboring points).
const double kMeasurementDiscountFactor = getenv("MEASUREMENT_DISCOUNT_3D") ? atof(getenv("MEASUREMENT_DISCOUNT_3D")) : 1;

// Approximation factor for finding the nearest neighbor.
// Set to 0 to find the exact nearest neighbor.
// For a reasonable speedup, set to 2.
const double kSearchTreeEpsilon = getenv("SEARCH_TREE_EPSILON") ? atof(getenv("SEARCH_TREE_EPSILON")) : 0;

const double pi = boost::math::constants::pi<double>();

// ------------------------

const bool use_lf_tracker = false;

const bool kUseMotion = true;

// -----Color Parameters----

// Whether to use color in our measurement model.
const bool kUseColor = true;

// Whether to use two colors in our measurement model.
const bool kTwoColors = false;

// The parameter to use for our color Laplacian for color 1.
const double kValueSigma1 = getenv("VALUE_SIGMA") ? atof(getenv("VALUE_SIGMA")) : 13.9;

// The parameter to use for our color Laplacian for color 2.
const double kValueSigma2 = getenv("VALUE_SIGMA2") ? atof(getenv("VALUE_SIGMA2")) : 15.2;

// How much we expect the colors to match (there might have been lens flare,
// the lighting might have changed, etc. which would cause all the colors
// to be completely wrong).
const double kProbColorMatch = getenv("PROB_COLOR_MATCH") ? atof(getenv("PROB_COLOR_MATCH")) : 0.05;

// How much to care about color as a function of the particle sampling resolution.
// When we are sampling sparsely, we do not expect the colors to align well.
// Set to 0 to ignore this term.
// If non-zero, we set prob_color_match_ = kProbColorMatch *
//    exp(-pow(sampling_resolution, 2) / (2 * pow(kColorThreshFactor, 2));
const double kColorThreshFactor = getenv("COLOR_THRESH_FACTOR") ? atof(getenv("COLOR_THRESH_FACTOR")) : 10;

// Which color space to use for our color matches.
// 0: Use blue and green,
// 1: Use (R + G + B) / 3.
const int kColorSpace = getenv("COLOR_SPACE") ? atoi(getenv("COLOR_SPACE")) : 0;

// ------------------

// Use caching, unless we use color, in which case we can't use caching.
const bool use_caching = !kUseColor;

/*const bool kUse_annealing = getenv("USE_ANNEALING");

const double pi = boost::math::constants::pi<double>();

// We model each point as a Gaussian: exp(-x^2 / 2 sigma^2)
// With sigma = velodyne_horizontal_res * kSigmaFactor.
const double kSigmaFactor = getenv("SIGMA_FACTOR") ? atof(getenv("SIGMA_FACTOR")) : 1;

// We multiply our log measurement probability by this factor, to decrease
// our confidence in the probabilities (e.g. take into account
// dependencies between neighboring points).
const double kMeasurementDiscountFactor = getenv("MEASUREMENT_DISCOUNT_3D") ? atof(getenv("MEASUREMENT_DISCOUNT_3D")) : 0.9;

const double kSigmaGridFactor = getenv("SIGMA_GRID_FACTOR") ? atof(getenv("SIGMA_GRID_FACTOR")) : 2;

const double kSmoothingFactor = getenv("SMOOTHING_FACTOR_3D") ? atof(getenv("SMOOTHING_FACTOR_3D")) : 0.1;

// ------------------------

const double kSearchTreeEpsilon = getenv("SEARCH_TREE_EPSILON") ? atof(getenv("SEARCH_TREE_EPSILON")) : 2;

const double kReductionFactor = getenv("REDUCTION_FACTOR") ? atof(getenv("REDUCTION_FACTOR")) : 3;

const double kMinMeasurementVariance = getenv("MIN_MEASUREMENT_VAR") ? atof(getenv("MIN_MEASUREMENT_VAR")) : 0.01;


// How far to spill over in the occupancy map (number of sigmas).
//const double kSpilloverRadius = getenv("SPILLOVER_RADIUS") ? atof(getenv("SPILLOVER_RADIUS")) : 2.0;

//const bool doNearestKSearch = getenv("NEAREST_K_SEARCH");

// The probablility that a point has no match.
//const double kProbNoMatch = getenv("PROB_NO_MATCH") ? atof(getenv("PROB_NO_MATCH")) : 0.1;

// Whether to use a Laplacian distribution, instead of a Gaussian.
//const bool kUseLaplacian = getenv("USE_LAPLACIAN");

//const int kMinSpillover = getenv("MIN_SPILLOVER") ? atoi(getenv("MIN_SPILLOVER")) : 0;

const bool use_lf_tracker = getenv("USE_LF");

const bool kUseMotion = getenv("USE_MOTION");

// Color Parameters.
const bool kUseColor = getenv("USE_COLOR");
const bool kTwoColors = getenv("TWO_COLORS");

const double kValueSigma1 = getenv("VALUE_SIGMA") ? atof(getenv("VALUE_SIGMA")) : 20;
const double kValueSigma2 = getenv("VALUE_SIGMA2") ? atof(getenv("VALUE_SIGMA2")) : 20;

const double kColorThreshFactor = getenv("COLOR_THRESH_FACTOR") ? atof(getenv("COLOR_THRESH_FACTOR")) : 2;

const double kProbColorMatch = getenv("PROB_COLOR_MATCH") ? atof(getenv("PROB_COLOR_MATCH")) : 0.2;

// 0 = blue and green, 1 = mean of RGB.
const int kColorSpace = getenv("COLOR_SPACE") ? atoi(getenv("COLOR_SPACE")) : 0;

// ------------------

// Use caching, unless we use color, in which case we can't use caching.
const bool use_caching = !kUseColor;*/

const bool kUseClosestMotion = getenv("USE_CLOSEST_MOTION");

// Total size = 3.7 GB
// At a resolution of 1.2 cm, a 10 m wide object will take 1000 cells.
const int kMaxXSize = use_lf_tracker ? 1000 : 1;
const int kMaxYSize = use_lf_tracker ? 1000 : 1;
// At a resolution of 1.2 cm, a 5 m tall object will take 500 cells.
const int kMaxZSize = use_lf_tracker ? 500 : 1;

const double kResolution = 0.01;

}


LFDiscrete3d::LFDiscrete3d()
    : lf_cache_3D_log_prob_(kMaxXSize * kMaxYSize * kMaxZSize),
      lf_cache_3D_cached_(kMaxXSize * kMaxYSize * kMaxZSize),
      searchTree_(false),  //  //By setting sorted to false,
                                // the radiusSearch operations will be faster.
      min_occupancy_(kSmoothingFactor),
      max_nn_(1),
      nn_indices_(max_nn_),
      nn_sq_dists_(max_nn_),
      fast_functions_(FastFunctions::getInstance()),
      color_exp_factor1_(-1.0 / kValueSigma1),
      color_exp_factor2_(-1.0 / kValueSigma2)
  {
  }

void LFDiscrete3d::setPrevPoints(
    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > prev_points) {
  prev_points_ = prev_points;

  // Find the min and max of the previous points.
  pcl::getMinMax3D(*prev_points, minPt_orig_, maxPt_orig_);

  z_range_ = maxPt_orig_.z - minPt_orig_.z;

  //printf("Setting resolution\n");
  //searchTree_.setResolution(10);

  //printf("Deleting tree\n");
  //searchTree_.deleteTree();

  // Set the input cloud for the search tree to the previous points for NN
  // lookups.
  int original_no_of_points = static_cast<int> (prev_points_->points.size ());
  //printf("original_no_of_points: %d\n", original_no_of_points);
  searchTree_.setInputCloud(prev_points_);
  //printf("Adding points to cloud\n");
  //searchTree_.addPointsFromInputCloud ();
  //printf("Added points to cloud\n");

  // Set epsilon to 2 for a speedup.
  searchTree_.setEpsilon(kSearchTreeEpsilon);
}

void LFDiscrete3d::init(const double xy_grid_step,
          const double z_grid_step,
          const double horizontal_distance,
          const double down_sample_factor) {
  xy_grid_step_ = xy_grid_step;
  z_grid_step_ = z_grid_step;

  /*printf("Setting new resolution\n");
  searchTree_.setResolution(xy_grid_step_*10);
  printf("Set new resolution\n");*/

  const double z_centering = fabs(z_grid_step_ - z_range_) / 2;

  const double epsilon = 0.0001;

  // Add some padding.
  minPt_.x = minPt_orig_.x - (2 * xy_grid_step_ + epsilon);
  minPt_.y = minPt_orig_.y - (2 * xy_grid_step_ + epsilon);
  minPt_.z = minPt_orig_.z - (2 * z_grid_step_ + z_centering);

  min_center_pt_.x = minPt_.x + xy_grid_step_ / 2;
  min_center_pt_.y = minPt_.y + xy_grid_step_ / 2;
  min_center_pt_.z = minPt_.z + z_grid_step_ / 2;

  maxPt_.x = maxPt_orig_.x + 2 * xy_grid_step_;
  maxPt_.y = maxPt_orig_.y + 2 * xy_grid_step_;
  maxPt_.z = maxPt_orig_.z + 2 * z_grid_step_;

  // Find the appropriate size for the occupancy grid.
  xSize_ = min(kMaxXSize, max(1, static_cast<int>(
      ceil((maxPt_.x - minPt_.x) / xy_grid_step_))));
  ySize_ = min(kMaxYSize, max(1, static_cast<int>(
      ceil((maxPt_.y - minPt_.y) / xy_grid_step_))));
  zSize_ = min(kMaxZSize, max(1, static_cast<int>(
      ceil((maxPt_.z - minPt_.z) / z_grid_step_))));

  xySize_ = xSize_ * ySize_;

  //printf("Size: %d, %d, %d\n", xSize_, ySize_, zSize_);

  /*std::vector<std::vector<std::vector<bool> > > vect1(xSize,
            vector<vector<bool> >(ySize, vector<bool>(zSize, false)));*/

  // Resize the likelihood field cache.
  //lf_cache_3D_ = std::vector<std::vector<std::vector<LFVal> > > (xSize,
  //    vector<vector<LFVal> >(ySize, vector<LFVal>(zSize)));

  /*lf_cache_3D_prob_.resize(xSize);
  lf_cache_3D_cached_.resize(xSize);
  for (size_t i = 0; i < xSize; ++i) {
    lf_cache_3D_prob_[i].resize(ySize);
    lf_cache_3D_cached_[i].resize(ySize);
    for (size_t j = 0; j < ySize; ++j) {
      lf_cache_3D_prob_[i][j].resize(zSize);
      lf_cache_3D_cached_[i][j].resize(zSize, false);
    }
  }*/

  //lf_cache_3D_.resize(xSize);
  /*for (int i = 0; i < xSize_; ++i) {
    //lf_cache_3D_[i].resize(ySize);
    for (int j = 0; j < ySize_; ++j) {
      std::fill(
          lf_cache_3D_cached_[i][j].begin(),
          lf_cache_3D_cached_[i][j].begin() + zSize_, false);
    }
  }*/
  std::fill(lf_cache_3D_cached_.begin(),
      lf_cache_3D_cached_.begin() + xSize_ * ySize_ * zSize_, false);

  /*std::vector<std::vector<std::vector<bool> > > vect1(xSize,
          vector<vector<bool> >(ySize, vector<bool>(zSize, false)));
  std::vector<std::vector<std::vector<double> > > vect2(xSize,
      vector<vector<double> >(ySize, vector<double>(zSize)));*/

  /*vect1.resize(xSize);
  vect2.resize(xSize);
  for (size_t i = 0; i < xSize; ++i) {
    vect1[i].resize(ySize);
    vect2[i].resize(ySize);
    for (size_t j = 0; j < ySize; ++j) {
      vect1[i][j].resize(zSize);
      vect2[i][j].resize(zSize);
    }
  }*/

  // Build the occupancy map.
   //vector<vector<vector<double> >  > occupancyMap3D(xSize,
   //    vector<vector<double> >(ySize, vector<double>(zSize, 0)));

  //printf("Grid size: %d, %d, %d\n", *xSize, *ySize, *zSize);

  // Set the minimum occupancy of all grid cells.
  //const size_t num_prev_points = prev_points->size();

  // Ratio of the previous model size to the current model size.
  // If less than 1, then this indicates the chance of a match to the current
  // model.
  // Example: If the previous model had 50 points and the current model has
  // 200, the chance of a match is 50 / 200.  The chance of no match is 150 / 200.
  //const double prob_no_match1 = 1 - min(point_ratio, 1.0);

  //const double prob_no_match = prob_no_match1 + (1 - prob_no_match1) * kProbNoMatch;
  //const double prob_no_match = kProbNoMatch;

  // Compute the sensor horizontal resolution
  const double velodyne_horizontal_res_actual = 2 * horizontal_distance * tan(.18 / 2.0 * pi / 180.0);

  // The effective resolution = resolution / downsample factor.
  const double velodyne_horizontal_res = velodyne_horizontal_res_actual / down_sample_factor;

  // The vertical resolution for the Velodyne is 2.2 * the horizontal resolution.
  const double velodyne_vertical_res = 2.2 * velodyne_horizontal_res;
  //const double velodyne_vertical_res = velodyne_horizontal_res;


  // Compute the variance to use for the GMM.
  //const double spillover_sigma_xy = velodyne_horizontal_res * kSigmaFactor;
  //const double spillover_sigma_z = velodyne_vertical_res * kSigmaFactor;

  //double spillover_sigma;
  /*if (kUseSmallSigma) {
    spillover_sigma = velodyne_horizontal_res * kSigmaFactor;
  } else {
    if (kCombineErrors) {*/

  const double error1_xy = kUse_annealing ? kSigmaGridFactor * xy_grid_step_ : 0;
  const double error2_xy = velodyne_horizontal_res * kSigmaFactor;
  const double spillover_sigma_xy = sqrt(pow(error1_xy, 2) + pow(error2_xy, 2) + pow(kMinMeasurementVariance, 2));

  const double error1_z = kUse_annealing ? kSigmaGridFactor * z_grid_step_ : 0;
  const double error2_z = velodyne_vertical_res * kSigmaFactor;
  const double spillover_sigma_z = sqrt(pow(error1_z, 2) + pow(error2_z, 2) + pow(kMinMeasurementVariance, 2));

  // Convert the sigma to a factor such that
  // exp(-x^2 / 2 sigma^2) = exp(x^2 * factor)
  // where x is the distance.
  xy_exp_factor_ = -1.0 / (2 * pow(spillover_sigma_xy, 2));

  z_exp_factor_ = -1.0 / (2 * pow(spillover_sigma_z, 2));

  // TODO - try using z as well.
  /*search_radius_ = kSpilloverRadius * spillover_sigma_xy;


  const int max_index = search_radius_ / kResolution;
  prob_spatials_.resize(max_index);

  for (int i = 0; i < max_index; ++i) {
    const double distance = i * kResolution;
    const double prob = fast_functions_.getFastLog(
        exp(distance * xy_exp_factor) + min_occupancy_);
    prob_spatials_[i] = prob;
  }

  default_val_ = log(min_occupancy_);*/

  // Set color params.
  if (kColorThreshFactor == 0) {
    prob_color_match_ = kProbColorMatch;
  } else {
    prob_color_match_ = kProbColorMatch * exp(-pow(xy_grid_step_, 2) /
        (2 * pow(kColorThreshFactor, 2)));
  }
}

LFDiscrete3d::~LFDiscrete3d() {
  // TODO Auto-generated destructor stub
}

void LFDiscrete3d::scoreXYZTransforms(
    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >& current_points,
    const double xy_stepSize,
    const double z_stepSize,
    const vector<XYZTransform>& transforms,
    const MotionModel& motion_model,
    const double horizontal_distance,
    const double down_sample_factor,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms) {
  // Initialize variables for tracking grid.
  init(xy_stepSize, z_stepSize, horizontal_distance, down_sample_factor);

  const size_t num_transforms = transforms.size();

  // Compute scores for all of the transforms using the occupancy map.
  scored_transforms->clear();
  scored_transforms->resize(num_transforms);

  for(size_t i = 0; i < num_transforms; ++i){
    const XYZTransform& transform = transforms[i];
    const double& x = transform.x;
    const double& y = transform.y;
    const double& z = transform.z;
    const double& volume = transform.volume;

    const double log_prob = getLogProbability(current_points,
        motion_model, x, y, z);

    // Save the complete transform with its log probability.
    const ScoredTransformXYZ scored_transform(x, y, z,
        log_prob, volume);
    scored_transforms->set(scored_transform, i);
  }
}



double LFDiscrete3d::getLogProbability(
    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >& current_points,
    const MotionModel& motion_model,
    const double x,
    const double y,
    const double z) {

  // Total log measurement probability.
  double log_measurement_prob = 0;

  // Compute the grid offset to apply to each point based on the proposed
  // displacement (x, y, z).
  const double x_offset = (x - minPt_.x) / xy_grid_step_;
  const double y_offset = (y - minPt_.y) / xy_grid_step_;
  const double z_offset = (z - minPt_.z) / z_grid_step_;

  // Iterate over every point, and look up its score in the occupancy map.
  const size_t num_points = current_points->size();

  for (size_t i = 0; i < num_points; ++i) {
    // Extract the point so we can compute its score.
    const pcl::PointXYZRGB& pt = (*current_points)[i];

    // TODO - compare the speed here to branching.
    // Compute the grid-cell for this point.
    // Cut off indices at the limits.
    const int x_index = min(max(0,
        static_cast<int>(pt.x / xy_grid_step_ + x_offset)), xSize_ - 1);
    const int y_index = min(max(0,
        static_cast<int>(pt.y / xy_grid_step_ + y_offset)), ySize_ - 1);
    const int z_index = min(max(0,
        static_cast<int>(pt.z / z_grid_step_ + z_offset)), zSize_ - 1);

    // Look up the score in the occupancy map.
    //log_measurement_prob += occupancyMap3D_[x_index_shifted][y_index_shifted][z_index_shifted];

    const int index = get_index(x_index, y_index, z_index);

    // Find if the object was cached.
    const bool cached = lf_cache_3D_cached_[index];

    // Compute the probability, if necessary, and cache.
    if (!cached || !use_caching){
      lf_cache_3D_log_prob_[index] = get_log_prob(x_index, y_index, z_index,
          x, y, z, pt);
      lf_cache_3D_cached_[index] = true;
    }

    log_measurement_prob += lf_cache_3D_log_prob_[index];
  }

  // Compute the motion model probability.
  double motion_model_prob;
  //printf("motion_model_prob: %lf\n", motion_model_prob);
  /*if (!kUseMotion) {
    motion_model_prob = 1;
  } else if (kUseClosestMotion) {*/
    motion_model_prob = motion_model.computeScore(x, y, z);
  /*} else {
    motion_model_prob = motion_model.computeScore(x, y, z);
  }*/

  // Combine the motion model score with the (discounted) measurement score to
  // get the final log probability.
  const double log_prob =
      log(motion_model_prob) + kMeasurementDiscountFactor * log_measurement_prob;

  return log_prob;
}

double LFDiscrete3d::get_log_prob(
    const int x_index, const int y_index, const int z_index, //) {//,
    const double x_vel, const double y_vel, const double z_vel,
    const pcl::PointXYZRGB& pt_unshifted) {
  // Position for lookup.
  //double x, y, z;

  if (use_caching) {
    // Get the 3d position for the center of this grid cell.
    pt_.x = (x_index * xy_grid_step_ + min_center_pt_.x);
    pt_.y = (y_index * xy_grid_step_ + min_center_pt_.y);
    pt_.z = (z_index * z_grid_step_ + min_center_pt_.z);
  } else {
    pt_.x = pt_unshifted.x + x_vel;
    pt_.y = pt_unshifted.y + y_vel;
    pt_.z = pt_unshifted.z + z_vel;
  }

  /*pt_.x = x;
  pt_.y = y;
  pt_.z = z;*/
  pt_.rgb = pt_unshifted.rgb;

  //const pcl::PointXYZ pt3(x, y, z);

  //TODO - try using octtree with radiusSearch.
  //int numNeighbors;
  //if (doNearestKSearch) {
    searchTree_.nearestKSearch(pt_, max_nn_, nn_indices_, nn_sq_dists_);
    //searchTree_.approxNearestSearch(pt_, nn_index_, nn_sq_dist_);
  /*} else {
    searchTree_.radiusSearch(pt_, search_radius_,
        nn_indices_, nn_sq_dists_, max_nn_);
  }*/


  // Store the sum of the nearest neighbor probabilities.
  //double sum_exp = 0;

  // The default to add to each point.
  //const double use_k2 = min_occupancy_;

  // Loop over all neighbors.
  //for (int i = 0; i < numNeighbors; ++i) {
    // TODO - why do we do this?
    // If we have a match, set k2 to minimum occupancy (not sure why).
    //use_k2 = min_occupancy_;

    // Find the neighbor.
    const pcl::PointXYZRGB& prev_pt = (*prev_points_)[nn_indices_[0]];

    //printf("Points: %lf, %lf, %lf and %lf, %lf, %lf\n", prev_pt.x, prev_pt.y,
    //    prev_pt.z, pt_.x, pt_.y, pt_.z);

    // Compute the weighted distance to the neighbor.
    //const double distance_xy_sq = pow(prev_pt.x - pt_.x, 2) + pow(prev_pt.y - pt_.y, 2);
    //const double distance_z_sq = pow(prev_pt.z - pt_.z, 2);

    // Compute the log probability of this neighbor match.
    const double log_point_match_prob_i = nn_sq_dists_[0] * xy_exp_factor_;
        //distance_xy_sq * xy_exp_factor_ + distance_z_sq * z_exp_factor_;

    //printf("%lf %lf %lf %lf\n", distance_xy_sq, distance_z_sq, xy_exp_factor_,
    //    z_exp_factor_);

    // Compute the probability of this neighbor match.
    const double point_match_prob_spatial_i = exp(
        log_point_match_prob_i);
    //printf("nn_sq_dists_[0]: %lf, log val: %lf, exp: %lf\n", nn_sq_dists_[0],
    //       log_point_match_prob_i, point_match_prob_spatial_i);

    double point_prob;
    if (kUseColor) {
      point_prob = computeColorProb(prev_pt, pt_,
          point_match_prob_spatial_i);
      //printf("point_prob: %lf\n", point_prob);
    } else {
      // Compute the probability for this point.
      point_prob = point_match_prob_spatial_i + min_occupancy_;
    }

    // Update the total probability of all matches.
    //sum_exp += point_match_prob_i;
  //}

    // Compute the log.
    const double log_point_prob = log(point_prob); //fast_functions_.getFastLog(point_prob);

  //const double dist = nn_sq_dists_[0];
  //const double dist = nn_sq_dist_;

  //const double log_point_prob = dist < search_radius_ ?
  //    prob_spatials_[dist / kResolution] : default_val_;

  return log_point_prob;
}

double LFDiscrete3d::computeColorProb(const pcl::PointXYZRGB& prev_pt,
    const pcl::PointXYZRGB& pt, const double point_match_prob_spatial_i) const {
  const double factor1 = min_occupancy_ / (min_occupancy_ + 1);

  double use_k2;
  if (kUseColor && !kTwoColors) {
    use_k2 = factor1 / 255;
  } else if (kUseColor && kTwoColors) {
    use_k2 = factor1 / pow(255, 2);
  } else {
    use_k2 = min_occupancy_;
    //use_k2 /= (product_of_standard_devs_spatial * pow(2 * pi, 1.5));
  }

  // Find the colors from the 2 points.
  int color1 = 0, color2 = 0, color3 = 0, color4 = 0;
  if (kColorSpace == 0) {
    // Blue and Green.
    color1 = pt.b;
    color2 = prev_pt.b;
    //printf("Colors: %d, %d\n", color1, color2);

    color3 = pt.g;
    color4 = prev_pt.g;
  } else if (kColorSpace == 1) {
    // Mean of RGB.
    color1 = (pt.r + pt.g + pt.b) / 3;
    color2 = (prev_pt.r + prev_pt.g + prev_pt.b) / 3;
  } else {
    printf("Unknown color space: %d\n", kColorSpace);
    exit(1);
  }

  const double color_distance1 = fabs(color1 - color2);
  //printf("Color distance: %lf\n", color_distance1);

  double point_match_prob;
  if (kTwoColors) {
    const double color_distance2 = fabs(color3 - color4);

    point_match_prob = point_match_prob_spatial_i * ((1-prob_color_match_) * 1.0 / pow(255, 2) +
        prob_color_match_ * (
            -0.5 * color_exp_factor1_ * exp(color_distance1 * color_exp_factor1_)) *
            -0.5 * color_exp_factor2_ * exp(color_distance2 * color_exp_factor2_));

  } else {
    point_match_prob = point_match_prob_spatial_i * ((1-prob_color_match_) * 1.0 / 255 +
        prob_color_match_ * -1 * color_exp_factor1_ *
        exp(color_distance1 * color_exp_factor1_));
  }

  //sum_exp += f;

  //sum_exp += exp(log_point_match_prob) * 1.0 / 255;

  use_k2 *= (1 - point_match_prob_spatial_i);

  //printf("%lf %lf %lf\n", point_match_prob_spatial_i, point_match_prob, use_k2);

  const double point_prob = point_match_prob + use_k2;

  return point_prob;
}
