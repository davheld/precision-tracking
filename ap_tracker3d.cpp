/*
 * ap_tracker_3d.cpp
 *
 *  Created on: Sep 1, 2013
 *      Author: davheld
 */

#include "ap_tracker3d.h"

#include <vector>
#include <algorithm>

#include <boost/math/constants/constants.hpp>

#include "density_grid_tracker.h"
#include "scored_transform.h"

using std::vector;
using std::max;

namespace {

const bool k_NNTracking = false;
const bool use_lf_tracker = false;

//const double min_transforms = getenv("MIN_TRANSFORMS") ? atoi(getenv("MIN_TRANSFORMS")) : 1;

const double pi = boost::math::constants::pi<double>();

const double kMinResFactor = getenv("MIN_RES_FACTOR") ? atof(getenv("MIN_RES_FACTOR")) : 1;

const bool kSearchYaw = false;

const double kMaxZ = getenv("MAX_Z") ? atof(getenv("MAX_Z")) : 1;

const double kMinXYStep = getenv("MIN_XY_STEP") ? atof(getenv("MIN_XY_STEP")) : 0.05;

const double kReductionFactor = getenv("REDUCTION_FACTOR") ? atof(getenv("REDUCTION_FACTOR")) : 3;

const size_t kMaxNumTransformsActual = getenv("MAX_NUM_ACTUAL") ? atoi(getenv("MAX_NUM_ACTUAL")) : 0;

const double kMaxNumTransformsKNN = getenv("NN_MAX_NUM_TRANSFORMS") ?
    atoi(getenv("NN_MAX_NUM_TRANSFORMS")) : 10000;

const double kMaxNumTransformsGrid = getenv("MAX_NUM_TRANSFORMS") ?
    atoi(getenv("MAX_NUM_TRANSFORMS")) : 10000;

} // namespace

APTracker3d::APTracker3d()
  :lf_discrete_3d_(LFDiscrete3d::getInstance()),
   density_grid_tracker_(DensityGridTracker::getInstance()),
   fast_functions_(FastFunctions::getInstance()){
  // TODO Auto-generated constructor stub
}

APTracker3d::~APTracker3d() {
  // TODO Auto-generated destructor stub
}

void APTracker3d::recomputeProbs(
    const double prior_prob,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms) const {
  const std::vector<double>& conditional_probs =
      scored_transforms->getNormalizedProbs();

  std::vector<ScoredTransformXYZ>& scored_transforms_vect =
      scored_transforms->getScoredTransforms();

  size_t num_probs = conditional_probs.size();

  for (size_t i = 0; i < num_probs; ++i) {
    const double& conditional_prob = conditional_probs[i];
    const double new_log_prob = log(prior_prob * conditional_prob);
    scored_transforms_vect[i].setUnnormalizedLogProb(new_log_prob);
  }
}

void APTracker3d::track(
    const double& xy_step_size,
    const double& z_step_size,
    const std::pair <double, double>& xRange,
    const std::pair <double, double>& yRange,
    const std::pair <double, double>& zRange,
    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > current_points,
    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > prev_points,
    const Eigen::Vector3f &current_points_centroid,
    const MotionModel& motion_model,
    const int coarse_search_num_points,
    const double horizontal_distance,
    const double down_sample_factor_prev,
    const double point_ratio,
    ScoredTransforms<ScoredTransformXYZ>* final_scored_transforms3D) {
  const double kMaxXYStepSize = xy_step_size;

  // Compute the sensor horizontal resolution
  //const double velodyne_horizontal_res = 2 * horizontal_distance * tan(.18 / 2.0 * pi / 180.0);

  // Compute the sensor horizontal resolution.
  const double velodyne_horizontal_res_actual = 2 * horizontal_distance * tan(.18 / 2.0 * pi / 180.0);

  // The effective resolution = resolution / downsample factor.
  const double velodyne_horizontal_res = velodyne_horizontal_res_actual / down_sample_factor_prev;

  const double kMinXYStepSize =
      max(velodyne_horizontal_res / kMinResFactor, kMinXYStep);

  // Place to store the final scored transforms.
  //ScoredTransforms<ScoredTransformXYZ> final_scored_transforms3D;

  // Initialize the xy step size to the max.
  double current_xy_step_size = kMaxXYStepSize;

  // Based on the ratio of horizontal to vertical resolution.
  double current_z_step_size = 2.2 * kMaxXYStepSize;
  //double current_z_step_size = kMaxXYStepSize;
  //double current_z_step_size = 0.5;

  // Track at a low resolution, get a score for various 2D transforms.
  ScoredTransforms<ScoredTransformXYZ> scored_transforms3D;

  if (k_NNTracking && use_lf_tracker) {
    lf_discrete_3d_.setPrevPoints(prev_points);
  }

  if (k_NNTracking) {
    if (use_lf_tracker) {
      lf_discrete_3d_.track(current_xy_step_size, current_z_step_size, xRange, yRange, zRange,
          current_points, current_points_centroid,
          motion_model, horizontal_distance, down_sample_factor_prev, point_ratio,
          &scored_transforms3D);
    }
  } else {
    density_grid_tracker_.track(
        current_xy_step_size, current_z_step_size, xRange, yRange, zRange,
        current_points, prev_points, current_points_centroid,
        motion_model, horizontal_distance, down_sample_factor_prev, point_ratio,
        &scored_transforms3D);
  }

  // Recompute the probability of each of these transforms using the prior
  // probability.
  recomputeProbs(1, &scored_transforms3D);

  // Save previous output (the part that we are not recomputing) to the
  // final scored transforms.
  final_scored_transforms3D->addScoredTransforms(scored_transforms3D);

  while (current_xy_step_size > kMinXYStepSize) {
    // To get the new positions, offset each old position by the old position
    // by the old resolution / 4 (we divide each grid cell into 4 blocks).
    const double new_xy_step_size = current_xy_step_size / kReductionFactor;
    const double new_z_step_size = current_z_step_size / kReductionFactor;

    // Make new transforms at a higher resolution.
    vector<XYZTransform> new_xyz_transforms;
    // Keep track of the total probability of the region that we are recomputing
    // the probability of.
    double total_recomputing_prob;
    makeNewTransforms3D(new_xy_step_size, new_z_step_size, k_NNTracking,
        final_scored_transforms3D, &new_xyz_transforms,
        &total_recomputing_prob);

    if (new_xyz_transforms.size() > 0) {
      scored_transforms3D.clear();
      // Recompute some of the transforms at a higher resolution.
      if (k_NNTracking) {
          lf_discrete_3d_.scoreXYZTransforms(
                  current_points, current_points_centroid,
                  new_xy_step_size, new_z_step_size, new_xyz_transforms, motion_model,
                  horizontal_distance, down_sample_factor_prev, point_ratio,
                  &scored_transforms3D);
      } else {
        density_grid_tracker_.scoreXYZTransforms(
            current_points, prev_points, current_points_centroid,
            new_xy_step_size, new_z_step_size, new_xyz_transforms, motion_model,
            horizontal_distance, down_sample_factor_prev, point_ratio,
            &scored_transforms3D);
      }

      // Recompute the probability of each of these transforms using the prior
      // probability.
      recomputeProbs(total_recomputing_prob, &scored_transforms3D);

      // Add the previous output to the final scored transforms.
      final_scored_transforms3D->addScoredTransforms(scored_transforms3D);

      current_xy_step_size = new_xy_step_size;
      current_z_step_size = new_z_step_size;

    } else {
      break;
    }
  }

  /*if (kSearchYaw) {
    ScoredTransformXYZ best_transform;
    double best_transform_prob;
    final_scored_transforms3D->findBest(&best_transform, &best_transform_prob);

    /*Eigen::Vector4d centroid4d = Eigen::Vector4d::Zero();
    centroid4d(0) = current_points_centroid(0);
    centroid4d(1) = current_points_centroid(1);
    centroid4d(2) = current_points_centroid(2);
    centroid4d(3) = 1;

    const double time_diff = 0.1;

    Eigen::Vector3d mean_velocity = motion_model.computeMeanVelocity(*final_scored_transforms3D,
        centroid4d, time_diff);

    const double x = mean_velocity(0) * time_diff;
    const double y = mean_velocity(1) * time_diff;
    const double z = mean_velocity(2) * time_diff;

    ScoredTransformXYZ mean_transform(x, y, z, 0, 0, 0, 0, 1);*/

    /*nn_tracker_.findYaw(
        current_points, prev_points,
        current_xy_step_size, current_z_step_size,
        current_points_centroid,
        best_transform,
        motion_model,
        horizontal_distance, down_sample_factor_prev, point_ratio,
        &scored_transforms3D);

    recomputeProbs(best_transform_prob, &scored_transforms3D);

    // Add the previous output to the final scored transforms.
    final_scored_transforms3D->clear();
    final_scored_transforms3D->addScoredTransforms(scored_transforms3D);
  }*/

  //showScoredTransforms(final_scored_transforms2D);
}

bool compareTransforms1(const ScoredTransformXYZ& transform_i,
    const ScoredTransformXYZ& transform_j) {
  return transform_i.getUnnormalizedLogProb() > transform_j.getUnnormalizedLogProb();
}

// Decreases the resolution of all grid cells above a certain threshold probability.
void APTracker3d::makeNewTransforms3D(
    const double xy_offset, const double z_offset, const bool usekNN,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms,
    std::vector<XYZTransform>* new_xyz_transforms,
    double* total_recomputing_prob) const {
  std::vector<ScoredTransformXYZ>& scored_transforms_xyz =
      scored_transforms->getScoredTransforms();

  if (kMaxNumTransformsActual > 0) {
    std::sort(scored_transforms_xyz.begin(), scored_transforms_xyz.end(),
      compareTransforms1);
  }

  /*const std::vector<ScoredTransformXY>& scored_transforms_xy =
      scored_transforms->getScoredTransforms();*/

  // This will recompute all of the old probabilities.
  // TODO - Scored transform should contain an XY transform.
  /*const size_t num_transforms = scored_transforms_xy.size();
  new_xy_transforms->clear();
  new_xy_transforms->reserve(num_transforms);
  for (size_t i = 0; i < num_transforms; ++i) {
    const ScoredTransformXY& scored_transform = scored_transforms_xy[i];
    XYTransform new_transform(
        scored_transform.getX(),
        scored_transform.getY(),
        scored_transform.getArea());
    new_xy_transforms->push_back(new_transform);
  }*/

  // To get the new positions, offset each old position by the old position
  // by the old resolution / 4 (we divide each grid cell into 4 blocks).
  //const double xy_offset = old_xy_step / 3;
  //*new_xy_step = xy_offset;

  //const double z_offset = max(0.5, old_z_step / 3);
  //const double z_offset = old_z_step / 3;
  //*new_z_step = z_offset;

  vector<size_t> to_remove;

  const double volume = pow(xy_offset, 2) * z_offset;

  // Keep track of the total probability of the region that we are recomputing
  // the probability of.
  *total_recomputing_prob = 0;

  double prob_sum = 0;

  int kMaxNumTransforms;
  if (usekNN) {
    kMaxNumTransforms = kMaxNumTransformsKNN;
  } else {
    kMaxNumTransforms = kMaxNumTransformsGrid;
  }

  // Only recompute probabilities that are greater than min_prob.
  const double min_prob = kMaxNumTransforms == 0 ? 0 : 1.0 / kMaxNumTransforms;
  //const double min_prob = 0;

  const std::vector<double>& probs = scored_transforms->getNormalizedProbs();

  // Allocate space for the new transforms that we will recompute.
  const size_t max_num_transforms = kMaxNumTransformsActual > 0 ? std::min(probs.size(), kMaxNumTransformsActual) : probs.size();
  new_xyz_transforms->clear();
  new_xyz_transforms->reserve(max_num_transforms);

  //double max_prob = 0;

  // For each old transform with probability greater than the minimum
  // threshold, create 4 new transforms at higher resolution.
  for (size_t i = 0; i < max_num_transforms; ++i) {
    prob_sum += probs[i];

    const ScoredTransformXYZ& old_scored_transform = scored_transforms_xyz[i];
    const double old_x = old_scored_transform.getX();
    const double old_y = old_scored_transform.getY();
    const double old_z = old_scored_transform.getZ();

    if (probs[i] > min_prob) {
      *total_recomputing_prob += probs[i];

      to_remove.push_back(i);

      // Create 4 new transforms, offset from the old one.
      for (int i = -1; i <= 1; ++i) {
        const double new_x = old_x + xy_offset * i;
        for (int j = -1; j <= 1; ++j) {
          const double new_y = old_y + xy_offset * j;
          //for (int k = -1; k <= 1; ++k) {
            const double new_z = old_z; // + z_offset * k;
            XYZTransform new_transform(new_x, new_y, new_z, volume);
            new_xyz_transforms->push_back(new_transform);
          //}
        }
      }
    }
  }

  // Remove old items.
  for (int i = static_cast<int>(to_remove.size() - 1); i >= 0; --i) {
    size_t remove_index = to_remove[i];
    scored_transforms_xyz.erase(scored_transforms_xyz.begin() + remove_index);
  }
}

