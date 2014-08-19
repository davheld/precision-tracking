/*
 * adh_tracker_3d.cpp
 *
 *  Created on: Sep 1, 2013
 *      Author: davheld
 */

#include "adh_tracker3d.h"

#include <vector>
#include <algorithm>

#include "density_grid_tracker.h"
#include "scored_transform.h"

using std::vector;
using std::max;

namespace {

// Whether to include color probabilities when performing the alignment.
const bool use_color = false;

// Whether to use the density grid tracker (pre-caching) or the lf_tracker
// (post-caching).  The LF_tracker is slightly more accurate but about
// twice as slow.
const bool use_lf_tracker = true;

// We compute the minimum sampling resolution based on the sensor
// resolution - we are limited in accuracy by the sensor resolution,
// so there is no point in sampling at a much finer scale.
// The minimum sampling resolution is set to be no smaller than
// sensor_resolution / kMinResFactor.
const double kMinResFactor = 1;

// The desired sampling resolution.
const double kDesiredSamplingResolution = 0.05;

// How much to reduce the sampling resolution each iteration.
const double kReductionFactor = 3;

// Set this to limit the maximum number of transforms that we
// evaluate at each iteration beyond the first.
const size_t kMaxNumTransforms = 0;

// Only divide cells whose probabilities are greater than kMinProb.
const double kMinProb = 0.0001;

} // namespace

ADHTracker3d::ADHTracker3d()
  :lf_discrete_3d_(LFDiscrete3d::getInstance()),
   density_grid_tracker_(DensityGridTracker::getInstance()),
   fast_functions_(FastFunctions::getInstance()){
  // TODO Auto-generated constructor stub
}

ADHTracker3d::~ADHTracker3d() {
  // TODO Auto-generated destructor stub
}

void ADHTracker3d::recomputeProbs(
    const double prior_prob,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms) const {
  // Get the conditional probabilities for the region that we subdivided,
  // p(Cell | Region)
  const std::vector<double>& conditional_probs =
      scored_transforms->getNormalizedProbs();

  std::vector<ScoredTransformXYZ>& scored_transforms_vect =
      scored_transforms->getScoredTransforms();

  // Compute the joint probability of each cell and the region.
  size_t num_probs = conditional_probs.size();
  for (size_t i = 0; i < num_probs; ++i) {
    const double conditional_prob = conditional_probs[i];
    // P(Cell, Region) = p(Region) p(Cell | Region)
    const double new_log_prob = log(prior_prob * conditional_prob);
    scored_transforms_vect[i].setUnnormalizedLogProb(new_log_prob);
  }
}

void ADHTracker3d::track(
    const double initial_xy_sampling_resolution,
    const double initial_z_sampling_resolution,
    const std::pair <double, double>& xRange,
    const std::pair <double, double>& yRange,
    const std::pair <double, double>& zRange,
    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > current_points,
    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > prev_points,
    const Eigen::Vector3f &current_points_centroid,
    const MotionModel& motion_model,
    const double horizontal_distance,
    const double down_sample_factor_prev,
    const double xy_sensor_resolution,
    const double z_sensor_resolution,
    ScoredTransforms<ScoredTransformXYZ>* final_scored_transforms3D) {
  // Compute the minimum sampling resolution based on the sensor
  // resolution - we are limited in accuracy by the sensor resolution,
  // so there is no point in sampling at a much finer scale.
  const double min_xy_sampling_resolution =
      max(xy_sensor_resolution / kMinResFactor, kDesiredSamplingResolution);

  // Initialize the sampling resolution.
  double current_xy_sampling_resolution = initial_xy_sampling_resolution;
  double current_z_sampling_resolution = initial_z_sampling_resolution;

  // Our most recently scored transforms.
  ScoredTransforms<ScoredTransformXYZ> scored_transforms3D;

  // Create initial candidate transforms.
  vector<XYZTransform> xyz_transforms;
  createCandidateXYZTransforms(
        current_xy_sampling_resolution, current_z_sampling_resolution,
        xRange, yRange, zRange, &xyz_transforms);

  // Initially track at a coarse resolution and get the probability of
  // various transforms.
  if (use_color) {
    lf_rgbd_6d_.setPrevPoints(prev_points);

    lf_rgbd_6d_.score3DTransforms(
          current_points, current_points_centroid,
          current_xy_sampling_resolution, current_z_sampling_resolution,
          xy_sensor_resolution, z_sensor_resolution, 1,
          xyz_transforms, motion_model,
          &scored_transforms3D);
  } else if (use_lf_tracker) {
    lf_discrete_3d_.setPrevPoints(prev_points);

    lf_discrete_3d_.scoreXYZTransforms(
            current_points,
            current_xy_sampling_resolution, current_z_sampling_resolution,
            xyz_transforms, motion_model,
            horizontal_distance, down_sample_factor_prev,
            &scored_transforms3D);

  } else {
    density_grid_tracker_.scoreXYZTransforms(
        current_points, prev_points,
        current_xy_sampling_resolution, current_z_sampling_resolution,
        xyz_transforms, motion_model,
        xy_sensor_resolution, z_sensor_resolution, &scored_transforms3D);
  }

  // Normalize the probabilities so they sum to 1.
  recomputeProbs(1, &scored_transforms3D);

  // Save the previous output to the final scored transforms.
  final_scored_transforms3D->addScoredTransforms(scored_transforms3D);

  while (current_xy_sampling_resolution > min_xy_sampling_resolution) {
    // To get the new positions, offset each old position by the old position
    // by the old resolution / 4 (we divide each grid cell into 4 blocks).
    const double new_xy_sampling_resolution =
        current_xy_sampling_resolution / kReductionFactor;
    const double new_z_sampling_resolution =
        current_z_sampling_resolution / kReductionFactor;

    // Make new transforms at a higher resolution.
    vector<XYZTransform> new_xyz_transforms;
    // Keep track of the total probability of the region that we are recomputing
    // the probability of.
    double total_recomputing_prob;
    makeNewTransforms3D(
          new_xy_sampling_resolution, new_z_sampling_resolution,
          final_scored_transforms3D, &new_xyz_transforms,
          &total_recomputing_prob);

    if (new_xyz_transforms.size() > 0) {
      scored_transforms3D.clear();
      // Recompute some of the transforms at a higher resolution.
      if (use_color) {
        lf_rgbd_6d_.score3DTransforms(
              current_points, current_points_centroid,
              new_xy_sampling_resolution, new_z_sampling_resolution,
              xy_sensor_resolution, z_sensor_resolution, 1,
              new_xyz_transforms, motion_model,
              &scored_transforms3D);
      } else if (use_lf_tracker) {
          lf_discrete_3d_.scoreXYZTransforms(
                  current_points,
                  new_xy_sampling_resolution, new_z_sampling_resolution, new_xyz_transforms, motion_model,
                  horizontal_distance, down_sample_factor_prev,
                  &scored_transforms3D);
      } else {
        density_grid_tracker_.scoreXYZTransforms(
            current_points, prev_points,
              new_xy_sampling_resolution, new_z_sampling_resolution,
              new_xyz_transforms, motion_model,
              xy_sensor_resolution, z_sensor_resolution, &scored_transforms3D);
      }

      // Recompute the probability of each of these transforms using the prior
      // probability.
      recomputeProbs(total_recomputing_prob, &scored_transforms3D);

      // Add the previous output to the final scored transforms.
      final_scored_transforms3D->addScoredTransforms(scored_transforms3D);

      current_xy_sampling_resolution = new_xy_sampling_resolution;
      current_z_sampling_resolution = new_z_sampling_resolution;

    } else {
      break;
    }
  }
}

bool compareTransforms1(const ScoredTransformXYZ& transform_i,
    const ScoredTransformXYZ& transform_j) {
  return transform_i.getUnnormalizedLogProb() > transform_j.getUnnormalizedLogProb();
}

// Decreases the resolution of all grid cells above a certain threshold probability.
void ADHTracker3d::makeNewTransforms3D(
    const double xy_offset, const double z_offset,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms,
    std::vector<XYZTransform>* new_xyz_transforms,
    double* total_recomputing_prob) const {
  std::vector<ScoredTransformXYZ>& scored_transforms_xyz =
      scored_transforms->getScoredTransforms();

  if (kMaxNumTransforms > 0) {
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

  const std::vector<double>& probs = scored_transforms->getNormalizedProbs();

  // Allocate space for the new transforms that we will recompute.
  const size_t max_num_transforms = kMaxNumTransforms > 0 ? std::min(probs.size(), kMaxNumTransforms) : probs.size();
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

    // Only divide cells whose probabilities are greater than kMinProb.
    if (probs[i] > kMinProb) {
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

void ADHTracker3d::createCandidateXYZTransforms(
    const double xy_sampling_resolution,
    const double z_sampling_resolution,
    const std::pair <double, double>& xRange,
    const std::pair <double, double>& yRange,
    const std::pair <double, double>& zRange_orig,
    std::vector<XYZTransform>* transforms) const {
  if (xy_sampling_resolution == 0) {
    printf("Error - xy sampling resolution must be > 0");
    exit(1);
  }

  if (z_sampling_resolution == 0) {
    if (zRange_orig.first != zRange_orig.second) {
      printf("Error - z_sampling_resolution = 0 but the z range"
             "is %lf to %lf\n", zRange_orig.first, zRange_orig.second);
    }

    // Since our z sampling resolution is 0, set the z value for all samples.
    const double z = zRange_orig.first;

    // Compute the number of transforms along each direction.
    const int num_x_locations = (xRange.second - xRange.first) / xy_sampling_resolution + 1;
    const int num_y_locations = (yRange.second - yRange.first) / xy_sampling_resolution + 1;

    // Reserve space for all of the transforms.
    transforms->reserve(num_x_locations * num_y_locations);

    // In this case, the volume is actually an area since we are only
    // sampling in the x and y dimensions.
    const double volume = pow(xy_sampling_resolution, 2);

    // Create candidate transforms.  We only sample in the horizontal direction.
    for (double x = xRange.first; x <= xRange.second; x += xy_sampling_resolution) {
      for (double y = yRange.first; y <= yRange.second; y += xy_sampling_resolution) {
        XYZTransform transform(x, y, z, volume);
        transforms->push_back(transform);
      }
    }
  } else {
    // Make sure we hit 0 in our z range, in case the sampling resolution
    // is too large.
    std::pair<double, double> zRange;
    if (z_sampling_resolution > fabs(zRange_orig.second - zRange_orig.first)) {
      zRange.first = 0;
      zRange.second = 0;
    } else {
      zRange.first = zRange_orig.first;
      zRange.second = zRange_orig.second;
    }

    // Compute the number of transforms along each direction.
    const int num_x_locations = (xRange.second - xRange.first) / xy_sampling_resolution + 1;
    const int num_y_locations = (yRange.second - yRange.first) / xy_sampling_resolution + 1;
    const int num_z_locations = (zRange.second - zRange.first) / z_sampling_resolution + 1;

    // Reserve space for all of the transforms.
    transforms->reserve(num_x_locations * num_y_locations * num_z_locations);

    const double volume = pow(xy_sampling_resolution, 2) * z_sampling_resolution;

    // Create candidate transforms.
    for (double x = xRange.first; x <= xRange.second; x += xy_sampling_resolution) {
      for (double y = yRange.first; y <= yRange.second; y += xy_sampling_resolution) {
        for (double z = zRange.first; z <= zRange.second; z += z_sampling_resolution) {
          XYZTransform transform(x, y, z, volume);
          transforms->push_back(transform);
        }
      }
    }
  }
}

