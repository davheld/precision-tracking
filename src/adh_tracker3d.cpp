/*
 * adh_tracker_3d.cpp
 *
 *  Created on: Sep 1, 2013
 *      Author: davheld
 *
 */


#include <vector>
#include <algorithm>

#include <precision_tracking/adh_tracker3d.h>

using std::vector;
using std::max;

namespace precision_tracking {


ADHTracker3d::ADHTracker3d(const Params *params)
  : params_(params)
{
}

ADHTracker3d::~ADHTracker3d()
{
}

void ADHTracker3d::track(
    const double initial_xy_sampling_resolution,
    const double initial_z_sampling_resolution,
    const std::pair <double, double>& xRange,
    const std::pair <double, double>& yRange,
    const std::pair <double, double>& zRange,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& prev_points,
    const Eigen::Vector3f &current_points_centroid,
    const MotionModel& motion_model,
    const double xy_sensor_resolution,
    const double z_sensor_resolution,
    boost::shared_ptr<AlignmentEvaluator> alignment_evaluator,
    ScoredTransforms<ScoredTransformXYZ>* final_scored_transforms3D) const
{
  // Compute the minimum sampling resolution based on the sensor
  // resolution - we are limited in accuracy by the sensor resolution,
  // so there is no point in sampling at a much finer scale.
  const double min_xy_sampling_resolution =
      max(xy_sensor_resolution / params_->kMinResFactor, params_->kDesiredSamplingResolution);

  // Initialize the sampling resolution.
  double current_xy_sampling_resolution = initial_xy_sampling_resolution;
  double current_z_sampling_resolution = initial_z_sampling_resolution;

  // Create initial candidate transforms.
  vector<XYZTransform> candidate_transforms;
  createCandidateXYZTransforms(
        current_xy_sampling_resolution, current_z_sampling_resolution,
        xRange, yRange, zRange, &candidate_transforms);

  // Initially track at a coarse resolution and get the probability of
  // various transforms.
  alignment_evaluator->setPrevPoints(prev_points);

  // Total probability for the region that we are evaluating.
  double region_prob = 1;

  while(candidate_transforms.size() > 0) {
    // Compute the probability of each of the candidate transforms.
    ScoredTransforms<ScoredTransformXYZ> scored_transforms3D;
    alignment_evaluator->score3DTransforms(
          current_points, current_points_centroid,
          current_xy_sampling_resolution, current_z_sampling_resolution,
          xy_sensor_resolution, z_sensor_resolution,
          candidate_transforms, motion_model, &scored_transforms3D);

    // Normalize the probabilities so they sum to 1.
    recomputeProbs(region_prob, &scored_transforms3D);

    // Save the output to the final scored transforms.
    final_scored_transforms3D->appendScoredTransforms(scored_transforms3D);

    // If we are below the minimum sampling resolution, we are done.
    if (current_xy_sampling_resolution <= min_xy_sampling_resolution) {
      break;
    }

    // Next we want to sample more finely, so reduce the sampling resolution.
    const double new_xy_sampling_resolution =
        current_xy_sampling_resolution / params_->kReductionFactor;
    const double new_z_sampling_resolution =
        current_z_sampling_resolution / params_->kReductionFactor;

    // Make candidate transforms at the new sampling resolution.
    makeNewTransforms3D(
          new_xy_sampling_resolution, new_z_sampling_resolution,
          current_xy_sampling_resolution, current_z_sampling_resolution,
          final_scored_transforms3D, &candidate_transforms,
          &region_prob);

    current_xy_sampling_resolution = new_xy_sampling_resolution;
    current_z_sampling_resolution = new_z_sampling_resolution;
    }
}

void ADHTracker3d::recomputeProbs(
    const double prior_region_prob,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms) const
{
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
    const double new_log_prob = log(prior_region_prob * conditional_prob);
    scored_transforms_vect[i].setUnnormalizedLogProb(new_log_prob);
  }
}

void ADHTracker3d::makeNewTransforms3D(
    const double xy_sampling_resolution, const double z_sampling_resolution,
    const double old_xy_sampling_resolution,
    const double old_z_sampling_resolution,
    ScoredTransforms<ScoredTransformXYZ>* scored_transforms,
    std::vector<XYZTransform>* new_xyz_transforms,
    double* total_recomputing_prob) const
{
  // If we are only using the top k transforms, we need to sort them.
  if (params_->kMaxNumTransforms > 0) {
    scored_transforms->sortDescending();
  }

  std::vector<ScoredTransformXYZ>& scored_transforms_xyz =
      scored_transforms->getScoredTransforms();

  // Any transforms that we are recomputing at a higher resolution should
  // be removed from the list of previously scored transforms.
  vector<size_t> to_remove;

  // Compute the sampling volume of each transform.
  const double volume = z_sampling_resolution > 0 ?
        pow(xy_sampling_resolution, 2) * z_sampling_resolution :
        pow(xy_sampling_resolution, 2);

  // Keep track of the total probability of the region that we are recomputing
  // the probability of at a higher resolution.
  *total_recomputing_prob = 0;

  const std::vector<double>& probs = scored_transforms->getNormalizedProbs();

  // Allocate space for the new transforms that we will recompute.
  const size_t max_num_transforms = params_->kMaxNumTransforms > 0 ?
        std::min(probs.size(), params_->kMaxNumTransforms) : probs.size();
  new_xyz_transforms->clear();
  new_xyz_transforms->reserve(max_num_transforms);

  // For each region with probability greater than the minimum
  // threshold, sample more finely in that region.
  for (size_t i = 0; i < max_num_transforms; ++i) {
    const ScoredTransformXYZ& old_scored_transform = scored_transforms_xyz[i];
    const double old_x = old_scored_transform.getX();
    const double old_y = old_scored_transform.getY();
    const double old_z = old_scored_transform.getZ();

    // Only subdivide cells whose probabilities are greater than the minimum
    // threshold.
    if (probs[i] > params_->kMinProb) {
      *total_recomputing_prob += probs[i];

      // We are sampling more finely in this region, so we can remove
      // the previously computed probability for this transform.
      to_remove.push_back(i);

      // Get the initial sampling point in this region.
      const double min_x = old_x - old_xy_sampling_resolution / 2 + xy_sampling_resolution / 2;
      const double min_y = old_y - old_xy_sampling_resolution / 2 + xy_sampling_resolution / 2;
      const double min_z = old_z - old_z_sampling_resolution / 2 + z_sampling_resolution / 2;

      // Sample more finely in this region.
      for (int i = 0; i < params_->kReductionFactor; ++i) {
        const double new_x = min_x + xy_sampling_resolution * i;

        for (int j = 0; j < params_->kReductionFactor; ++j) {
          const double new_y = min_y + xy_sampling_resolution * j;

          if (z_sampling_resolution == 0) {
            const double new_z = old_z;

            XYZTransform new_transform(new_x, new_y, new_z, volume);
            new_xyz_transforms->push_back(new_transform);
          } else {
            for (int k = 0; k < params_->kReductionFactor; ++k) {
              const double new_z = min_z + z_sampling_resolution * k;

              XYZTransform new_transform(new_x, new_y, new_z, volume);
              new_xyz_transforms->push_back(new_transform);
            }
          }
        }
      }
    }
  }

  // Remove regions that we are resampling at a higher resolution.
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
    std::vector<XYZTransform>* transforms) const
{
  if (xy_sampling_resolution == 0) {
    printf("Error - xy sampling resolution must be > 0");
    exit(1);
  }

  if (z_sampling_resolution == 0) {
    if (zRange_orig.first != zRange_orig.second) {
      printf("Error - z_sampling_resolution = 0 but the z range "
             "is %lf to %lf.  You need to set kInitialZSamplingResolution > 0\n",
             zRange_orig.first, zRange_orig.second);
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

} // namespace precision_tracking
