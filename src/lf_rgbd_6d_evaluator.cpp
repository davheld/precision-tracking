/*
 * lf_rgbd_6d_evaluator.cpp
 *
 * Created on: August 6, 2014
 *      Author: davheld
 *
 */


#include <utility>
#include <vector>
#include <algorithm>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <boost/math/constants/constants.hpp>

#include <precision_tracking/lf_rgbd_6d_evaluator.h>


using std::max;
using std::min;
using std::pair;
using std::vector;

namespace precision_tracking {

namespace {

// --- Constants ---

const double pi = boost::math::constants::pi<double>();

} // namespace


LF_RGBD_6D_Evaluator::LF_RGBD_6D_Evaluator(const Params *params)
    : AlignmentEvaluator(params),
      searchTree_(false),  //  //By setting sorted to false,
                                // the radiusSearch operations will be faster.
      max_nn_(1),
      nn_indices_(max_nn_),
      nn_sq_dists_(max_nn_),
      use_color_(params->useColor),
      color_exp_factor1_(-1.0 / params_->kValueSigma1),
      color_exp_factor2_(-1.0 / params_->kValueSigma2)
{
}

LF_RGBD_6D_Evaluator::~LF_RGBD_6D_Evaluator()
{
  // TODO Auto-generated destructor stub
}

void LF_RGBD_6D_Evaluator::setPrevPoints(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr prev_points)
{
  AlignmentEvaluator::setPrevPoints(prev_points);

  // Set the input cloud for the search tree to the previous points for NN
  // lookups.
  searchTree_.setInputCloud(prev_points_);

  // Set search tree epsilon for a speedup.
  searchTree_.setEpsilon(params_->kSearchTreeEpsilon);
}

void LF_RGBD_6D_Evaluator::init(const double xy_sampling_resolution,
          const double z_sampling_resolution,
          const double sensor_horizontal_resolution,
          const double sensor_vertical_resolution,
          const size_t num_current_points)
{
  AlignmentEvaluator::init(xy_sampling_resolution, z_sampling_resolution,
                           sensor_horizontal_resolution,
                           sensor_vertical_resolution,
                           num_current_points);

  // Compute the total particle sampling resolution
  const double sampling_resolution = sqrt(pow(xy_sampling_resolution_, 2) +
                                          pow(z_sampling_resolution_, 2));

  // Set the probability of seeing a color match, which is based on the
  // particle sampling resolution - when we are sampling sparsely, we do not
  // expect the colors to align well.
  if (params_->kColorThreshFactor == 0) {
    prob_color_match_ = params_->kProbColorMatch;
  } else {
    prob_color_match_ = params_->kProbColorMatch * exp(-pow(sampling_resolution, 2) /
        (2 * pow(params_->kColorThreshFactor, 2)));
  }
}

void LF_RGBD_6D_Evaluator::score6DTransforms(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& current_points,
    const Eigen::Vector3f& current_points_centroid,
    const double xy_sampling_resolution,
    const double z_sampling_resolution,
    const double sensor_horizontal_resolution,
    const double sensor_vertical_resolution,
    const std::vector<Transform6D>& transforms,
    const MotionModel& motion_model,
    ScoredTransforms<ScoredTransform6D>* scored_transforms)
{
  // Initialize variables for tracking grid.
  const size_t num_current_points = current_points->size();
  init(xy_sampling_resolution, z_sampling_resolution,
       sensor_horizontal_resolution, sensor_vertical_resolution,
       num_current_points);

  const size_t num_transforms = transforms.size();

  // Compute scores for all of the transforms using the density grid.
  scored_transforms->clear();
  scored_transforms->resize(num_transforms);

  for(size_t i = 0; i < num_transforms; ++i){
    const Transform6D& transform = transforms[i];
    const double delta_x = transform.x;
    const double delta_y = transform.y;
    const double delta_z = transform.z;
    const double roll = transform.roll;
    const double pitch = transform.pitch;
    const double yaw = transform.yaw;
    const double volume = transform.volume;

    const double log_prob = getLogProbability(
          current_points, current_points_centroid, motion_model,
          delta_x, delta_y, delta_z, roll, pitch, yaw);

    // Save the complete transform with its log probability.
    const ScoredTransform6D scored_transform(
                delta_x, delta_y, delta_z, roll, pitch, yaw, log_prob, volume);
    scored_transforms->set(scored_transform, i);
  }
}

void LF_RGBD_6D_Evaluator::makeEigenRotation(
    const double roll, const double pitch, const double yaw,
    Eigen::Quaternion<float>* rotation) const
{
  Eigen::AngleAxisf roll_angle(roll, Eigen::Vector3f::UnitZ());
  Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitX());

  *rotation = roll_angle * yaw_angle * pitch_angle;
}

void LF_RGBD_6D_Evaluator::makeEigenTransform(
    const Eigen::Vector3f& centroid,
    const double delta_x, const double delta_y, const double delta_z,
    const double roll, const double pitch, const double yaw,
    Eigen::Affine3f* transform) const
{
  // We want to rotate the object about its own centroid (not about the
  // camera center), so we first subtract off the centroid, then rotate,
  // then add back the centroid.
  const Eigen::Affine3f center_points(Eigen::Translation<float, 3>(-centroid));

  //const Eigen::Matrix4f rotate_points;
  Eigen::Quaternion<float> rotate_points;
  makeEigenRotation(roll, pitch, yaw, &rotate_points);

  const Eigen::Translation<float,3> uncenter_points(centroid);

  // Now shift the points according to the translation.
  const Eigen::Translation<float,3> translate_points(delta_x, delta_y, delta_z);

  // Center, rotate, uncenter, then translate, as described above.
  Eigen::Affine3f transformationMatrix;
  transformationMatrix = translate_points * uncenter_points * rotate_points *
      center_points;

  // Copy from a matrix to a transform.
  *transform = transformationMatrix;
}

double LF_RGBD_6D_Evaluator::getLogProbability(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
    const Eigen::Vector3f& current_points_centroid,
    const MotionModel& motion_model,
    const double x, const double y, const double z)
{
  // For a 3D transform, set the angular rotations to 0.
  const double roll = 0;
  const double pitch = 0;
  const double yaw = 0;

  return getLogProbability(current_points, current_points_centroid, motion_model,
                    x, y, z, roll, pitch, yaw);
}

double LF_RGBD_6D_Evaluator::getLogProbability(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& current_points,
    const Eigen::Vector3f& current_points_centroid,
    const MotionModel& motion_model,
    const double delta_x,
    const double delta_y,
    const double delta_z,
    const double roll,
    const double pitch,
    const double yaw)
{
  // Make a new cloud to store the transformed cloud for the current points.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_current_points(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  // Make the desired transform.
  Eigen::Affine3f transform;
  makeEigenTransform(current_points_centroid, delta_x, delta_y, delta_z, roll,
                     pitch, yaw, &transform);

  // Transform the cloud.
  pcl::transformPointCloud(*current_points, *transformed_current_points,
                           transform);

  // Total log measurement probability.
  double log_measurement_prob = 0;

  // Iterate over every point, and look up its score in the density grid.
  const size_t num_points = current_points->size();
  for (size_t i = 0; i < num_points; ++i) {
    // Extract the point so we can compute its score.
    const pcl::PointXYZRGB& current_pt = (*transformed_current_points)[i];

    // Compute the probability.
    log_measurement_prob += get_log_prob(current_pt);
  }

  // Compute the motion model probability.
  const double motion_model_prob = motion_model.computeScore(delta_x, delta_y,
                                                             delta_z);

  // Combine the motion model score with the (discounted) measurement score to
  // get the final log probability.
  const double log_prob = log(motion_model_prob) +
      measurement_discount_factor_ * log_measurement_prob;

  return log_prob;
}

double LF_RGBD_6D_Evaluator::getPointProbability(
    const pcl::PointXYZRGB& point)
{
  // Get the distance to the tracked object.
  const double horizontal_distance = sqrt(pow(point.x, 2) + pow(point.y, 2));

  // The horizontal resolution for the 64-beam Velodyne spinning at 10 Hz
  // is 0.18 degrees.
  const double velodyne_horizontal_angular_res = 0.18;

  // There are 64 beams spanning 26.8 vertical degrees, so the average spacing
  // between each beam is computed as follows.
  const double velodyne_vertical_angular_res = 26.8 / 63;

  // We convert the angular resolution to meters for a given range.
  const double sensor_horizontal_resolution =
      2 * horizontal_distance *
      tan(velodyne_horizontal_angular_res / 2.0 * pi / 180.0);
  const double sensor_vertical_resolution =
      2 * horizontal_distance *
      tan(velodyne_vertical_angular_res / 2.0 * pi / 180.0);

  // Initialize variables for tracking grid.
  const size_t num_current_points = 1;
  const double xy_sampling_resolution = 0;
  const double z_sampling_resolution = 0;
  init(xy_sampling_resolution, z_sampling_resolution,
       sensor_horizontal_resolution, sensor_vertical_resolution,
       num_current_points);

  get_log_prob(point);
}

double LF_RGBD_6D_Evaluator::get_log_prob(const pcl::PointXYZRGB& current_pt)
{
  // Find the nearest neighbor.
  searchTree_.nearestKSearch(current_pt, max_nn_, nn_indices_, nn_sq_dists_);
  const pcl::PointXYZRGB& prev_pt = (*prev_points_)[nn_indices_[0]];

  // Compute the log probability of this neighbor match.
  // The NN search is isotropic, but our measurement model is not!
  // To acccount for this, we weight the NN search only by the isotropic
  // xyz_exp_factor_.
  const double log_point_match_prob_i = nn_sq_dists_[0] * xyz_exp_factor_;

  // Compute the probability of this neighbor match.
  const double point_match_prob_spatial_i = exp(log_point_match_prob_i);

  // Compute the point match probability, incorporating color if necessary.
  double point_prob;
  if (use_color_) {
    point_prob = computeColorProb(prev_pt, current_pt,
                                  point_match_prob_spatial_i);
  } else {
    point_prob = point_match_prob_spatial_i + smoothing_factor_;
  }

  // Compute the log.
  const double log_point_prob = log(point_prob);

  return log_point_prob;
}

double LF_RGBD_6D_Evaluator::computeColorProb(const pcl::PointXYZRGB& prev_pt,
    const pcl::PointXYZRGB& pt, const double point_match_prob_spatial_i) const
{
  // Because we are using color, we have to modify the smoothing factor.
  const double factor1 = smoothing_factor_ / (smoothing_factor_ + 1);

  double smoothing_factor;
  if (use_color_ && !params_->kTwoColors) {
    smoothing_factor = factor1 / 255;
  } else if (use_color_ && params_->kTwoColors) {
    smoothing_factor = factor1 / pow(255, 2);
  } else {
    smoothing_factor = smoothing_factor_;
  }
  smoothing_factor *= (1 - point_match_prob_spatial_i);

  // Find the colors of the 2 points.
  int color1 = 0, color2 = 0, color3 = 0, color4 = 0;
  if (params_->kColorSpace == 0) {
    // Blue and Green.
    color1 = pt.b;
    color2 = prev_pt.b;

    color3 = pt.g;
    color4 = prev_pt.g;
  } else if (params_->kColorSpace == 1) {
    // Mean of RGB.
    color1 = (pt.r + pt.g + pt.b) / 3;
    color2 = (prev_pt.r + prev_pt.g + prev_pt.b) / 3;
  } else {
    printf("Unknown color space: %d\n", params_->kColorSpace);
    exit(1);
  }

  // Compute the probability of the match, using the spatial and color distance.
  const double color_distance1 = fabs(color1 - color2);
  double point_match_prob;
  if (params_->kTwoColors) {
    const double color_distance2 = fabs(color3 - color4);

    point_match_prob =
        point_match_prob_spatial_i *
        ((1-prob_color_match_) * 1.0 / pow(255, 2) +
         prob_color_match_ * (
           -0.5 * color_exp_factor1_ * exp(color_distance1 * color_exp_factor1_)) *
           -0.5 * color_exp_factor2_ * exp(color_distance2 * color_exp_factor2_));
  } else {
    point_match_prob =
        point_match_prob_spatial_i *
        ((1-prob_color_match_) * 1.0 / 255 +
         prob_color_match_ *
         -1 * color_exp_factor1_ * exp(color_distance1 * color_exp_factor1_));
  }
  const double point_prob = point_match_prob + smoothing_factor;

  return point_prob;
}

} // namespace precision_tracking
