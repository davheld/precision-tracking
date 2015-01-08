/*
 * params.h
 *
 *      Author: Brice Rebsamen
 *
 * Data structure to keep track of parameters for the tracker.
 *
 */

#ifndef __PRECISION_TRACKING__PARAMS__H__
#define __PRECISION_TRACKING__PARAMS__H__


#include <cstddef>

namespace precision_tracking
{

/// Parameters for most of the tunable aspects of the precision tracker.
struct Params
{
  /// @{ Tracker section

  /// Whether to return the mean or mode of the distribution.  The mean
  /// typically is more accurate because it accounts for the uncertainty
  /// of the distribution, and because it can be computed at a finer resolution
  /// than our sampling resolution.
  bool useMean;

  /// @}


  /// @{ ADH tracker section

  /// We compute the minimum sampling resolution based on the sensor
  /// resolution - we are limited in accuracy by the sensor resolution,
  /// so there is no point in sampling at a much finer scale.
  /// The minimum sampling resolution is set to be no smaller than
  /// sensor_resolution / kMinResFactor.
  double kMinResFactor;

  /// The desired sampling resolution.
  double kDesiredSamplingResolution;

  /// How much to reduce the sampling resolution each iteration.
  double kReductionFactor;

  /// Set this to limit the maximum number of transforms that we
  /// evaluate at each iteration beyond the first.
  size_t kMaxNumTransforms;

  /// Only divide cells whose probabilities are greater than kMinProb.
  double kMinProb;

  /// @}


  /// @{ Alignment evaluator section

  /// Factor to multiply the sensor resolution for our measurement model.
  /// We model each point as a Gaussian: exp(-x^2 / 2 sigma^2)
  /// With sigma^2 = (sensor_resolution * kSigmaFactor)^2 + other terms.
  double kSigmaFactor;

  /// Factor to multiply the particle sampling resolution for our measurement
  /// model. We model each point as a Gaussian: exp(-x^2 / 2 sigma^2)
  /// With sigma^2 = (sampling_resolution * kSigmaGridFactor)^2 + other terms.
  double kSigmaGridFactor;

  /// The noise in our sensor which is independent of the distance to the tracked
  /// object. We model each point as a Gaussian: exp(-x^2 / 2 sigma^2)
  /// With sigma^2 = kMinMeasurementVariance^2 + other terms.
  double kMinMeasurementVariance;

  /// We add this to our Gaussian so we don't give 0 probability to points
  /// that don't align.
  /// We model each point as a Gaussian: exp(-x^2 / 2 sigma^2) + kSmoothingFactor
  double kSmoothingFactor;

  /// We multiply our log measurement probability by this factor, to decrease
  /// our confidence in the measurement model (e.g. to take into account
  /// dependencies between neighboring points).
  double kMeasurementDiscountFactor;

  /// We assume that there are this many independent points per object.  Beyond
  /// this many, we discount the measurement model accordingly.
  double kMaxDiscountPoints;

  /// @}


  /// @{ Density grid evaluator section

  /// How far to spill over in the density grid (number of sigmas).
  double kSpilloverRadius;

  /// @{ size of the grid in number of cells
  int kMaxXSize;
  int kMaxYSize;
  int kMaxZSize;
  /// @}

  /// @}


  /// @{ down sampler section

  /// Whether to round up or down for deterministic downsampling when computing
  /// how many points to skip.
  bool kUseCeil;

  /// @}


  /// @{ lg rgbd 6d evaluator section

  /// Approximation factor for finding the nearest neighbor.
  /// Set to 0 to find the exact nearest neighbor.
  /// For a reasonable speedup, set to 2.
  double kSearchTreeEpsilon;

  /// Whether to use two colors in our measurement model.
  bool kTwoColors;

  /// The parameter to use for our color Laplacian for color 1.
  double kValueSigma1;

  /// The parameter to use for our color Laplacian for color 2.
  double kValueSigma2;

  /// How much we expect the colors to match (there might have been lens flare,
  /// the lighting might have changed, etc. which would cause all the colors
  /// to be completely wrong).
  double kProbColorMatch;

  /// How much to care about color as a function of the particle sampling resolution.
  /// When we are sampling sparsely, we do not expect the colors to align well.
  /// Set to 0 to ignore this term.
  /// If non-zero, we set prob_color_match_ = kProbColorMatch *
  ///    exp(-pow(sampling_resolution, 2) / (2 * pow(kColorThreshFactor, 2));
  double kColorThreshFactor;

  /// Which color space to use for our color matches.
  /// 0: Use blue and green,
  /// 1: Use (R + G + B) / 3.
  int kColorSpace;

  /// @}


  /// @{ Motion model section

  /// @{ How much noise to add to the velocity covariance.
  double kPropagationVarianceXY;
  double kPropagationVarianceZ;
  /// @}

  /// The measurement noise for a centroid-based Kalman filter.
  double kCentroidMeasurementNoise;

  /// The initial velocity variance for a centroid-based Kalman filter.
  double kCentroidInitVelocityVariance;

  /// Minimum probability returned by the motion model.
  double kMotionMinProb;

  /// @}


  /// @{ Precision tracker section

  /// Whether to use color - note that using color will make the tracker
  /// very slow!
  bool useColor;

  // Whether to track the full 3D point cloud or a 2D projection.  Tracking with
  // the full 3D point cloud is more accurate but uses much more memory,
  // due to our caching scheme.
  bool use3D;

  /// We downsample the current frame of the tracked object to have this many
  /// points.
  int kCurrFrameDownsample;

  /// We downsample the previous frame of the tracked object to have this many
  /// points.
  int kPrevFrameDownsample;

  /// Whether to deterministically or stochastically downsample points from
  /// the tracked object.
  bool stochastic_downsample;

  /// max z value of velocities to estimate.
  double maxZ;

  /// Start with a very coarse xy sampling for efficiency.
  double kInitialXYSamplingResolution;

  /// Do not sample in the z-direction - assume minimal vertical motion.
  double kInitialZSamplingResolution;

  /// @}



  /// Defaults constructor assigns default values to each parameter
  Params()
  {
    // Tracker section
    useMean = true;

    // ADH tracker section
    kMinResFactor = 1;
    kDesiredSamplingResolution = 0.05;
    kReductionFactor = 3;
    kMaxNumTransforms = 0;
    kMinProb = 0.0001;

    // Alignment evaluator section
    kSigmaFactor = 0.5;
    kSigmaGridFactor = 1;
    kMinMeasurementVariance = 0.03;
    kSmoothingFactor = 0.8;
    kMeasurementDiscountFactor = 1;
    kMaxDiscountPoints = 150.0;

    // Density grid evaluator section
    kSpilloverRadius = 2.0;
    kMaxXSize = 1000; // At a resolution of 3.7 cm, a 10 m wide object will take 270 cells
    kMaxYSize = 1000;
    kMaxZSize = 250;  // At a resolution of 3.7 cm, a 5 m tall object will take 135 cells.

    // down sampler section
    kUseCeil = true;

    // lg rgbd 6d evaluator section
    kSearchTreeEpsilon = 2;
    kTwoColors = false;
    kValueSigma1 = 13.9;
    kValueSigma2 = 15.2;
    kProbColorMatch = 0.05;
    kColorThreshFactor = 1;
    kColorSpace = 0;

    // Motion model section
    kPropagationVarianceXY = 10;
    kPropagationVarianceZ = 10;
    kCentroidMeasurementNoise = 0.4;
    kCentroidInitVelocityVariance = 5;
    kMotionMinProb = 1e-4;

    // Precision tracker section
    useColor = false;
    use3D = false;
    kCurrFrameDownsample = 150;
    kPrevFrameDownsample = 2000;
    stochastic_downsample = false;
    maxZ = 0;   // Set to 0 so we do not search over z (for objects moving
                // in urban settings, the vertical motion is small).
    kInitialXYSamplingResolution = 1;
    kInitialZSamplingResolution = 0;
  }
};

} // namespace precision_tracking

#endif // __PRECISION_TRACKING__PARAMS__H__
