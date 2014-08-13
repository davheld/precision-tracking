/*
 * tracker.cpp
 *
 *  Created on: Jul 30, 2014
 *      Author: davheld
 */

#include <string>
#include <cstdio>

#include "track_manager_color.h"
#include "model_builder.h"
#include "high_res_timer.h"

using std::string;

struct TrackResults {
  int track_num;
  std::vector<Eigen::Vector3f> estimated_velocities;
  std::vector<bool> ignore_frame;
};

void getGTVelocities(const string& gt_folder, const int track_num,
                     std::vector<double>* gt_velocities) {
  string filename = gt_folder + "/track%dgt.txt";

  std::ostringstream filename_stream;
  filename_stream << gt_folder << "/track" << track_num << "gt.txt";
  filename = filename_stream.str();

  //printf("Loading file: %s\n", filename.c_str());

  FILE* fid = fopen(filename.c_str(), "r");

  if (fid == NULL) {
    printf("Cannot open file: %s\n", filename.c_str());
    exit(1);
  }

  double velocity;
  while(fscanf(fid, "%lf\n", &velocity) > 0) {
    gt_velocities->push_back(velocity);
  }

  fclose(fid);
}

void computeErrorStatistics(const std::vector<double>& errors) {
  double sum_sq = 0;

  size_t num_frames = errors.size();

  //printf("Computing error statistics over %zu frames\n", num_frames);

  for (size_t i = 0; i < num_frames; ++i) {
    sum_sq += pow(errors[i], 2);
  }

  const double rms_error = sqrt(sum_sq / errors.size());

  //printf("RMS error: %lf\n", rms_error);
  printf("%lf\n", rms_error);
}

void evaluateTracking(const std::vector<TrackResults>& velocity_estimates,
                      const string& gt_folder) {

  std::vector<double> errors;

  // Iterate over all tracks.
  for (size_t i = 0; i < velocity_estimates.size(); ++i) {
    TrackResults track_results = velocity_estimates[i];

    int track_num = track_results.track_num;

    std::vector<double> gt_velocities;
    getGTVelocities(gt_folder, track_num, &gt_velocities);

    if (track_results.estimated_velocities.size() != gt_velocities.size()) {
      //printf("Error: estimated velocities != gt_velocities; %zu != %zu\n",
      //       track_results.estimated_velocities.size(), gt_velocities.size());
    }

    int skipped = 0;

    std::vector<double> track_errors;

    for (size_t j = 0; j < track_results.estimated_velocities.size(); ++j) {
      if (track_results.ignore_frame[j]) {
        //printf("Skipping frame: %zu\n", j);
        skipped++;
        continue;
      }

      //printf("Not skipping frame: %zu\n", j);

      const Eigen::Vector3f& estimated_velocity =
          track_results.estimated_velocities[j];

      const double estimated_velocity_magnitude = estimated_velocity.norm();
      const double gt_velocity_magnitude = gt_velocities[j-skipped];

      //printf("%zu: Vel: %lf, gt: %lf\n", j, estimated_velocity_magnitude,
      //       gt_velocity_magnitude);

      errors.push_back(estimated_velocity_magnitude - gt_velocity_magnitude);

      track_errors.push_back(estimated_velocity_magnitude - gt_velocity_magnitude);
    }
    //printf("Errors for track: %d: ", track_num);
    //computeErrorStatistics(track_errors);
  }

  printf("Overall stats:\n");

  computeErrorStatistics(errors);
}

int main(int argc, char **argv)
{
  if (argc < 3) {
    printf("Usage: %s tm_file gt_folder\n", argv[0]);
    return (1);
  }

  string color_tm_file = argv[1];
  string gt_folder = argv[2];

  std::vector<TrackResults> velocity_estimates;

  // Load tracks.
  printf("Loading file: %s\n", color_tm_file.c_str());
  track_manager_color::TrackManagerColor track_manager(color_tm_file);
  const std::vector< boost::shared_ptr<track_manager_color::Track> >& tracks =
      track_manager.tracks_;

  // Iterate over all tracks.
  printf("Found %zu tracks\n", tracks.size());

  // Structure for tracking.
  ModelBuilder aligner(false);

  printf("Tracking objects - please wait...\n");

  HighResTimer hrt("Total time for tracking");
  hrt.start();

  int total_num_frames = 0;

  for (size_t i = 0; i < tracks.size(); ++i) {
    // Reset the model builder for this new track.
    aligner.clear();

    // Extract frames.
    const boost::shared_ptr<track_manager_color::Track>& track = tracks[i];
    std::vector< boost::shared_ptr<track_manager_color::Frame> > frames =
        track->frames_;

    // Structure for storing track output.
    TrackResults track_estimates;
    track_estimates.track_num = track->track_num_;


    // Iterate over all frames for this track.
    /*if ((i+1) % 10 == 0){
      printf("Processing track %zu / %zu, tracknum %d with %zu frames\n", i+1, tracks.size(),
            track->track_num_, frames.size());
    }*/

    bool skip_next = false;
    double prev_angle = 0;
    double prev_time = 0;

    //printf("Iterating over %zu frames\n", frames.size());
    for (size_t j = 0; j < frames.size(); ++j) {
      boost::shared_ptr<track_manager_color::Frame> frame = frames[j];

      const Eigen::Vector3f& centroid = frame->getCentroid();
      const double angle = atan2(centroid(1), centroid(0));
      const double angle_diff = fabs(angle - prev_angle);

      const double curr_time = frame->timestamp_;
      const double time_diff = curr_time - prev_time;

      prev_angle = angle;
      //printf("%zu: Angle: %lf, angle diff: %lf\n", j, angle, angle_diff);

      if (j > 0) {
        if (angle_diff <= 1 || j == 0) {
          if (!skip_next) {
            if (time_diff >= 0.05) {
              track_estimates.ignore_frame.push_back(false);
            } else {
              track_estimates.ignore_frame.push_back(true);
            }
          } else {
            track_estimates.ignore_frame.push_back(true);
          }
          skip_next = false;
        } else {
          track_estimates.ignore_frame.push_back(true);
            skip_next = true;
            if (j > 1) {
              track_estimates.ignore_frame[j-2] = true;
            }
        }
      }

      // Track object.
      Eigen::Vector3f estimated_velocity;
      aligner.addPoints(frame->cloud_, frame->timestamp_, frame->getCentroid(),
                         &estimated_velocity);

      // The first time we don't have a velocity yet.
      if (j > 0) {
        total_num_frames++;
        track_estimates.estimated_velocities.push_back(estimated_velocity);
      }
    }

    velocity_estimates.push_back(track_estimates);
  }


  hrt.stop();
  hrt.print();
  const double ms = hrt.getMilliseconds();
  printf("Mean runtime per frame: %lf ms\n", ms / total_num_frames);

  evaluateTracking(velocity_estimates, gt_folder);

  return 0;
}

