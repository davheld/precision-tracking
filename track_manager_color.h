#ifndef TRACK_MANAGER_COLOR_H
#define TRACK_MANAGER_COLOR_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <float.h>
#include <Eigen/Eigen>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#define POINTCLOUD_SERIALIZATION_VERSION 0
#define TRACK_SERIALIZATION_VERSION 2
#define TRACKMANAGER_SERIALIZATION_VERSION 2
#define FRAME_SERIALIZATION_VERSION 0

namespace track_manager_color {

  class Frame {
  public:
    int serialization_version_;

    // Points for the object observed in a single frame.
    //! Assumed to be stored in local coordinates!
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    // Time that this cloud was observed.
    double timestamp_;

    Frame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double timestamp);
    Frame(std::istream& istrm);
    void serialize(std::ostream& out) const;
    bool deserialize(std::istream& istrm);

    //! Returns false if there were no points.
    bool operator!=(const Frame& fr);
    bool operator==(const Frame& fr);
    void getVelodyneXYZ(double* x, double* y, double* z) const;
    Eigen::Vector3f getCentroid();
    Eigen::MatrixXf getBoundingBox();
    double getDistance();

  private:
    //! For caching of getCentroid() call.
    boost::shared_ptr<Eigen::Vector3f> centroid_;
    //! For caching of getBoundingBox() call.
    //! bounding_box_.col(0) are the small x and y coords; .col(1) are the large.
    boost::shared_ptr<Eigen::MatrixXf> bounding_box_;
  };
 
  class Track {
  public:
    int serialization_version_;
    int track_num_;
    std::string label_;
    std::vector< boost::shared_ptr<Frame> > frames_;
    
    //! Initializes with label == "unknown", and that's it.
    Track();
    Track(std::istream& istrm);
    Track(const std::string& label, const std::vector< boost::shared_ptr<Frame> >& frames);

    //! Reserves space in the vectors of velo centers, timestamps, and clouds.
    void reserve(size_t num);
    void insertFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
             double timestamp);
    bool operator==(const Track& tr);
    bool operator!=(const Track& tr);
    void serialize(std::ostream& out) const;
    bool deserialize(std::istream& istrm);
    //! Returns false if timestamp < timestamps_.front() || timestamp > timestamps_.back(), otherwise fills *idx with the index of the cloud with the nearest timestamp.
    bool seek(double timestamp, double max_time_difference, size_t* idx);
    //! Like seek, returns false if timestamp < timestamps_.front() || timestamp > timestamps_.back().  Otherwise, fills *idx with the index of the cloud with nearest timestamp less than timestamp, and fills *interpolation with a value in [0, 1] indicating how much to weight cloud *idx+1 vs cloud *idx.
    bool interpolatedSeek(double timestamp, double max_time_difference, size_t* idx, double* interpolation);
    double getMeanNumPoints() const;
    double getMeanDistance();
  };

  class TrackManagerColor {
  public:
    int serialization_version_;
    std::vector< boost::shared_ptr<Track> > tracks_;

    //! Returns the maximum number of clouds in any track.
    size_t getMaxNumClouds() const;
    size_t getNumClouds() const;
    size_t getNumLabeledClouds() const;
    bool operator==(const TrackManagerColor& tm);
    bool operator!=(const TrackManagerColor& tm);
    bool save(const std::string& filename);
    void serialize(std::ostream& out);
    bool deserialize(std::istream& istrm);
    bool deserialize(std::istream& istrm, const int tracknum);
    //! Put the tracks in descending order of track length.
    void sortTracks();
    //! Sort based on some other criteria.  Descending.
    void sortTracks(double (*rateTrack)(const Track&));
    void sortTracks(const std::vector<double>& track_ratings);
    void insertTrack(boost::shared_ptr<Track> track);
    void reserve(size_t size);
    void getFramesNear(double timestamp, double tol,
		       std::vector< boost::shared_ptr<Frame> >* frames,
		       std::vector<std::string>* class_names,
		       std::vector<int>* track_ids) const;
    
    TrackManagerColor();
    TrackManagerColor(const std::string& filename, const int tracknum);
    TrackManagerColor(const std::string& filename);
    TrackManagerColor(std::istream& istrm);
    TrackManagerColor(const std::vector< boost::shared_ptr<Track> >& tracks);
  };

  bool checkLine(std::istream& istrm, const std::string& expected_input);
  void  deserializePointCloud(std::istream& istrm,
      pcl::PointCloud<pcl::PointXYZRGB>& point_cloud);
  void serializePointCloud(
      const pcl::PointCloud<pcl::PointXYZRGB> point_cloud, std::ostream& out);
  bool cloudsEqual(const pcl::PointCloud<pcl::PointXYZRGB>& c1,
      const pcl::PointCloud<pcl::PointXYZRGB>& c2);
  bool streamTrack(std::string track_manager_filename, const Track& tr); 
  double getTrackLength(const Track& tr);
  bool floatEq(float x, float y, int maxUlps = 5);
}


   
#endif //TRACK_MANAGER_COLOR_H
