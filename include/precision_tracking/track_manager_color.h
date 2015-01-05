/*
 * track_manager_color.h
 *
 *      Author: Alex Teichman
 *
 * I/O functionality for reading/writing a sequence of point clouds to disk.
 *
 */

#ifndef __PRECISION_TRACKING__TRACK_MANAGER_COLOR_H
#define __PRECISION_TRACKING__TRACK_MANAGER_COLOR_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <float.h>

#include <Eigen/Eigen>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace precision_tracking {

namespace track_manager_color {

const int POINTCLOUD_SERIALIZATION_VERSION = 1;
const int TRACK_SERIALIZATION_VERSION = 2;
const int TRACKMANAGER_SERIALIZATION_VERSION = 2;
const int FRAME_SERIALIZATION_VERSION = 0;

  class Frame {
  public:
    int serialization_version_;

    // Points for the object observed in a single frame, in local coordinates.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    // Time that this object was observed.
    double timestamp_;

    Frame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double timestamp);
    Frame(std::istream& istrm);
    void serialize(std::ostream& out) const;
    bool deserialize(std::istream& istrm);

    //! Returns false if there were no points.
    bool operator!=(const Frame& fr);
    bool operator==(const Frame& fr);
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
    
    TrackManagerColor();
    TrackManagerColor(const std::string& filename, const int tracknum);
    TrackManagerColor(const std::string& filename);
    TrackManagerColor(std::istream& istrm);
    TrackManagerColor(const std::vector< boost::shared_ptr<Track> >& tracks);
  };

  bool checkLine(std::istream& istrm, const std::string& expected_input);
  bool readCloud(std::istream& s, pcl::PCLPointCloud2 &cloud);
  void deserializePointCloud(std::istream& istrm,
      pcl::PointCloud<pcl::PointXYZRGB>& point_cloud);
  inline std::ostream& writeCloud(std::ostream& s,
                                  const pcl::PCLPointCloud2& cloud);
  void serializePointCloud(
      const pcl::PointCloud<pcl::PointXYZRGB> cloud, std::ostream& out);
  bool cloudsEqual(const pcl::PointCloud<pcl::PointXYZRGB>& c1,
        const pcl::PointCloud<pcl::PointXYZRGB>& c2);
  double getTrackLength(const Track& tr);
  bool floatEq(float x, float y, int maxUlps = 5);
}

} // namespace precision_tracking
   
#endif //__PRECISION_TRACKING__TRACK_MANAGER_COLOR_H
