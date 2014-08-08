#include "track_manager_color.h"

#include <pcl/conversions.h>
#include <pcl/io/file_io.h>

#include <vector>

//#include "helper.h"

namespace{
  //const char* logger = ROSCONSOLE_DEFAULT_NAME ".track_viewer";
}

//using namespace sensor_msgs;
using namespace std;
using namespace Eigen;
using namespace pcl;

/************************************************************/
/******************** TrackManager ********************/
/************************************************************/
namespace track_manager_color {

TrackManagerColor::TrackManagerColor() :
  serialization_version_(TRACKMANAGER_SERIALIZATION_VERSION),
  tracks_(vector< boost::shared_ptr<Track> >())
{
}

/*TrackManager::TrackManager(std::istream& istrm)  :
  serialization_version_(TRACKMANAGER_SERIALIZATION_VERSION),
  tracks_(vector< shared_ptr<Track> >())
{
  bool success = deserialize(istrm);
  if(!success)
    throw 1;
}*/

TrackManagerColor::TrackManagerColor(const std::vector< boost::shared_ptr<Track> >& tracks)  :
  serialization_version_(TRACKMANAGER_SERIALIZATION_VERSION),
  tracks_(tracks)
{
}

TrackManagerColor::TrackManagerColor(const std::string& filename, const int tracknum) :
	serialization_version_(TRACKMANAGER_SERIALIZATION_VERSION),
	tracks_(vector< boost::shared_ptr<Track> >())
{
	ifstream ifs(filename.c_str(), ios::in);

	if (ifs.fail()){
		printf("Error - Could not open file: %s\n", filename.c_str());
		throw 1;
	}

	printf("Deserializing tracks from file: %s\n", filename.c_str());
	bool success = deserialize(ifs, tracknum);
	printf("Finished deserializing tracks\n");
	if(!success)
		throw 1;
}

TrackManagerColor::TrackManagerColor(const string& filename) :
  serialization_version_(TRACKMANAGER_SERIALIZATION_VERSION),
  tracks_(vector< boost::shared_ptr<Track> >())
{
  ifstream ifs(filename.c_str(), ios::in);

  printf("Deserializing tracks from file: %s\n", filename.c_str());
  bool success = deserialize(ifs);
  printf("Finished deserializing tracks\n");
  if(!success)
    throw 1;
}

bool TrackManagerColor::save(const string& filename) {
  ofstream ofs(filename.c_str(), ios::out);
  serialize(ofs);
  ofs.close();
  return true;
}

size_t TrackManagerColor::getMaxNumClouds() const {
  size_t max_num_clouds = 0;
  for(size_t i = 0; i < tracks_.size(); ++i) {
    if(tracks_[i]->frames_.size() > max_num_clouds)
      max_num_clouds = tracks_[i]->frames_.size();
  }
  return max_num_clouds;
}

  
size_t TrackManagerColor::getNumClouds() const {
  size_t num = 0;
  for(size_t i = 0; i < tracks_.size(); ++i)
    num += tracks_[i]->frames_.size();
  return num;
}

size_t TrackManagerColor::getNumLabeledClouds() const {
  size_t num = 0;
  for(size_t i = 0; i < tracks_.size(); ++i) {
    if(tracks_[i]->label_.compare("unlabeled") != 0)
      num += tracks_[i]->frames_.size();
  }
  return num;
}

void TrackManagerColor::serialize(ostream& out) {
  out << "TrackManager" << endl;
  out << "serialization_version_" << endl;
  out << serialization_version_ << endl;
//   out << "num_tracks" << endl;
//   out << tracks_.size() << endl;
  for(size_t i=0; i<tracks_.size(); ++i) {
    printf("Serializing track %zu\n", i);
    tracks_[i]->serialize(out);
  }
}

bool TrackManagerColor::deserialize(istream& istrm, const int tracknum) {
  tracks_.clear();
  string line;

  getline(istrm, line);
  if(line.compare("TrackManager") != 0) {
    return false;
  }

  getline(istrm, line);
  if(line.compare("serialization_version_") != 0)
    return false;

  istrm >> serialization_version_;
  if(serialization_version_ != TRACKMANAGER_SERIALIZATION_VERSION) {
    cerr << "Expected TrackManager serialization_version_ == " << TRACKMANAGER_SERIALIZATION_VERSION;
    cerr << ".  This file is vs " << serialization_version_ << ", aborting." << endl;
    return false;
  }
  getline(istrm, line);

  int i = 0;
  while(true) {
    boost::shared_ptr<Track> tr(new Track());
    if(tr->deserialize(istrm)){
    	if (i == tracknum){
        tracks_.push_back(tr);
        //once we find the target tracknum, exit
        break;
    	}
    i++;
    } else {
      break;
    }
  }

  return true;
}


bool TrackManagerColor::deserialize(istream& istrm) {
  tracks_.clear();
  string line;

  getline(istrm, line);
  if(line.compare("TrackManager") != 0) {
    return false;
  }

  getline(istrm, line);
  if(line.compare("serialization_version_") != 0)
    return false;

  istrm >> serialization_version_;
  if(serialization_version_ != TRACKMANAGER_SERIALIZATION_VERSION) {
    cerr << "Expected TrackManager serialization_version_ == " << TRACKMANAGER_SERIALIZATION_VERSION;
    cerr << ".  This file is vs " << serialization_version_ << ", aborting." << endl;
    return false;
  }
  getline(istrm, line);
   
  int i = 0;
  while(true) {
    boost::shared_ptr<Track> tr(new Track());
    if(tr->deserialize(istrm)){
      //if (i % 1000 == 0){
    		printf("Deserializing track #%d\n", i);
      //}
    	i++;
      tracks_.push_back(tr);
    } else
      break;
  }
  
  return true;
}

bool TrackManagerColor::operator!=(const TrackManagerColor& tm) {
  return !operator==(tm);
}

bool TrackManagerColor::operator==(const TrackManagerColor& tm) {
  if(serialization_version_ != tm.serialization_version_)
    return false;
  if(tracks_.size() != tm.tracks_.size())
    return false;
  for(size_t i=0; i<tracks_.size(); ++i) {
    if(*tracks_[i] != *tm.tracks_[i])
      return false;
  }
  return true;
}

double getTrackLength(const Track& tr) {
	//double size = tr.frames_[0]->cloud_->width * tr.frames_[0]->cloud_->height;
  double z = tr.getMeanNumPoints() + tr.frames_[0]->cloud_->points.size();
  return (double) tr.frames_.size() + 1.0 / (1.0 + exp(-z / 10000.0)); // Break ties consistently with high probability.
}

void TrackManagerColor::sortTracks() {
  sortTracks(&getTrackLength);
}
  
void TrackManagerColor::sortTracks(double (*rateTrack)(const Track&)) {
  vector< pair<double, boost::shared_ptr<Track> > > length_idx(tracks_.size());
  for(size_t i=0; i<tracks_.size(); ++i) {
    length_idx[i].first = rateTrack(*tracks_[i]);
    length_idx[i].second = tracks_[i];
  }
  greater< pair<double, boost::shared_ptr<Track> > > emacs = greater< pair<double, boost::shared_ptr<Track> > >();
  sort(length_idx.begin(), length_idx.end(), emacs); //Descending.

  for(size_t i=0; i<tracks_.size(); ++i) {
    tracks_[i] = length_idx[i].second;
  }
}

void TrackManagerColor::sortTracks(const vector<double>& track_ratings) {
  assert(track_ratings.size() == tracks_.size());
  vector< pair<double, boost::shared_ptr<Track> > > length_idx(tracks_.size());
  for(size_t i=0; i<tracks_.size(); ++i) {
    length_idx[i].first = track_ratings[i];
    length_idx[i].second = tracks_[i];
  }
  greater< pair<double, boost::shared_ptr<Track> > > emacs = greater< pair<double, boost::shared_ptr<Track> > >();
  sort(length_idx.begin(), length_idx.end(), emacs); //Descending.

  for(size_t i=0; i<tracks_.size(); ++i) {
    tracks_[i] = length_idx[i].second;
  }
}

void TrackManagerColor::insertTrack(boost::shared_ptr<Track> track) {
  tracks_.push_back(track);
}

void TrackManagerColor::reserve(size_t size) {
  tracks_.reserve(size);
}


void TrackManagerColor::getFramesNear(double timestamp, double tol,
				 std::vector< boost::shared_ptr<Frame> >* frames,
				 std::vector<std::string>* class_names,
				 std::vector<int>* track_ids) const
{
  assert(frames->empty());
  assert(class_names->empty());
  assert(track_ids->empty());

  frames->reserve(tracks_.size());
  class_names->reserve(tracks_.size());
  track_ids->reserve(tracks_.size());

  for(size_t i = 0; i < tracks_.size(); ++i) {
    size_t idx;
    bool valid = tracks_[i]->seek(timestamp, tol, &idx);
    if(valid) {
      frames->push_back(tracks_[i]->frames_[idx]);
      class_names->push_back(tracks_[i]->label_);
      track_ids->push_back(i);
    }
  }
}

  

/************************************************************/
/******************** Track ********************/
/************************************************************/

void Track::serialize(ostream& out) const {
  out << "Track" << endl;
  out << "serialization_version_" << endl;
  out << TRACK_SERIALIZATION_VERSION << endl;
  /*out << "label_" << endl;
  out << label_ << endl;
  out << "velodyne_offset_" << endl;
  for(int i = 0; i < 4; ++i) {
    for(int j = 0; j < 4; ++j) { 
      out.write((char*)&velodyne_offset_[i][j], sizeof(double));
    }
  }
  out << endl;*/
  
  out << "num_frames_" << endl;
  out << frames_.size() << endl;
  for(size_t i=0; i<frames_.size(); ++i) {
    printf("Serializing frame %zu\n", i);
    frames_[i]->serialize(out);
  }
}

bool Track::deserialize(istream& istrm) {
  if(istrm.eof())
    return false;

  long begin = istrm.tellg();
  frames_.clear();
  string line;

  getline(istrm, line);
  if(line.compare("Track") != 0) {
    istrm.seekg(begin);
    return false;
  }

  if (!checkLine(istrm, "serialization_version_")) return false;

  istrm >> serialization_version_;
  if(serialization_version_ != TRACK_SERIALIZATION_VERSION
     && serialization_version_ != 0) {
    cerr << "Track serialization version is wrong, aborting." << endl;
    return false;
  }
  getline(istrm, line);

if (!checkLine(istrm, "track_num_")) return false;

  /*getline(istrm, line);
  if(line.compare("track_num_") != 0)
    return false;*/
  istrm >> track_num_;
  getline(istrm, line);

  /*getline(istrm, line);
  if(line.compare("label_") != 0)
    return false;
  getline(istrm, label_);

  getline(istrm, line);
  if(line.compare("velodyne_offset_") != 0)
    return false;
  for(int i = 0; i < 4; ++i)
    for(int j = 0; j < 4; ++j)
      istrm.read((char*) &velodyne_offset_[i][j], sizeof(double));
  getline(istrm, line);*/
    
  getline(istrm, line);
  if(line.compare("num_frames_") != 0)
    return false;
  size_t num_frames = 0;
  istrm >> num_frames;
  getline(istrm, line);
  
  frames_.resize(num_frames);
  for(size_t i=0; i<num_frames; ++i) {
    assert(!frames_[i]);
    frames_[i] = boost::shared_ptr<Frame>(new Frame(istrm));
  }
  
  return true;
}

bool Track::seek(double timestamp, double max_time_difference, size_t* idx) {
  assert(idx);
  
  if(timestamp < frames_.front()->timestamp_ || timestamp > frames_.back()->timestamp_)
    return false;
    
  double min_delta = FLT_MAX;
  size_t best_idx = 0;
  for(size_t i = 0; i < frames_.size(); ++i) {
    //double delta = fabs(frames_[i]->timestamp_ - timestamp);
    double delta = fabs(frames_[i]->estimateAdjustedTimestamp() - timestamp);
    if(delta < min_delta) { 
      min_delta = delta;
      best_idx = i;
    }
  }

  if(min_delta <= max_time_difference) {
    *idx = best_idx;
    return true;
  }
  else {
    return false;
  }
}

bool Track::interpolatedSeek(double timestamp, double max_time_difference, size_t* idx, double* interpolation) {
  assert(idx);
  assert(interpolation);

  size_t nearest = 0;
  bool success = seek(timestamp, max_time_difference, &nearest);
  if(!success)
    return false;

  if(timestamp < frames_[nearest]->timestamp_)
    *idx = nearest - 1;
  else
    *idx = nearest;

  *interpolation = (timestamp - frames_[*idx]->timestamp_) / (frames_[*idx + 1]->timestamp_ - frames_[*idx]->timestamp_);
  
  return true;
}

double Track::getMeanNumPoints() const {
  double total = 0;
  for(size_t i = 0; i < frames_.size(); ++i) {
    total += frames_[i]->cloud_->points.size();
  }
  return total / (double)frames_.size();
}
  
double Track::getMeanDistance() {
  double total = 0;
  for(size_t i = 0; i < frames_.size(); ++i) {
    total += frames_[i]->getDistance();
  }
  return total / (double)frames_.size();
}
   
Track::Track(const std::string& label,
			    const std::vector< boost::shared_ptr<Frame> >& frames) :
  serialization_version_(TRACK_SERIALIZATION_VERSION),
  label_(label),
  frames_(frames)
{
}
  
Track::Track() :
  serialization_version_(TRACK_SERIALIZATION_VERSION),
  label_("unlabeled")
{
}

/*Track::Track(istream& istrm) :
  serialization_version_(TRACK_SERIALIZATION_VERSION),
  label_("unlabeled")
{
  long begin = istrm.tellg();
  istrm.seekg(begin);

  bool success = deserialize(istrm);
  if(!success)
    throw 1;
}*/

void Track::reserve(size_t num) {
  frames_.reserve(num);
}

void Track::insertFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                       double timestamp)
{
  frames_.push_back(boost::shared_ptr<Frame>(new Frame(cloud, timestamp)));
}

bool Track::operator!=(const Track& tr) {
  return !operator==(tr);
}

bool Track::operator==(const Track& tr) {
  if(tr.frames_.size() != frames_.size())
    return false;
  if(tr.label_.compare(label_) != 0)
    return false;
  for(size_t i=0; i<frames_.size(); ++i) {
    if(! (*frames_[i] == *tr.frames_[i])) {
      //cout << "Frame " << i << " differs." << endl;
      return false;
    }
  }
  return true;
}

/************************************************************/
/******************** Frame  ********************/
/************************************************************/

bool Frame::operator!=(const Frame& fr) {
  return !operator==(fr);
}

bool Frame::operator==(const Frame& fr) {
  if(!floatEq(timestamp_, fr.timestamp_)) {
    //cout << "Timestamps differ: " << timestamp_ << " " << fr.timestamp_ << endl;
    return false;
  }
  
  if(!cloudsEqual(*cloud_, *fr.cloud_)) {
    //cout << "Clouds differ." << endl;
    return false;
  }

  return true;
}

Frame::Frame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    double timestamp) :
  serialization_version_(FRAME_SERIALIZATION_VERSION),
  cloud_(cloud),
  timestamp_(timestamp)
{
}

Frame::Frame(istream& istrm) :
  serialization_version_(FRAME_SERIALIZATION_VERSION),
  spin_offset_(-1)
{
  assert(deserialize(istrm));
  assert(!centroid_);
}

bool Frame::deserialize(std::istream& istrm) {
  string line;
  getline(istrm, line);
  if(line.compare("Frame") != 0) {
    cout << "Expected 'Frame', got " << line << endl;
    return false;
  }
  
  getline(istrm, line);
  if(line.compare("serialization_version_") != 0) {
    cout << "Expected 'serialization_version_', got " << line << endl;
    return false;
  }
  istrm >> serialization_version_;
  if(serialization_version_ != FRAME_SERIALIZATION_VERSION) {
    cerr << "Frame serialization version is " << serialization_version_ << ", expected " << FRAME_SERIALIZATION_VERSION << ", aborting." << endl;
    return false;
  }
  getline(istrm, line);

  getline(istrm, line);
  if(line.compare("timestamp_") != 0) {
    cout << "Expected 'timestamp', got " << line << endl;
    return false;
  }
  istrm.read((char*)&timestamp_, sizeof(double));
  getline(istrm, line);

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  deserializePointCloud(istrm, cloud);

  cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  *cloud_ = cloud;

  /*getline(istrm, line);
  if(line.compare("robot_pose_") != 0) {
    cout << "Expected 'robot_pose_', got " << line << endl;
    return false;
  }
  istrm.read((char*)&robot_pose_.x, sizeof(double));
  istrm.read((char*)&robot_pose_.y, sizeof(double));
  istrm.read((char*)&robot_pose_.z, sizeof(double));
  istrm.read((char*)&robot_pose_.roll, sizeof(double));
  istrm.read((char*)&robot_pose_.pitch, sizeof(double));
  istrm.read((char*)&robot_pose_.yaw, sizeof(double));
  getline(istrm, line);*/

  //cloud_ = boost::shared_ptr<pcl::PointCloud>(new pcl::PointCloud());
  /*cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_->header.stamp = 0; //ros::Time(1); //Avoid a warning about timestamps from ROS.  We aren't using them anyway.
  cloud_->header.frame_id = "Frame_id";
  bool success = true;*/
  //cloud_->header.line = "Header"; //set a default header to avoid warnings
  //bool success = deserializePointCloudROS(istrm, cloud_.get());

  //printf("smooth_points_matrix_: rows: %d, cols: %d\n", smooth_points_matrix_.rows(), smooth_points_matrix_.cols());

  //printf("Converting smooth cloud to matrix format\n");
  //smoothCloudToMatrix();

  //printf("smooth_points_matrix_: rows: %d, cols: %d\n", smooth_points_matrix_.rows(), smooth_points_matrix_.cols());

  return true;
}


void Frame::serialize(std::ostream& out) const{
  printf("Serializing frame\n");
  out << "Frame" << endl;
  out << "serialization_version_" << endl;
  out << FRAME_SERIALIZATION_VERSION << endl;
  out << "timestamp_" << endl;
  out.write((char*) &timestamp_, sizeof(double));
  out << endl;
  /*out << "robot_pose_" << endl;
  out.write((char*) &robot_pose_.x, sizeof(double));
  out.write((char*) &robot_pose_.y, sizeof(double));
  out.write((char*) &robot_pose_.z, sizeof(double));
  out.write((char*) &robot_pose_.roll, sizeof(double));
  out.write((char*) &robot_pose_.pitch, sizeof(double));
  out.write((char*) &robot_pose_.yaw, sizeof(double));
  out << endl;*/
  //serializePointCloud(*cloud_, out);
  out << endl;
}
//     out << "timestamp" << endl;
//     out.write((char*)&timestamps_[i], sizeof(double));
//     out << endl;
//     assert(velodyne_centers_[i].size() == 3);
//     out << "velo_center" << endl;
//     out.write((char*)&velodyne_centers_[i][0], sizeof(float));
//     out.write((char*)&velodyne_centers_[i][1], sizeof(float));
//     out.write((char*)&velodyne_centers_[i][2], sizeof(float));
//     out << endl;
//     serializePointCloudROS(*clouds_[i], out);
//     out << endl;
  

bool Frame::smoothCloudToMatrix() const{
  if(cloud_->points.size() < 1){
  	printf("Cloud has no points - not creating smooth_points_matrix_\n");
    return false;
  }
    
  smooth_points_matrix_ = MatrixXd(cloud_->points.size(), 4);
  for(size_t i = 0; i < cloud_->points.size(); ++i) {
  	smooth_points_matrix_.coeffRef(i, 0) = cloud_->points[i].x;
  	smooth_points_matrix_.coeffRef(i, 1) = cloud_->points[i].y;
  	smooth_points_matrix_.coeffRef(i, 2) = cloud_->points[i].z;
  	smooth_points_matrix_.coeffRef(i, 3) = 1;
  }

  printf("Created smooth_points_matrix_\n");

  return true;
}

bool Frame::getCloudInVeloCoords(Eigen::MatrixXd* velo_points) const {
  if(cloud_->points.size() < 1)
    return false;

  if (smooth_points_matrix_.rows() == 0 && smooth_points_matrix_.cols() == 0) {
  	smoothCloudToMatrix();
  }

  printf("smooth_points_matrix_: Rows: %ld, cols: %ld\n",
      smooth_points_matrix_.rows(), smooth_points_matrix_.cols());

  *velo_points = smooth_points_matrix_ * smooth_to_velo_.cast<double>();
  return true;
}

/*dgc_pose_t findRobotPose(const double& targetTimeStamp, const dgc::dgc_velodyne_spin& spin){
  double bestDelta = 99999;
  int bestSpin = -1;

  for(int i = 0; i < spin.num_scans; i++) {
    printf("Spin: %d, timestamp: %lf\n", i, spin.scans[i].pose[0].timestamp);
    double delta = fabs(spin.scans[i].pose[0].timestamp - targetTimeStamp);
    if (delta < bestDelta){
      bestDelta = delta;
      bestSpin = i;
    }
  }

  printf("Best delta: %lf\n", bestDelta);

  return spin.scans[bestSpin].robot;

}*/

void Frame::getVelodyneXYZ(double* x, double* y, double* z) const {
  // -- Get the velodyne center.
  *x = 0;
  *y = 0;
  *z = 0;
}

  Vector3f Frame::getCentroid() {
    if(centroid_)
      return *centroid_;

    centroid_ = boost::shared_ptr<Vector3f>(new Vector3f());
    *centroid_ = Vector3f::Zero();
    for(size_t i = 0; i < cloud_->points.size(); ++i) {
      centroid_->coeffRef(0) += cloud_->points[i].x;
      centroid_->coeffRef(1) += cloud_->points[i].y;
      centroid_->coeffRef(2) += cloud_->points[i].z;
    }
    *centroid_ /= (double)cloud_->points.size();

    return *centroid_;
  }

  Eigen::MatrixXf Frame::getBoundingBoxVelo() {
    MatrixXf boundingBoxSmooth = getBoundingBox();

    MatrixXf boundingBoxVelo = boundingBoxSmooth * smooth_to_velo_;//.cast<double>();

    return boundingBoxVelo;
  }
      

  MatrixXf Frame::getBoundingBox() {
    if(bounding_box_)
      return *bounding_box_;

    bounding_box_ = boost::shared_ptr<MatrixXf>(new MatrixXf(2, 2));
    MatrixXf& bb = *bounding_box_;
    bb(0, 0) = FLT_MAX; // Small x.
    bb(1, 0) = FLT_MAX; // Small y.
    bb(0, 1) = -FLT_MAX; // Big x.
    bb(1, 1) = -FLT_MAX; // Big y.
    for(size_t i = 0; i < cloud_->points.size(); ++i) {
      double x = cloud_->points[i].x;
      double y = cloud_->points[i].y;
      if(x < bb(0, 0))
	bb(0, 0) = x;
      if(x > bb(0, 1))
	bb(0, 1) = x;
      if(y < bb(1, 0))
	bb(1, 0) = y;
      if(y > bb(1, 1))
	bb(1, 1) = y;
    }

    return *bounding_box_;
  }

  double Frame::getDistance() {
    Vector3f centroid = getCentroid();
    return (centroid.cast<double>()).norm();
  }

  double Frame::estimateSpinOffset() {
    if(spin_offset_ != -1) {
    	printf("Spin offset already computed:");
    	printf("Spin offset: %lf", spin_offset_);
      return spin_offset_;
    }

    if(cloud_->points.size() == 0) {
    	printf("Error - empty point cloud.  No spin offset.");
      return 0;
    }

    //printf("Estimating spin offset from points\n");
    MatrixXd cloud;
    bool valid = getCloudInVeloCoords(&cloud);

    // Compute the centroid.
    VectorXd mean = cloud.colwise().sum() / (double)cloud.rows();

    double x = mean(0);
    double y = mean(1);
    double angle = atan2(-y, -x);
    if(angle < 0)
      angle += 2.0 * M_PI;
    if(!(angle >= 0.0 && angle <= 2.0 * M_PI)) {
      cout << "npts: " << cloud.rows() << endl;
      cout << "valid: " << valid << endl;
      cout << cloud << endl;
      cout << "--- " << endl;
      cout << smooth_to_velo_ << endl;
      cout << "--- " << endl;
      cout << cloud_ << endl;
      cout << "angle: " << angle << endl;
      cout << "x: " << x << ", y: " << y << endl;
    }

    spin_offset_ = angle / (2.0 * M_PI);
    if(!(spin_offset_ >= 0.0 && spin_offset_ <= 1.0))
      cout << "spin_offset_: " << spin_offset_ << endl;
    
    return spin_offset_;
  }
    
  double Frame::estimateAdjustedTimestamp() {
    double spin_time = 0.1;
    double offset = estimateSpinOffset();
    double adjusted_timestamp = timestamp_ + offset * spin_time;
    return adjusted_timestamp;
  }

/************************************************************/
/******************** Helper Functions ********************/
/************************************************************/

bool checkLine(istream& istrm, const string& expected_input) {
  string line;
  getline(istrm, line);
  if(line.compare(expected_input.c_str()) != 0) {
      printf("Expected: '%s', got: '%s'\n", expected_input.c_str(), line.c_str());
    return false;
  }
  return true;
}

int readHeader(std::istream& fs, pcl::PCLPointCloud2 &cloud,
               Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
               int &pcd_version, int &data_type, unsigned int &data_idx) {
  // Default values
  data_idx = 0;
  data_type = 0;
  pcd_version = pcl::PCDReader::PCD_V6;
  origin      = Eigen::Vector4f::Zero ();
  orientation = Eigen::Quaternionf::Identity ();
  cloud.width = cloud.height = cloud.point_step = cloud.row_step = 0;
  cloud.data.clear ();

  // By default, assume that there are _no_ invalid (e.g., NaN) points
  //cloud.is_dense = true;

  int nr_points = 0;
  //std::ifstream fs;
  std::string line;

  int specified_channel_count = 0;

  /*if (file_name == "" || !boost::filesystem::exists (file_name))
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] Could not find file '%s'.\n", file_name.c_str ());
    return (-1);
  }*/

  // Open file in binary mode to avoid problem of
  // std::getline() corrupting the result of ifstream::tellg()
  /*fs.open (file_name.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] Could not open file '%s'! Error : %s\n", file_name.c_str (), strerror(errno));
    fs.close ();
    return (-1);
  }

  // Seek at the given offset
  fs.seekg (offset, std::ios::beg);*/

  // field_sizes represents the size of one element in a field (e.g., float = 4, char = 1)
  // field_counts represents the number of elements in a field (e.g., x = 1, normal_x = 1, fpfh = 33)
  std::vector<int> field_sizes, field_counts;
  // field_types represents the type of data in a field (e.g., F = float, U = unsigned)
  std::vector<char> field_types;
  std::vector<std::string> st;

  // Read the header and fill it in with wonderful values
  try
  {
    while (!fs.eof ())
    {
      getline (fs, line);
      // Ignore empty lines
      if (line == "")
        continue;

      // Tokenize the line
      boost::trim (line);
      boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

      std::stringstream sstream (line);
      sstream.imbue (std::locale::classic ());

      std::string line_type;
      sstream >> line_type;

      // Ignore comments
      if (line_type.substr (0, 1) == "#")
        continue;

      // Version numbers are not needed for now, but we are checking to see if they're there
      if (line_type.substr (0, 7) == "VERSION")
        continue;

      // Get the field indices (check for COLUMNS too for backwards compatibility)
      if ( (line_type.substr (0, 6) == "FIELDS") || (line_type.substr (0, 7) == "COLUMNS") )
      {
        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        cloud.fields.resize (specified_channel_count);
        for (int i = 0; i < specified_channel_count; ++i)
        {
          std::string col_type = st.at (i + 1);
          cloud.fields[i].name = col_type;
        }

        // Default the sizes and the types of each field to float32 to avoid crashes while using older PCD files
        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i, offset += 4)
        {
          cloud.fields[i].offset   = offset;
          cloud.fields[i].datatype = pcl::PCLPointField::FLOAT32;
          cloud.fields[i].count    = 1;
        }
        cloud.point_step = offset;
        continue;
      }

      // Get the field sizes
      if (line_type.substr (0, 4) == "SIZE")
      {
        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <SIZE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_sizes.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          int col_type ;
          sstream >> col_type;
          cloud.fields[i].offset = offset;                // estimate and save the data offsets
          offset += col_type;
          field_sizes[i] = col_type;                      // save a temporary copy
        }
        cloud.point_step = offset;
        //if (cloud.width != 0)
          //cloud.row_step   = cloud.point_step * cloud.width;
        continue;
      }

      // Get the field types
      if (line_type.substr (0, 4) == "TYPE")
      {
        if (field_sizes.empty ())
          throw "TYPE of FIELDS specified before SIZE in header!";

        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <TYPE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_types.resize (specified_channel_count);

        for (int i = 0; i < specified_channel_count; ++i)
        {
          field_types[i] = st.at (i + 1).c_str ()[0];
          cloud.fields[i].datatype = static_cast<uint8_t> (pcl::getFieldType (field_sizes[i], field_types[i]));
        }
        continue;
      }

      // Get the field counts
      if (line_type.substr (0, 5) == "COUNT")
      {
        if (field_sizes.empty () || field_types.empty ())
          throw "COUNT of FIELDS specified before SIZE or TYPE in header!";

        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <COUNT> differs than the number of elements in <FIELDS>!";

        field_counts.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          cloud.fields[i].offset = offset;
          int col_count;
          sstream >> col_count;
          cloud.fields[i].count = col_count;
          offset += col_count * field_sizes[i];
        }
        // Adjust the offset for count (number of elements)
        cloud.point_step = offset;
        continue;
      }

      // Get the width of the data (organized point cloud dataset)
      if (line_type.substr (0, 5) == "WIDTH")
      {
        sstream >> cloud.width;
        if (cloud.point_step != 0)
          cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
        continue;
      }

      // Get the height of the data (organized point cloud dataset)
      if (line_type.substr (0, 6) == "HEIGHT")
      {
        sstream >> cloud.height;
        continue;
      }

      // Check the format of the acquisition viewpoint
      if (line_type.substr (0, 9) == "VIEWPOINT")
      {
        if (st.size () < 8)
          throw "Not enough number of elements in <VIEWPOINT>! Need 7 values (tx ty tz qw qx qy qz).";
        continue;
      }

      // Get the number of points
      if (line_type.substr (0, 6) == "POINTS")
      {
        sstream >> nr_points;
        // Need to allocate: N * point_step
        cloud.data.resize (nr_points * cloud.point_step);
        continue;
      }
      break;
    }
  }
  catch (const char *exception)
  {
    printf ("[pcl::PCDReader::readHeader] %s\n", exception);
    //fs.close ();
    return (-1);
  }

  // Exit early: if no points have been given, there's no sense to read or check anything anymore
  if (nr_points == 0)
  {
    printf ("[pcl::PCDReader::readHeader] No points to read\n");
    //fs.close ();
    return (-1);
  }

  // Compatibility with older PCD file versions
  if (cloud.width == 0 && cloud.height == 0)
  {
    cloud.width  = nr_points;
    cloud.height = 1;
    cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
  }
  //assert (cloud.row_step != 0);       // If row_step = 0, either point_step was not set or width is 0

  // if both height/width are not given, assume an unorganized dataset
  if (cloud.height == 0)
  {
    cloud.height = 1;
    PCL_WARN ("[pcl::PCDReader::readHeader] no HEIGHT given, setting to 1 (unorganized).\n");
    if (cloud.width == 0)
      cloud.width  = nr_points;
  }
  else
  {
    if (cloud.width == 0 && nr_points != 0)
    {
      printf ("[pcl::PCDReader::readHeader] HEIGHT given (%d) but no WIDTH!\n", cloud.height);
      //fs.close ();
      return (-1);
    }
  }

  if (int (cloud.width * cloud.height) != nr_points)
  {
    printf("[pcl::PCDReader::readHeader] HEIGHT (%d) x WIDTH (%d) != number of points (%d)\n", cloud.height, cloud.width, nr_points);
    //fs.close ();
    return (-1);
  }

  // Close file
  //fs.close ();

  return (0);
}

int readCloud(std::istream& fs, pcl::PCLPointCloud2 &cloud,
              Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
              int &pcd_version) {
  int data_type;
  unsigned int data_idx;

  int res = readHeader (fs, cloud, origin, orientation, pcd_version, data_type, data_idx);

  if (res < 0)
    return (res);

  unsigned int idx = 0;

  // Get the number of points the cloud should have
  unsigned int nr_points = cloud.width * cloud.height;

  // Setting the is_dense property to true by default
  cloud.is_dense = true;

  /*if (file_name == "" || !boost::filesystem::exists (file_name))
  {
    printf("[pcl::PCDReader::read] Could not find file '%s'.\n", file_name.c_str ());
    return (-1);
  }*/

  // if ascii
  if (data_type == 0)
  {
    // Re-open the file (readHeader closes it)
    /*std::ifstream fs;
    fs.open (file_name.c_str ());
    if (!fs.is_open () || fs.fail ())
    {
      PCL_ERROR ("[pcl::PCDReader::read] Could not open file %s.\n", file_name.c_str ());
      return (-1);
    }

    fs.seekg (data_idx);*/

    std::string line;
    std::vector<std::string> st;

    // Read the rest of the file
    try
    {
      while (idx < nr_points)
      {
        getline (fs, line);
        // Ignore empty lines
        if (line == "")
          continue;

        // Tokenize the line
        boost::trim (line);
        boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

        /*if (idx >= nr_points)
        {
          printf("[pcl::PCDReader::read] input cloud has more points (%d) than advertised (%d)!\n", idx, nr_points);
          break;
        }*/

        size_t total = 0;
        // Copy data
        for (unsigned int d = 0; d < static_cast<unsigned int> (cloud.fields.size ()); ++d)
        {
          // Ignore invalid padded dimensions that are inherited from binary data
          if (cloud.fields[d].name == "_")
          {
            total += cloud.fields[d].count; // jump over this many elements in the string token
            continue;
          }
          for (unsigned int c = 0; c < cloud.fields[d].count; ++c)
          {
            switch (cloud.fields[d].datatype)
            {
              case pcl::PCLPointField::INT8:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT8>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::UINT8:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT8>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::INT16:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT16>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::UINT16:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT16>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::INT32:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT32>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::UINT32:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT32>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::FLOAT32:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::FLOAT32>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::FLOAT64:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::FLOAT64>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              default:
                PCL_WARN ("[pcl::PCDReader::read] Incorrect field data type specified (%d)!\n",cloud.fields[d].datatype);
                break;
            }
          }
          total += cloud.fields[d].count; // jump over this many elements in the string token
        }
        idx++;
      }
    }
    catch (const char *exception)
    {
      printf("[pcl::PCDReader::read] %s\n", exception);
      //fs.close ();
      return (-1);
    }

    // Close file
    //fs.close ();
  }

  if ((idx != nr_points) && (data_type == 0))
  {
    printf("[pcl::PCDReader::read] Number of points read (%d) is different than expected (%d)\n", idx, nr_points);
    return (-1);
  }

  return (0);
}

template<typename PointT> int
readCloud(std::istream& s, pcl::PointCloud<PointT> &cloud) {
  pcl::PCLPointCloud2 blob;
  int pcd_version;
  printf("Reading cloud\n");

  pcl::uint8_t x;

  int res = readCloud(s, blob, cloud.sensor_origin_, cloud.sensor_orientation_,
                  pcd_version);
  printf("Done reading cloud\n");

  // If no error, convert the data
  if (res == 0)
    pcl::fromPCLPointCloud2 (blob, cloud);

  return (res);
}

bool readCloud(std::istream& s, pcl::PCLPointCloud2 &v)
{
  string line;

  /*if (!checkLine(s, "header")) return false;
  if (!checkLine(s, "seq")) return false;
  s >> v.header.seq;
  getline(s, line);

  if (!checkLine(s, "stamp")) return false;
  s >> v.header.stamp;
  getline(s, line);

  if (!checkLine(s, "frame_id")) return false;
  s >> v.header.frame_id;
  getline(s, line);*/

  if (!checkLine(s, "height")) return false;
  s >> v.height;
  getline(s, line);
  //printf("Read height: %d\n", v.height);

  if (!checkLine(s, "width")) return false;
  s >> v.width;
  //printf("Read width: %d\n", v.width);
  getline(s, line);

  int num_fields;
  if (!checkLine(s, "numfields")) return false;
  s >> num_fields;
  //printf("Read numfields: %d\n", num_fields);
  getline(s, line);

  v.fields.resize(num_fields);

  if (!checkLine(s, "fields[]")) return false;

  for (size_t i = 0; i < num_fields; ++i) {
    //printf("Reading field %zu\n", i);
    s >> v.fields[i].name;
    getline(s, line);
    //printf("Read name: %s\n", v.fields[i].name.c_str());

    s >> v.fields[i].offset;
    getline(s, line);

    s >> v.fields[i].datatype;
    getline(s, line);

    s >> v.fields[i].count;
    getline(s, line);
  }

  if (!checkLine(s, "is_bigendian: ")) return false;
  s >> v.is_bigendian;
  //printf("Read bigendian: %d\n", v.is_bigendian);
  getline(s, line);

  if (!checkLine(s, "point_step: ")) return false;
  s >> v.point_step;
  //printf("Read pointstep: %d\n", v.point_step);
  getline(s, line);

  if (!checkLine(s, "row_step: ")) return false;
  s >> v.row_step;
  //printf("Read row step: %d\n", v.row_step);
  getline(s, line);

  int num_data;
  if (!checkLine(s, "numdata: ")) return false;
  s >> num_data;
  //printf("Read num data: %d\n", num_data);
  getline(s, line);

  v.data.resize(num_data);

  size_t data_size;
  if (!checkLine(s, "datasize: ")) return false;
  s >> data_size;
  //printf("Read data size: %zu\n", data_size);
  getline(s, line);

  s.read((char*)&v.data[0], data_size);
  getline(s, line);

  /*if (!checkLine(s, "data[]")) return false;
  for (size_t i = 0; i < num_data; ++i)
  {
    printf("Read data %zu\n", i);
    int num;
    s >> num;
    getline(s, line);
    printf("Read num: %d\n", num);

   pcl::uint8_t data;
    s >> data;
    getline(s, line);
    v.data[i] = data;
  }*/


  if (!checkLine(s, "is_dense: ")) return false;
  s >> v.is_dense;
  //printf("Read is dense: %d\n", v.is_dense);
  getline(s, line);

  return true;
}

void  deserializePointCloud(std::istream& istrm,
    pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) {
  pcl::PCLPointCloud2 msg;
  readCloud(istrm, msg);

  pcl::fromPCLPointCloud2(msg, point_cloud);
}


/*void serializePointCloudROS(const sensor_msgs::PointCloud& cloud, ostream& out) {
  out << "Cloud" << endl;
  out << "serialization_length" << endl;
  out << cloud.serializationLength() << endl;
  uint8_t data[cloud.serializationLength()];
  cloud.serialize(data, 0);
  assert(sizeof(char*) == sizeof(uint8_t*));
  out.write((char*)data, cloud.serializationLength());
}*/

/*bool deserializePointCloudROS(std::istream& istrm, sensor_msgs::PointCloud* cloud) {
  string line;

  getline(istrm, line);
  if(line.compare("Cloud") != 0) {
    cout << "Expected 'Cloud', got " << line << endl;
    return false;
  }

  getline(istrm, line);
  if(line.compare("serialization_length") != 0) {
    cout << "Expected 'serialization_length', got " << line << endl;
    return false;
  }

  uint32_t serialization_length = 0;
  istrm >> serialization_length;
  getline(istrm, line);
  
  uint8_t data[serialization_length];
  istrm.read((char*)data, serialization_length);
  cloud->deserialize(data);
  getline(istrm, line);
  
  return true;
}*/

/*bool deserializePointCloudROS(std::istream& istrm, sensor_msgs::PointCloud* cloud) {
		string line;

		getline(istrm, line);
		if(line.compare("Cloud") != 0)
				return false;

		getline(istrm, line);
		if(line.compare("serialization_length") != 0)
				return false;

		uint32_t serialization_length = 0;
		istrm >> serialization_length;
		getline(istrm, line);

		boost::shared_array<uint8_t> buffer(new uint8_t[serialization_length]);
		istrm.read((char*)buffer.get(), serialization_length);
		ros::serialization::IStream stream(buffer.get(), serialization_length);
		stream >> *cloud;
		getline(istrm, line);

		return true;
}*/


/*bool deserializePointCloud(istream& istrm, PointCloud* cloud) {
  string line;

  getline(istrm, line);
  if(line.compare("Cloud") != 0)
    return false;

  getline(istrm, line);
  if(line.compare("serialization_version_") != 0)
    return false;

  int serialization_version = 0;
  istrm >> serialization_version;
  if(serialization_version != POINTCLOUD_SERIALIZATION_VERSION)
    return false;
  getline(istrm, line);
 
  getline(istrm, line);
  if(line.compare("num_points") != 0)
    return false;

  size_t num_points = 0;
  istrm >> num_points;
  getline(istrm, line);
  
  getline(istrm, line);
  if(line.compare("points") != 0)
    return false;

  float* buf = (float*)malloc(sizeof(float)*num_points*3);
  istrm.read((char*)buf, sizeof(float)*num_points*3);
  cloud->set_points_size(num_points);
  for(size_t i=0; i<num_points; ++i) {
    cloud->points[i].x = buf[i*3];
    cloud->points[i].y = buf[i*3+1];
    cloud->points[i].z = buf[i*3+2];
  }
  free(buf);
  return true;
}

void serializePointCloud(const sensor_msgs::PointCloud& cloud, ostream& out) {
  out << "Cloud" << endl;
  out << "serialization_version_" << endl;
  out << POINTCLOUD_SERIALIZATION_VERSION << endl;
  out << "num_points" << endl;
  out << cloud.points.size() << endl;
  out << "points" << endl;

  float* buf = (float*)malloc(sizeof(float)*cloud.points.size()*3);
  for(size_t i=0; i<cloud.points.size(); ++i) {
    buf[i*3] = cloud.points[i].x;
    buf[i*3+1] = cloud.points[i].y;
    buf[i*3+2] = cloud.points[i].z;
  }
  out.write((char*)buf, sizeof(float)*cloud.points.size()*3);
  free(buf);
}*/


bool cloudsEqual(const pcl::PointCloud<pcl::PointXYZRGB>& c1,
    const pcl::PointCloud<pcl::PointXYZRGB>& c2) {

  // -- Check the points.
  /*if(c1.points.size() != c2.points.size()) {
    //cout << "Different number of points: " << c1.get_points_size() << " " << c2.get_points_size() << endl;
    return false;
  }

  for(size_t i=0; i<c1.points.size(); ++i) {
    if(!floatEq(c1.points[i].x, c2.points[i].x) ||
       !floatEq(c1.points[i].y, c2.points[i].y) ||
       !floatEq(c1.points[i].z, c2.points[i].z)) {
      //cout << "Points are different" << endl;
      return false;
    }
  }

  // -- Check the channels.
  if(c1.channels.size() != c2.channels.size()) {
    //cout << "Different number of channels." << endl;
    return false;
  }

  for(size_t i=0; i<c1.channels.size(); ++i) {
    if(c1.channels[i].values.size() != c1.channels[i].values.size())
      return false;
    for(size_t j=0; j<c1.channels[i].values.size(); ++j) {
      if(!floatEq(c1.channels[i].values[j], c2.channels[i].values[j]))
	return false;
    }
  }*/

  return true;
}


/*bool streamTrack(std::string track_manager_filename, const Track& tr) {
  // -- Stick the track on the end.
  ofstream out;
  out.open(track_manager_filename.c_str(), ios::out | ios::app | ios::binary); 
  tr.serialize(out);
  out.close();
  
  return true;
}*/

/*boost::shared_ptr<PointCloud> getRandomCloud() {
  int num_pts = 100;

  boost::shared_ptr<PointCloud> cloud(new PointCloud());
  cloud->points.resize(num_pts);
  cloud->channels.resize(1);
  cloud->channels[0].values.resize(num_pts);
  double mean_x = rand() % 1000;
  double mean_y = rand() % 1000;
  double mean_z = rand() % 1000;
  for(size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = mean_x + (double)(rand() % 1000) / 1000.0;
    cloud->points[i].y = mean_y + (double)(rand() % 1000) / 1000.0;
    cloud->points[i].z = mean_z + (double)(rand() % 1000) / 1000.0;
    cloud->channels[0].values[i] = rand() % 256;
  }

  return cloud;
}

void getRandomTransform(dgc_transform_t trans) {
  dgc_transform_identity(trans);
  dgc_transform_translate(trans, rand()%1000, rand()%1000, rand()%1000);
}
  
boost::shared_ptr<Frame> getRandomFrame(dgc_transform_t velodyne_offset) {
  boost::shared_ptr<PointCloud> cloud = getRandomCloud();
  double timestamp = (double)(rand() % 1000000) / (double)1e3;

  dgc_pose_t robot_pose;
  robot_pose.x = rand() % 1000;
  robot_pose.y = rand() % 1000;
  robot_pose.z = rand() % 1000;
  robot_pose.roll = (double)(rand() % 2 * M_PI * 1e6) / (double)1e6;
  robot_pose.pitch = (double)(rand() % 2 * M_PI * 1e6) / (double)1e6;
  robot_pose.yaw = (double)(rand() % 2 * M_PI * 1e6) / (double)1e6;

  return boost::shared_ptr<Frame>(new Frame(cloud, timestamp, robot_pose, velodyne_offset));
}

boost::shared_ptr<Track> getRandomTrack() {
  int num_frames = 10;

  int cl = rand() % 3;
  string label;
  switch(cl) {
  case 0:
    label = "bicyclist";
    break;
  case 1:
    label = "pedestrian";
    break;
  case 2:
    label = "car";
    break;
  }

  dgc_transform_t velodyne_offset;
  getRandomTransform(velodyne_offset);

  vector< boost::shared_ptr<Frame> > frames(num_frames);
  for(size_t i = 0; i < frames.size(); ++i)
    frames[i] = getRandomFrame(velodyne_offset);

  return boost::shared_ptr<Track>(new Track(label, velodyne_offset, frames));
}

boost::shared_ptr<TrackManagerColor> getRandomTrackManager() {
  int num_tracks = 15;

  vector< boost::shared_ptr<Track> > tracks(num_tracks);
  for(size_t i = 0; i < tracks.size(); ++i)
    tracks[i] = getRandomTrack();

  return boost::shared_ptr<TrackManagerColor>(new TrackManagerColor(tracks));
}*/

// See http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm.
bool floatEq(float x, float y, int maxUlps)
{
  // Make sure maxUlps is non-negative and small enough that the
  // default NAN won't compare as equal to anything.
  assert(maxUlps > 0 && maxUlps < 4 * 1024 * 1024);
  int aInt = *(int*)&x;
  // Make aInt lexicographically ordered as a twos-complement int
  if (aInt < 0)
    aInt = 0x80000000 - aInt;
  // Make bInt lexicographically ordered as a twos-complement int
  int bInt = *(int*)&y;
  if (bInt < 0)
    bInt = 0x80000000 - bInt;
  int intDiff = abs(aInt - bInt);
  if (intDiff <= maxUlps)
    return true;
  return false;
}  
  
}// namespace track_manager
