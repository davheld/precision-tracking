/*
 * track_manager_color.cpp
 *
 *      Author: Alex Teichman
 *
 */

#include <vector>

#include <pcl/conversions.h>
#include <pcl/io/file_io.h>

#include <precision_tracking/track_manager_color.h>


/************************************************************/
/******************** TrackManager ********************/
/************************************************************/

using std::vector;
using std::string;
using std::ifstream;
using std::istream;
using std::ofstream;
using std::ostream;
using std::ios;
using std::endl;
using std::cerr;
using std::pair;
using std::cout;

namespace precision_tracking {

namespace track_manager_color {

TrackManagerColor::TrackManagerColor() :
  serialization_version_(TRACKMANAGER_SERIALIZATION_VERSION),
  tracks_(vector< boost::shared_ptr<Track> >())
{
}

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
        // Once we find the target tracknum, exit.
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
  std::greater< pair<double, boost::shared_ptr<Track> > > emacs =
      std::greater< pair<double, boost::shared_ptr<Track> > >();
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
  std::greater< pair<double, boost::shared_ptr<Track> > > emacs =
      std::greater< pair<double, boost::shared_ptr<Track> > >();
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


/************************************************************/
/******************** Track ********************/
/************************************************************/

void Track::serialize(ostream& out) const {
  out << "Track" << endl;
  out << "serialization_version_" << endl;
  out << TRACK_SERIALIZATION_VERSION << endl;
  
  out << "num_frames_" << endl;
  out << frames_.size() << endl;
  for(size_t i=0; i<frames_.size(); ++i) {
    printf("Serializing frame %zu\n", i);
    frames_[i]->serialize(out);
  }
}

bool Track::deserialize(istream& istrm) {
  if(istrm.eof()) {
    printf("Error - Reached end of file\n");
    return false;
  }

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
  istrm >> track_num_;
  getline(istrm, line);
    
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
  serialization_version_(FRAME_SERIALIZATION_VERSION)
{
  deserialize(istrm);
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
    cerr << "Frame serialization version is " << serialization_version_
         << ", expected " << FRAME_SERIALIZATION_VERSION << ", aborting."
         << endl;
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
  serializePointCloud(*cloud_, out);
}

Eigen::Vector3f Frame::getCentroid() {
  if(centroid_)
    return *centroid_;

  centroid_ = boost::shared_ptr<Eigen::Vector3f>(new Eigen::Vector3f());
  *centroid_ = Eigen::Vector3f::Zero();
  for(size_t i = 0; i < cloud_->points.size(); ++i) {
    centroid_->coeffRef(0) += cloud_->points[i].x;
    centroid_->coeffRef(1) += cloud_->points[i].y;
    centroid_->coeffRef(2) += cloud_->points[i].z;
  }
  *centroid_ /= (double)cloud_->points.size();

  return *centroid_;
}

Eigen::MatrixXf Frame::getBoundingBox() {
  if(bounding_box_)
    return *bounding_box_;

  bounding_box_ = boost::shared_ptr<Eigen::MatrixXf>(new Eigen::MatrixXf(2, 2));
  Eigen::MatrixXf& bb = *bounding_box_;
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
  Eigen::Vector3f centroid = getCentroid();
  return (centroid.cast<double>()).norm();
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

bool readCloud(std::istream& s, pcl::PCLPointCloud2 &cloud)
{
  string line;
  if (!checkLine(s, "height")) return false;
  s >> cloud.height;
  getline(s, line);

  if (!checkLine(s, "width")) return false;
  s >> cloud.width;
  getline(s, line);

  int num_fields;
  if (!checkLine(s, "numfields")) return false;
  s >> num_fields;
  getline(s, line);

  cloud.fields.resize(num_fields);

  if (!checkLine(s, "fields[]")) return false;

  for (int i = 0; i < num_fields; ++i) {
    s >> cloud.fields[i].name;
    getline(s, line);

    s >> cloud.fields[i].offset;
    getline(s, line);

    s >> cloud.fields[i].datatype;
    getline(s, line);

    s >> cloud.fields[i].count;
    getline(s, line);
  }

  if (!checkLine(s, "is_bigendian: ")) return false;
  s >> cloud.is_bigendian;
  getline(s, line);

  if (!checkLine(s, "point_step: ")) return false;
  s >> cloud.point_step;
  getline(s, line);

  if (!checkLine(s, "row_step: ")) return false;
  s >> cloud.row_step;
  getline(s, line);

  int num_data;
  if (!checkLine(s, "numdata: ")) return false;
  s >> num_data;
  getline(s, line);

  cloud.data.resize(num_data);

  size_t data_size;
  if (!checkLine(s, "datasize: ")) return false;
  s >> data_size;
  getline(s, line);

  s.read((char*)&cloud.data[0], data_size);
  getline(s, line);

  if (!checkLine(s, "is_dense: ")) return false;
  s >> cloud.is_dense;
  getline(s, line);

  return true;
}

void  deserializePointCloud(std::istream& istrm,
    pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) {
  pcl::PCLPointCloud2 msg;
  readCloud(istrm, msg);

  pcl::fromPCLPointCloud2(msg, point_cloud);

  for (size_t i = 0; i < point_cloud.size(); ++i) {
    pcl::PointXYZRGB& pt = point_cloud[i];
  }
}

inline std::ostream& writeCloud(std::ostream& s,
                                const pcl::PCLPointCloud2 &cloud)
{
  s << "height" << std::endl;
  s << cloud.height << std::endl;
  s << "width" << std::endl;
  s << cloud.width << std::endl;
  s << "numfields" << std::endl;
  s << cloud.fields.size() << std::endl;
  s << "fields[]" << std::endl;
  for (size_t i = 0; i < cloud.fields.size (); ++i)
  {
    s << cloud.fields[i].name << std::endl;
    s << cloud.fields[i].offset << std::endl;
    s << cloud.fields[i].datatype << std::endl;
    s << cloud.fields[i].count << std::endl;
  }
  s << "is_bigendian: " << std::endl;
  s << "  " << cloud.is_bigendian << std::endl;
  s << "point_step: " << std::endl;
  s << "  " << cloud.point_step << std::endl;
  s << "row_step: " << std::endl;
  s << "  " << cloud.row_step << std::endl;
  s << "numdata: " << std::endl;
  s << "  " << cloud.data.size() << std::endl;

  size_t data_size = sizeof (cloud.data[0]) * cloud.data.size ();
  s << "datasize: " << std::endl;
  s << "  " << data_size << std::endl;

  s.write((const char*)&cloud.data[0], data_size);
  s << std::endl;

  s << "is_dense: " << std::endl;
  s << cloud.is_dense << std::endl;

  return s;
}


void serializePointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB> cloud, ostream& out) {
  pcl::PCLPointCloud2 msg;
  pcl::toPCLPointCloud2(cloud, msg);

  writeCloud(out, msg);
}

bool cloudsEqual(const pcl::PointCloud<pcl::PointXYZRGB>& c1,
    const pcl::PointCloud<pcl::PointXYZRGB>& c2) {

  // -- Check the points.
  if(c1.points.size() != c2.points.size()) {
    //cout << "Different number of points: " << c1.get_points_size()
    // << " " << c2.get_points_size() << endl;
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

  return true;
}

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
  
} // namespace track_manager

} // namespace precision_tracking
