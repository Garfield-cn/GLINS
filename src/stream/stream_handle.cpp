/**
* @Function: Handle stream data input, log, and output
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/stream/stream_handle.h"

#include "gici/gnss/gnss_common.h"

namespace gici {

StreamHandle::StreamHandle(YAML::Node& node)
{
  // Initialize streamers
  if (!node["streamers"].IsDefined()) {
    LOG(ERROR) << "Unable to load streamers!";
    return;
  }
  YAML::Node streamer_nodes = node["streamers"];
  for (size_t i = 0; i < streamer_nodes.size(); i++) {
    // check if ROS streamer
    YAML::Node streamer_node = streamer_nodes[i]["streamer"];
    if (!streamer_node["type"].IsDefined()) {
      LOG(FATAL) << "Unable to load streamer type!";
    }
    std::string type_str = streamer_node["type"].as<std::string>();
    StreamerType type;
    option_tools::convert(type_str, type);
    if (type == StreamerType::Ros) continue;

    auto streaming = std::make_shared<Streaming>(node, i);
    if (!streaming->valid()) continue;
    streamings_.push_back(streaming);
  }

  // Get formator options to determine the behaviors to callbacks
  if (!node["formators"].IsDefined()) {
    LOG(ERROR) << "Unable to load formators!";
    return;
  }
  for (auto it : node["formators"]) {
    if (!it["formator"].IsDefined()) {
      LOG(ERROR) << "Unable to load formator!";
      continue;
    }

    std::string io_type_str;
    if (!option_tools::safeGet(it["formator"], "io", &io_type_str)) {
      LOG(ERROR) << "Unable to load formator I/O type!";
      continue;
    }
    StreamIOType io_type;
    option_tools::convert(io_type_str, io_type);
    if (io_type != StreamIOType::Input) continue;

    std::string formator_tag;
    if (!option_tools::safeGet(it["formator"], "tag", &formator_tag)) {
      LOG(ERROR) << "Unable to load formator tag!";
      continue;
    }

    std::vector<std::string> formator_roles;
    if (!option_tools::safeGet(it["formator"], "role", &formator_roles)) {
      LOG(ERROR) << "Unable to load formator role!";
      continue;
    }

    Behaviors behavior; 
    behavior.role = formator_roles;
    behaviors_.insert(std::make_pair(formator_tag, behavior));
  }

  // Get replay option and enable replay
  bool enable_replay = false;
  StreamerReplayOptions replay_options;
  if (!node["replay"].IsDefined() || 
      !option_tools::safeGet(node["replay"], "enable", &enable_replay)) {
    LOG(INFO) << "Unable to load replay options. Disable replay!";
  }
  if (enable_replay) {
    if (!option_tools::safeGet(node["replay"], "speed", &replay_options.speed)) {
      LOG(INFO) << "Unable to load replay speed! Using default instead";
      replay_options.speed = 1.0;
    }
    if (!option_tools::safeGet(node["replay"], "start_offset", 
        &replay_options.start_offset)) {
      LOG(INFO) << "Unable to load replay start offset! Using default instead";
      replay_options.start_offset = 0.0;
    }
    Streaming::enableReplay(replay_options);
  }

  // Bind streamer pipelines
  Streaming::bindLogWithInput();

  // Set data callback
  for (size_t i = 0; i < streamings_.size(); i++) {
    Streaming::DataCallback callback = std::bind(&StreamHandle::dataCallback, 
      this, std::placeholders::_1, std::placeholders::_2);
    streamings_[i]->setDataCallback(callback);
  }

  // Initialize data handles
  gnss_local_ = std::make_shared<DataCluster::GNSS>();
  gnss_local_->init();

  // Start streamings
  for (size_t i = 0; i < streamings_.size(); i++) {
    streamings_[i]->start();
  }
}

StreamHandle::~StreamHandle()
{
  // Free handles
  gnss_local_->free();

  // Stop streamings
  for (size_t i = 0; i < streamings_.size(); i++) {
    streamings_[i]->stop();
  }
}

// Handle GNSS data
void StreamHandle::handleGNSS(const std::string& tag, 
                              const std::shared_ptr<DataCluster::GNSS>& gnss)
{
  // Get role
  if (behaviors_.find(tag) == behaviors_.end()) {
    LOG(ERROR) << "Formator tag " << tag << " not registored!";
    return;
  }
  std::vector<GnssRole> roles;
  GnssRole role_out;
  for (size_t i = 0; i < behaviors_.at(tag).role.size(); i++) {
    roles.push_back(GnssRole());
    option_tools::convert(behaviors_.at(tag).role[i], roles[i]);
    if (roles[i] == GnssRole::Rover || roles[i] == GnssRole::Reference ||
        roles[i] == GnssRole::Heading) role_out = roles[i]; 
  }

  // Update ephemeris and corrections
  for (auto it : gnss->types) {
    if (it == GnssDataType::Ephemeris) {
      bool found = false;
      for (auto it_role : roles) {
        if (it_role == GnssRole::Ephemeris) 
        { found = true; break; }
      }
      if (!found) continue;
      memcpy(gnss_local_->ephemeris->eph, 
        gnss->ephemeris->eph, sizeof(eph_t) * 2 * MAXSAT);
      memcpy(gnss_local_->ephemeris->geph, 
        gnss->ephemeris->geph, sizeof(geph_t) * 2 * NSATGLO);
    }

    if (it == GnssDataType::SSR) {
      bool found = false;
      for (auto it_role : roles) {
        if (it_role == GnssRole::Correction) 
        { found = true; break; }
      }
      if (!found) continue;
      gnss_common::updateSSR(gnss->ephemeris->ssr, gnss_local_);
    }

    if (it == GnssDataType::AntePos) {
      bool found = false;
      for (auto it_role : roles) {
        if (it_role == GnssRole::Reference) 
        { found = true; break; }
      }
      if (!found) continue;
      gnss_common::updateAntennaPosition(gnss->antenna, gnss_local_);
    }

    if (it == GnssDataType::IonPara) {
      bool found = false;
      for (auto it_role : roles) {
        if (it_role == GnssRole::Ephemeris) 
        { found = true; break; }
      }
      if (!found) continue;
      gnss_common::updateIonAndUTC(gnss->ephemeris, gnss_local_);
    }
  }
  
  // Find observation message
  bool has_observation = false;
  for (auto it : gnss->types) 
    if (it == GnssDataType::Observation) has_observation = true;
  if (!has_observation) return;

  // Set to epoch data
  GnssMeasurement epoch;
  epoch.timestamp = gnss_common::gpsTimeToUtc(
    gnss_common::gtimeToDouble(gnss->observation->data[0].time));
  epoch.role = role_out;
  epoch.tag = tag;
  epoch.position.setZero();
  double *rs, *dts, *var;
  double *rs_ssr, *dts_ssr, *var_ssr;
  auto& obs = gnss->observation;
  auto& nav = gnss_local_->ephemeris;
  int svh[MAXOBS], svh_ssr[MAXOBS], n = obs->n;
  rs = mat(6, n); dts = mat(2, n); var = mat(1, n);
  rs_ssr = mat(6, n); dts_ssr = mat(2, n); var_ssr = mat(1, n);
  satposs(obs->data[0].time, obs->data, n,
      nav, EPHOPT_BRDC, rs, dts, var, svh);
  satposs(obs->data[0].time, obs->data, n,
      nav, EPHOPT_SSRAPC, rs_ssr, dts_ssr, var_ssr, svh_ssr);
  for (int i = 0; i < n; i++) {
    Satellite satellite;

    // system and prn
    int prn = 0;
    char strprnnum[3];
    switch (satsys(obs->data[i].sat, &prn)) {
      case SYS_GPS: satellite.prn = 'G'; break;
      case SYS_GLO: satellite.prn = 'R'; break;
      case SYS_GAL: satellite.prn = 'E'; break;
      case SYS_CMP: satellite.prn = 'C'; break;
      default: continue;
    }
    sprintf(strprnnum, "%02d", prn);
    satellite.prn.append(strprnnum);

    // satellite position and clock
    if (svh_ssr[i] != -1 && rs_ssr[i * 6] != 0 && dts_ssr[i * 2] != 0) {
      satellite.sat_position = Eigen::Map<Eigen::Vector3d>(rs_ssr + i * 6);
      satellite.sat_velocity = Eigen::Map<Eigen::Vector3d>(rs_ssr + 3 + i * 6);
      satellite.sat_clock = dts_ssr[i * 2] * CLIGHT;
      satellite.sat_frequency = dts_ssr[1 + i * 2] * CLIGHT;
      satellite.sat_type = SatEphType::Precise;
    }
    else if (svh[i] != -1) {
      satellite.sat_position = Eigen::Map<Eigen::Vector3d>(rs + i * 6);
      satellite.sat_velocity = Eigen::Map<Eigen::Vector3d>(rs + 3 + i * 6);
      satellite.sat_clock = dts[i * 2] * CLIGHT;
      satellite.sat_frequency = dts[1 + i * 2] * CLIGHT;
      satellite.sat_type = SatEphType::Broadcast;
    }
    else continue;

    // ionosphere
    satellite.ionosphere = 0.0;

    // observations
    for (int j = 0; j < NFREQ + NEXOBS; j++) {
      if (obs->data[i].P[j] == 0.0) continue;
      Observation observation;
      int freq_index;
      int code_type = obs->data[i].code[j];
      double freq = sat2freq(obs->data[i].sat, code_type, nav);
      observation.wavelength = CLIGHT / freq;
      observation.pseudorange = obs->data[i].P[j];
      observation.phaserange = obs->data[i].L[j] * observation.wavelength;
      observation.doppler = -obs->data[i].D[j] * observation.wavelength;
      observation.SNR = obs->data[i].SNR[j] * 1.0e-3;
      observation.LLI = obs->data[i].LLI[j];
      observation.slip = false;
      satellite.observations.insert(std::make_pair(code_type, observation));
    }

    // TODO: Read files to get DCBs

    // Biases
    for (int j = 0; j < MAXCODE; j++) {
      // code bias
      double cbias = nav->ssr[obs->data[i].sat - 1].cbias[j];
      if (cbias != 0.0) {
        // relative to a common reference
        std::pair<int, int> code_pair = std::make_pair(0, i + 1);
        satellite.DCBs.insert(std::make_pair(code_pair, cbias));
      }

      // phase bias
      double pbias = nav->ssr[obs->data[i].sat - 1].pbias[j];
      if (pbias != 0.0) {
        std::pair<int, int> code_pair = std::make_pair(0, i + 1);
        satellite.DPBs.insert(std::make_pair(code_pair, pbias));
      }
    }

    // Time Group Delay (TGD) and Inter-System Corrections (ISC)
    // we convert them to DCBs 
    // if (satellite.getSystem() == 'R') {
    //   for (int j = 0; j < nav->ng; j++) {
    //     if (nav->geph[j].sat == obs->data[i].sat) {
    //       satellite.TGDs[0] = -nav->geph[j].dtaun * CLIGHT;
    //     }
    //   }
    // }
    // else {
    //   for (int j = 0; j < nav->n; j++) {
    //     if (nav->eph[j].sat == obs->data[i].sat) {
    //       for (int k = 0; k < 6; k++) {
    //         satellite.TGDs[k] = nav->eph[j].tgd[k] * CLIGHT;
    //       }
    //     }
    //   }
    // }
    
    epoch.satellites.insert(std::make_pair(satellite.prn, satellite));
  }
  free(rs); free(dts); free(var);
  free(rs_ssr); free(dts_ssr); free(var_ssr);

  // reference station position
  if (role_out == GnssRole::Reference) {
    if (gnss_local_->antenna->pos[0] == 0.0) {
      LOG(WARNING) << "Unable to get antenna position of reference station!";
      return;
    }
    epoch.position = 
      Eigen::Map<Eigen::Vector3d>(gnss_local_->antenna->pos);
  }

  // GPS ionosphere parameters
  epoch.ionosphere_parameters = 
    Eigen::Map<Eigen::VectorXd>(gnss_local_->ephemeris->ion_gps, 8);

  // Troposphere
  epoch.troposphere_wet = 0.0;

  // Delete duplicated phase observations
  gnss_common::deleteDuplicatePhases(epoch);

  // Call GNSS observation processor
  for (auto it_gnss_callback : gnss_callbacks_) {
    std::vector<std::string>& tags = it_gnss_callback.second;
    GnssCallback& callback = it_gnss_callback.first;
    auto it_tag = std::find(tags.begin(), tags.end(), tag);
    if (it_tag == tags.end()) continue;
    callback(epoch);
  }
}

// Handle IMU data
void StreamHandle::handleIMU(const std::string& tag, 
                             const std::shared_ptr<DataCluster::IMU>& imu)
{
  // Get role
  if (behaviors_.find(tag) == behaviors_.end()) {
    LOG(ERROR) << "Formator tag " << tag << " not registored!";
    return;
  }
  std::vector<ImuRole> roles;
  bool has_major = false;
  for (size_t i = 0; i < behaviors_.at(tag).role.size(); i++) {
    roles.push_back(ImuRole());
    option_tools::convert(behaviors_.at(tag).role[i], roles[i]);
    if (roles[i] == ImuRole::Minor) {
      LOG(WARNING) << "We do not support multiple IMUs currently!";
      return;
    }
    if (roles[i] == ImuRole::Major) has_major = true;
  }
  if (roles.size() > 1) {
    LOG(ERROR) << "A IMU should has only one role!";
  }
  else if (roles.size() == 0) {
    LOG(ERROR) << "No role for current IMU was specified!";
    return;
  }

  // Convert IMU data
  ImuMeasurement epoch(imu->time, 
    Eigen::Map<Eigen::Vector3d>(imu->angular_velocity), 
    Eigen::Map<Eigen::Vector3d>(imu->acceleration));

  // Call INS processor
  for (auto it_imu_callback : imu_callbacks_) {
    std::vector<std::string>& tags = it_imu_callback.second;
    ImuCallback& callback = it_imu_callback.first;
    auto it_tag = std::find(tags.begin(), tags.end(), tag);
    if (it_tag == tags.end()) continue;
    callback(tag, roles[0], epoch);
  }
}

// Handle Image data
void StreamHandle::handleImage(const std::string& tag, 
                             const std::shared_ptr<DataCluster::Image>& image)
{
  // Get role
  if (behaviors_.find(tag) == behaviors_.end()) {
    LOG(ERROR) << "Formator tag " << tag << " not registored!";
    return;
  }
  std::vector<CameraRole> roles;
  bool has_mono = false;
  for (size_t i = 0; i < behaviors_.at(tag).role.size(); i++) {
    roles.push_back(CameraRole());
    option_tools::convert(behaviors_.at(tag).role[i], roles[i]);
    if (roles[i] != CameraRole::Mono) {
      LOG(WARNING) << "We do not support multiple Cameras currently!";
      return;
    }
    else has_mono = true;
  }
  if (!has_mono) return;
  if (roles.size() > 1) {
    LOG(ERROR) << "A camera should has only one role!";
  }
  else if (roles.size() == 0) {
    LOG(ERROR) << "No role for current camera was specified!";
    return;
  }

  // Convert Image data
  cv::Mat image_mat(image->height, image->width, CV_8UC1, image->image);

  // Call Image processor
  for (auto it_image_callback : image_callbacks_) {
    std::vector<std::string>& tags = it_image_callback.second;
    ImageCallback& callback = it_image_callback.first;
    auto it_tag = std::find(tags.begin(), tags.end(), tag);
    if (it_tag == tags.end()) continue;
    callback(image->time, tag, roles[0], image_mat);
  }
}

// Data callback
void StreamHandle::dataCallback(
    const std::string& tag, const std::shared_ptr<DataCluster>& data)
{
  // GNSS data
  if (data->gnss && gnss_callbacks_.size() > 0) {
    mutex_gnss_.lock();
    handleGNSS(tag, data->gnss);
    mutex_gnss_.unlock();
  }

  // IMU data
  if (data->imu && imu_callbacks_.size() > 0) {
    mutex_imu_.lock();
    handleIMU(tag, data->imu);
    mutex_imu_.unlock();
  }

  // Image data
  if (data->image && image_callbacks_.size() > 0) {
    mutex_image_.lock();
    handleImage(tag, data->image);
    mutex_image_.unlock();
  }

  // Option data
  if (data->option) {

  }
}

}