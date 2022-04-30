/**
* @Function: Stream functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/stream/formator.h"

#include <mutex>
#include <glog/logging.h>
#include <vikit/timer.h>

#include "gici/gnss/gnss_common.h"

namespace gici {

DataFormat::DataFormat(FormatorType type)
{
  if (type == FormatorType::RTCM2 || type == FormatorType::RTCM3 ||
    type == FormatorType::GnssRaw) {
    gnss = std::make_shared<GNSS>();
    gnss->init();
    return;
  }
  if (type == FormatorType::IMUPack) {
    imu = std::make_shared<IMU>();
    return;
  }
  if (type == FormatorType::OptionPack) {
    option = std::make_shared<Option>();
    return;
  }
  if (type == FormatorType::NMEA) {
    solution = std::make_shared<Solution>();
    return;
  }
  if (type == FormatorType::ImagePack || type == FormatorType::ImageV4L2) {
    LOG(FATAL) << "Cannot initialize DataFormat::Image: "
           << "Image length should be given!";
  }
  LOG(FATAL) << "Cannot initialize: Data format not recognized!";
}

DataFormat::DataFormat(FormatorType type, int _width, int _height)
{
  if (type == FormatorType::ImagePack || type == FormatorType::ImageV4L2) {
    image = std::make_shared<Image>();
    image->init(_width, _height);
    return;
  }
  LOG(FATAL) << "Cannot initialize: Data format not recognized!";
}

DataFormat::~DataFormat()
{
  if (gnss != nullptr) gnss->free();
  if (image != nullptr) image->free();
}

void DataFormat::GNSS::init()
{
  if (!(observation = (obs_t *)malloc(sizeof(obs_t))) ||
    !(observation->data = (obsd_t *)malloc(sizeof(obsd_t) * MAXOBS)) ||
    !(ephemeris = (nav_t *)malloc(sizeof(nav_t))) ||
    !(ephemeris->eph = (eph_t  *)malloc(sizeof(eph_t) * MAXSAT * 2)) ||
    !(ephemeris->geph = (geph_t *)malloc(sizeof(geph_t) * NSATGLO * 2)) ||
    !(antenna = (sta_t *)malloc(sizeof(sta_t))) ) {
    free();
  }
  eph_t  eph0 = {0, -1, -1};
  geph_t geph0 = {0, -1};
  for (int i = 0; i < MAXSAT *2; i++) ephemeris->eph[i] = eph0;
  for (int i = 0; i < NSATGLO * 2 ; i++) ephemeris->geph[i] = geph0;
  ephemeris->n = MAXSAT * 2;
  ephemeris->ng = NSATGLO * 2;
  memset(antenna, 0, sizeof(sta_t));
}

void DataFormat::GNSS::free()
{
  ::free(observation->data);
  ::free(observation); observation = NULL;
  ::free(ephemeris->eph); ::free(ephemeris->geph);
  ::free(ephemeris); ephemeris = NULL;
  ::free(antenna); antenna = NULL;
}

void DataFormat::Image::init(int _width, int _height)
{
  width = _width;
  height = _height;
  if (!(image = (uint8_t *)malloc(sizeof(uint8_t) * width * height)))
    free();
}

void DataFormat::Image::free()
{
  ::free(image);
}

namespace gnss_common {

// Update observation data
extern void updateObservation(
  obs_t *obs, DataFormat::GNSS::Ptr& gnss_data)
{
    int n = 0;
    for (int i = 0; i < obs->n; i++) {
      gnss_data->observation[0].data[n++] = obs->data[i];
    }
    gnss_data->observation[0].n = n;
    sortobs(&gnss_data->observation[0]);

    //   int n = 0;
    // for (int i = 0; i < obs->n; i++) {
    //   gnss_data[iobs]->observation[0].data[n++] = obs->data[i];
    // }
    // gnss_data[iobs]->observation[0].n = n;
    // sortobs(&gnss_data[iobs]->observation[0]);
}

// Update ephemeris
extern void updateEphemeris(
  nav_t *nav, int sat, DataFormat::GNSS::Ptr& gnss_data)
{
  eph_t *eph1, *eph2, *eph3;
  geph_t *geph1, *geph2, *geph3;
  int prn;
  if (satsys(sat, &prn) != SYS_GLO) {
    eph1 = nav->eph + sat - 1;
    eph2 = gnss_data->ephemeris->eph + sat - 1;
    eph3 = gnss_data->ephemeris->eph + sat - 1 + MAXSAT;
    if (eph2->ttr.time == 0 ||
      (eph1->iode != eph3->iode && eph1->iode != eph2->iode) ||
      (timediff(eph1->toe, eph3->toe) !=0.0 &&
        timediff(eph1->toe, eph2->toe) != 0.0)||
      (timediff(eph1->toc, eph3->toc) != 0.0 &&
        timediff(eph1->toc, eph2->toc) != 0.0)) {
      *eph3 = *eph2;
      *eph2 = *eph1;
    }
  }
  else {
    geph1 = nav->geph + prn - 1;
    geph2 = gnss_data->ephemeris->geph + prn - 1;
    geph3 = gnss_data->ephemeris->geph + prn - 1 + MAXPRNGLO;
    if (geph2->tof.time == 0 ||
      (geph1->iode != geph3->iode && geph1->iode != geph2->iode)) {
      *geph3 = *geph2;
      *geph2 = *geph1;
    }
  }
}

// Update ion/utc parameters
extern void updateIonAndUTC(
  nav_t *nav, DataFormat::GNSS::Ptr& gnss_data)
{
  matcpy(gnss_data->ephemeris->utc_gps, nav->utc_gps,8,1);
  matcpy(gnss_data->ephemeris->utc_glo, nav->utc_glo,8,1);
  matcpy(gnss_data->ephemeris->utc_gal, nav->utc_gal,8,1);
  matcpy(gnss_data->ephemeris->utc_qzs, nav->utc_qzs,8,1);
  matcpy(gnss_data->ephemeris->utc_cmp, nav->utc_cmp,8,1);
  matcpy(gnss_data->ephemeris->utc_irn, nav->utc_irn,9,1);
  matcpy(gnss_data->ephemeris->utc_sbs, nav->utc_sbs,4,1);
  matcpy(gnss_data->ephemeris->ion_gps, nav->ion_gps,8,1);
  matcpy(gnss_data->ephemeris->ion_gal, nav->ion_gal,4,1);
  matcpy(gnss_data->ephemeris->ion_qzs, nav->ion_qzs,8,1);
  matcpy(gnss_data->ephemeris->ion_cmp, nav->ion_cmp,8,1);
  matcpy(gnss_data->ephemeris->ion_irn, nav->ion_irn,8,1);
}

// Update antenna position
extern void updateAntennaPosition(
  sta_t *sta, DataFormat::GNSS::Ptr& gnss_data)
{
  if (sta == NULL) {
    LOG(ERROR) << "Antenna position parameter has NULL pointer!";
    return;
  }
  for (int i = 0; i < 3; i++) {
    gnss_data->antenna->pos[i] = sta->pos[i];
  }
  double pos[3], del[3] = {0}, dr[3];
  ecef2pos(sta->pos, pos);
  if (sta->deltype == 1) { // ECEF
    del[2] = sta->hgt;
    enu2ecef(pos, del, dr);
    for (int i = 0; i < 3; i++) {
      gnss_data->antenna->pos[i] += sta->del[i] + dr[i];
    }
  }
  else {  // ENU
    enu2ecef(pos, sta->del, dr);
    for (int i = 0; i < 3; i++) {
      gnss_data->antenna->pos[i] += dr[i];
    }
  }
}

// Update ssr corrections
extern void updateSSR(
  ssr_t *ssr, DataFormat::GNSS::Ptr& gnss_data)
{
  if (ssr == NULL) {
    LOG(ERROR) << "SSR parameter has NULL pointer!";
    return;
  }
  for (int i = 0; i < MAXSAT; i++) {
    if (!ssr[i].update) continue;

    // check consistency between iods of orbit and clock
    if (ssr[i].iod[0] != ssr[i].iod[1]) continue;

    ssr[i].update = 0;

    // TODO: check corresponding ephemeris exists
    gnss_data->ephemeris->ssr[i] = ssr[i];
    gnss_data->ephemeris->ssr[i].update = 1;
  }
}

// Select data from GNSS stream
extern void updateStreamData(int ret, obs_t *obs, nav_t *nav, 
  sta_t *sta, ssr_t *ssr, int iobs, int sat, 
  std::vector<DataFormat::GNSS::Ptr>& gnss_data)
{
  GnssDataType type = static_cast<GnssDataType>(ret);
  // Observation data
  if (type == GnssDataType::Observation) {
    updateObservation(obs, gnss_data[iobs]);
  }
  // Ephemeris data
  else if (type == GnssDataType::Ephemeris) {
    updateEphemeris(nav, sat, gnss_data[0]);
  }
  // Ionosphere parameters
  else if (type == GnssDataType::IonPara) {
    updateIonAndUTC(nav, gnss_data[0]);
  }
  // Antenna position parameters
  else if (type == GnssDataType::AntePos) {
    updateAntennaPosition(sta, gnss_data[0]);
  }
  // SSR (precise ephemeris, DCBs, etc..)
  else if (type == GnssDataType::SSR) {
    updateSSR(ssr, gnss_data[0]);
  }
}

}

// Load option with info
#define LOAD_COMMON(opt) \
  if (!option_tools::safeGet(node, #opt, &config.opt)) { \
  LOG(INFO) << __FUNCTION__ << ": Unable to load " << #opt \
         << ". Using default instead."; }
// Load option with fatal error
#define LOAD_REQUIRED(opt) \
  if (!option_tools::safeGet(node, #opt, &config.opt)) { \
  LOG(FATAL) << __FUNCTION__ << ": Unable to load " << #opt << "!"; }

// RTCM 2 ----------------------------------------------------
RTCM2Formator::RTCM2Formator(Config& config)
{
  type_ = FormatorType::RTCM2;

  memset(&rtcm_, 0, sizeof(rtcm_t));
  init_rtcm(&rtcm_);
  rtcm_.time = gnss_common::doubleToGtime(config.start_time);
  for (int i = 0; i < MaxDataSize::RTCM2; i++) {
    data_.push_back(std::make_shared<DataFormat>(type_));
  }
}

RTCM2Formator::RTCM2Formator(YAML::Node& node)
{
  Config config;
  config.start_time = vk::Timer::getCurrentTime();
  LOAD_COMMON(start_time);

  type_ = FormatorType::RTCM2;

  memset(&rtcm_, 0, sizeof(rtcm_t));
  init_rtcm(&rtcm_);
  rtcm_.time = gnss_common::doubleToGtime(config.start_time);
  for (int i = 0; i < MaxDataSize::RTCM2; i++) {
    data_.push_back(std::make_shared<DataFormat>(type_));
  }
}

RTCM2Formator::~RTCM2Formator()
{
  free_rtcm(&rtcm_);
}

// Decode stream to data
int RTCM2Formator::decode(const uint8_t *buf, int size, 
    std::vector<DataFormat::Ptr>& data)
{
  // Clear old informations and get GNSS data handle
  std::vector<DataFormat::GNSS::Ptr> gnss_data;
  for (size_t i = 0; i < data_.size(); i++) {
    data_[i]->gnss->types.clear();
    gnss_data.push_back(data_[i]->gnss);
  }

  bool is_observation = false;
  bool is_others = false;
  int iobs = 0;
  for (int i = 0; i < size; i++) {
    int ret = input_rtcm2(&rtcm_, buf[i]);
    if (ret <= 0) continue;

    obs_t *obs = &rtcm_.obs;
    nav_t *nav = &rtcm_.nav;
    sta_t *sta = &rtcm_.sta;
    ssr_t *ssr = rtcm_.ssr;
    int sat = rtcm_.ephsat;
    gnss_common::updateStreamData(
        ret, obs, nav, sta, ssr, iobs, sat, gnss_data);
    GnssDataType type = static_cast<GnssDataType>(ret);
    DataFormat::GNSS::Ptr& gnss = 
      type == GnssDataType::Observation ? gnss_data[iobs] : gnss_data[0];
    if (std::find(gnss->types.begin(), gnss->types.end(), type)
      == gnss->types.end()) {
      gnss->types.push_back(type);
    }
    
    if (type == GnssDataType::Observation) {
      if (iobs < MaxDataSize::RTCM2) iobs++;
      if (iobs >= MaxDataSize::RTCM2) {
        LOG(WARNING) << "Max data length surpassed!";
        break;
      }
      is_observation = true;
    }
    else is_others = true;
  }

  data = data_;

  return is_observation ? iobs : is_others;
}

// Encode data to stream
int RTCM2Formator::encode(const DataFormat::Ptr& data, uint8_t *buf)
{
  LOG(ERROR) << "RTCM2 Encoding not supported!";
  return 0;
}

// RTCM 3 -------------------------------------------------
RTCM3Formator::RTCM3Formator(Config& config)
{
  type_ = FormatorType::RTCM3;

  memset(&rtcm_, 0, sizeof(rtcm_t));
  init_rtcm(&rtcm_);
  rtcm_.time = gnss_common::doubleToGtime(config.start_time);
  for (int i = 0; i < MaxDataSize::RTCM3; i++) {
    data_.push_back(std::make_shared<DataFormat>(type_));
  }
}

RTCM3Formator::RTCM3Formator(YAML::Node& node)
{
  Config config;
  config.start_time = vk::Timer::getCurrentTime();
  LOAD_COMMON(start_time);

  type_ = FormatorType::RTCM3;

  memset(&rtcm_, 0, sizeof(rtcm_t));
  init_rtcm(&rtcm_);
  rtcm_.time = gnss_common::doubleToGtime(config.start_time);
  for (int i = 0; i < MaxDataSize::RTCM3; i++) {
    data_.push_back(std::make_shared<DataFormat>(type_));
  }
}

RTCM3Formator::~RTCM3Formator()
{
  free_rtcm(&rtcm_);
}

// Decode stream to data
int RTCM3Formator::decode(const uint8_t *buf, int size, 
    std::vector<DataFormat::Ptr>& data)
{
  // Clear old informations and get GNSS data handle
  std::vector<DataFormat::GNSS::Ptr> gnss_data;
  for (size_t i = 0; i < data_.size(); i++) {
    data_[i]->gnss->types.clear();
    gnss_data.push_back(data_[i]->gnss);
  }

  bool is_observation = false;
  bool is_others = false;
  int iobs = 0;
  for (int i = 0; i < size; i++) {
    int ret = input_rtcm3(&rtcm_, buf[i]);
    if (ret <= 0) continue;

    obs_t *obs = &rtcm_.obs;
    nav_t *nav = &rtcm_.nav;
    sta_t *sta = &rtcm_.sta;
    ssr_t *ssr = rtcm_.ssr;
    int sat = rtcm_.ephsat;
    gnss_data[1]->observation[0].data[0].code[0] = 1;
    gnss_common::updateStreamData(
        ret, obs, nav, sta, ssr, iobs, sat, gnss_data);
    GnssDataType type = static_cast<GnssDataType>(ret);
    DataFormat::GNSS::Ptr& gnss = 
      type == GnssDataType::Observation ? gnss_data[iobs] : gnss_data[0];
    if (std::find(gnss->types.begin(), gnss->types.end(), type)
      == gnss->types.end()) {
      gnss->types.push_back(type);
    }
    
    if (type == GnssDataType::Observation) {
      if (iobs < MaxDataSize::RTCM3) iobs++;
      if (iobs >= MaxDataSize::RTCM3) {
        LOG(WARNING) << "Max data length surpassed!";
        break;
      }
      is_observation = true;
    }
    else is_others = true;
  }

  data = data_;

  return is_observation ? iobs : is_others;
}

// Encode data to stream
int RTCM3Formator::encode(
  const DataFormat::Ptr& data, uint8_t *buf)
{
  // Check the control structure
  std::map<GnssDataType, bool> type_valid;
  type_valid.insert(std::make_pair(GnssDataType::Observation, false));
  type_valid.insert(std::make_pair(GnssDataType::Ephemeris, false));
  type_valid.insert(std::make_pair(GnssDataType::AntePos, false));
  for (auto it : data->gnss->types) {
    type_valid.at(it) = true;
  }

  // Encode data
  std::vector<int> msg_obs = {1077, 1087, 1097, 1127};
  std::vector<int> msg_eph = {1019, 1020, 1045, 1046, 1042};
  std::vector<int> msg_ant = {1005};
  int n = 0;
  rtcm_t rtcm = rtcm_;
  memcpy(&rtcm.obs, data->gnss->observation, sizeof(obs_t));
  memcpy(&rtcm.nav, data->gnss->ephemeris, sizeof(nav_t));
  memcpy(&rtcm.sta, data->gnss->antenna, sizeof(sta_t));
  memcpy(rtcm.ssr, data->gnss->ephemeris->ssr, sizeof(ssr_t) * MAXSAT);

  if (type_valid.at(GnssDataType::Observation)) {
    // Set time
    rtcm.time = rtcm.obs.data[0].time;
    if (fabs(timediff(rtcm.time, rtcm_.time)) > 30.0) rtcm_.time = rtcm.time;

    for (size_t i = 0; i < msg_obs.size(); i++) {
      gen_rtcm3(&rtcm, msg_obs[i], 0, i != 3);
      memcpy(buf+n, rtcm.buff, rtcm.nbyte);
      n += rtcm.nbyte;
    }
  }
  if (type_valid.at(GnssDataType::Ephemeris)) {
    for (size_t i = 0; i < msg_eph.size(); i++) {
      gen_rtcm3(&rtcm, msg_eph[i], 0, i != 4);
      memcpy(buf+n, rtcm.buff, rtcm.nbyte);
      n += rtcm.nbyte;
    }
  }
  if (type_valid.at(GnssDataType::AntePos)) {
    for (size_t i = 0; i < msg_ant.size(); i++) {
      gen_rtcm3(&rtcm, msg_ant[i], 0, i != 3);
      memcpy(buf+n, rtcm.buff, rtcm.nbyte);
      n += rtcm.nbyte;
    }
  }

  return n;
}

// GNSS raw --------------------------------------------------------
GnssRawFormator::GnssRawFormator(Config& config)
{
  type_ = FormatorType::GnssRaw;
  option_tools::convert(config.sub_type, format_);

  init_raw(&raw_, static_cast<int>(format_));
  raw_.time = gnss_common::doubleToGtime(config.start_time);
  for (int i = 0; i < MaxDataSize::GnssRaw; i++) {
    data_.push_back(std::make_shared<DataFormat>(type_));
  }
}
  
GnssRawFormator::GnssRawFormator(YAML::Node& node)
{
  Config config;
  config.start_time = vk::Timer::getCurrentTime();
  LOAD_COMMON(start_time);
  LOAD_REQUIRED(sub_type);

  type_ = FormatorType::GnssRaw;
  option_tools::convert(config.sub_type, format_);

  init_raw(&raw_, static_cast<int>(format_));
  raw_.time = gnss_common::doubleToGtime(config.start_time);
  for (int i = 0; i < MaxDataSize::GnssRaw; i++) {
    data_.push_back(std::make_shared<DataFormat>(type_));
  }
} 

GnssRawFormator::~GnssRawFormator()
{
  free_raw(&raw_);
}

// Decode stream to data
int GnssRawFormator::decode(const uint8_t *buf, int size, 
    std::vector<DataFormat::Ptr>& data)
{
  // Clear old informations and get GNSS data handle
  std::vector<DataFormat::GNSS::Ptr> gnss_data;
  for (size_t i = 0; i < data_.size(); i++) {
    data_[i]->gnss->types.clear();
    gnss_data.push_back(data_[i]->gnss);
  }

  bool is_observation = false;
  bool is_others = false;
  int iobs = 0;
  for (int i = 0; i < size; i++) {
    int ret = input_raw(&raw_, static_cast<int>(format_), buf[i]);
    if (ret <= 0) continue;

    obs_t *obs = &raw_.obs;
    nav_t *nav = &raw_.nav;
    sta_t *sta = &raw_.sta;
    int sat = raw_.ephsat;
    gnss_common::updateStreamData(
        ret, obs, nav, sta, NULL, iobs, sat, gnss_data);
    GnssDataType type = static_cast<GnssDataType>(ret);
    DataFormat::GNSS::Ptr& gnss = 
      type == GnssDataType::Observation ? gnss_data[iobs] : gnss_data[0];
    if (std::find(gnss->types.begin(), gnss->types.end(), type)
      == gnss->types.end()) {
      gnss->types.push_back(type);
    }
    
    if (type == GnssDataType::Observation) {
      if (iobs < MaxDataSize::GnssRaw) iobs++;
      if (iobs >= MaxDataSize::GnssRaw) {
        LOG(WARNING) << "Max data length surpassed!";
        break;
      }
      is_observation = true;
    }
    else is_others = true;
  }

  data = data_;

  return is_observation ? iobs : is_others;
}

// Encode data to stream
int GnssRawFormator::encode(const DataFormat::Ptr& data, uint8_t *buf)
{
  LOG(ERROR) << "GNSS-Raw Encoding not supported!";
  return 0;
}

// Image V4L2 ------------------------------------------------
ImageV4L2Formator::ImageV4L2Formator(Config& config)
{
  type_ = FormatorType::ImageV4L2;

  init_img(&image_, config.width, config.height);
  data_.push_back(std::make_shared<DataFormat>(
    FormatorType::ImageV4L2, config.width, config.height));
}

ImageV4L2Formator::ImageV4L2Formator(YAML::Node& node)
{
  Config config;
  LOAD_REQUIRED(width);
  LOAD_REQUIRED(height);

  type_ = FormatorType::ImageV4L2;

  init_img(&image_, config.width, config.height);
  data_.push_back(std::make_shared<DataFormat>(
    FormatorType::ImageV4L2, config.width, config.height));
}

ImageV4L2Formator::~ImageV4L2Formator()
{
  free_img(&image_);
}

// Decode stream to data
int ImageV4L2Formator::decode(const uint8_t *buf, int size, 
    std::vector<DataFormat::Ptr>& data)
{
  int ret = input_image_v4l2(&image_, buf, size);
  if (ret <= 0) return 0;

  memcpy(data_[0]->image->image, image_.image, 
    sizeof(uint8_t) * image_.width * image_.height);
  data = data_;

  return 1;
}

// Encode data to stream
int ImageV4L2Formator::encode(const DataFormat::Ptr& data, uint8_t *buf)
{
  LOG(ERROR) << "Image-V4L2 Encoding not supported!";
  return 0;
}
  
// Image pack -------------------------------------------------
ImagePackFormator::ImagePackFormator(Config& config)
{
  type_ = FormatorType::ImagePack;

  init_img(&image_, config.width, config.height);
  for (int i = 0; i < MaxDataSize::ImagePack; i++) {
    data_.push_back(std::make_shared<DataFormat>(
      FormatorType::ImagePack, config.width, config.height));
  }
}

ImagePackFormator::ImagePackFormator(YAML::Node& node)
{
  Config config;
  LOAD_REQUIRED(width);
  LOAD_REQUIRED(height);

  type_ = FormatorType::ImagePack;

  init_img(&image_, config.width, config.height);
  for (int i = 0; i < MaxDataSize::ImagePack; i++) {
    data_.push_back(std::make_shared<DataFormat>(
      FormatorType::ImagePack, config.width, config.height));
  }
}

ImagePackFormator::~ImagePackFormator()
{
  free_img(&image_);
}

// Decode stream to data
int ImagePackFormator::decode(const uint8_t *buf, int size, 
    std::vector<DataFormat::Ptr>& data)
{
  int iobs = 0;
  for (int i = 0; i < size; i++) {
    int ret = input_image(&image_, buf[i]);
    if (ret <= 0) continue;

    data_[iobs]->image->time = gnss_common::gtimeToDouble(image_.time);
    // TODO: This memcpy may increase memory occupation, according to
    // https://bbs.csdn.net/topics/390705325, maybe it is caused by the
    // convertion between physical and vitural memory, and this is not 
    // a memory leak.
    memcpy(data_[iobs]->image->image, image_.image, 
      sizeof(uint8_t) * image_.width * image_.height);

    if (++iobs >= MaxDataSize::ImagePack) {
      LOG(WARNING) << "Max data length surpassed!";
      break;
    }
  }

  data = data_;

  return iobs;
}

// Encode data to stream
int ImagePackFormator::encode(
    const DataFormat::Ptr& data, uint8_t *buf)
{
  img_t *image;
  init_img(image, data->image->width, data->image->height);
  image->time = gnss_common::doubleToGtime(data->image->time);
  memcpy(image->image, data->image->image, 
       data->image->width * data->image->height);

  if (!gen_img(image)) return 0;

  memcpy(buf, image->buff, image->nbyte);
  int nbyte = image->nbyte;
  free_img(image);

  return nbyte;
}

// IMU pack --------------------------------------------------
IMUPackFormator::IMUPackFormator(Config& config)
{
  type_ = FormatorType::IMUPack;

  init_imu(&imu_);
}

IMUPackFormator::IMUPackFormator(YAML::Node& node)
{
  type_ = FormatorType::IMUPack;

  init_imu(&imu_);
}

IMUPackFormator::~IMUPackFormator()
{
  free_imu(&imu_);
}

// Decode stream to data
int IMUPackFormator::decode(const uint8_t *buf, int size, 
    std::vector<DataFormat::Ptr>& data)
{
  int n_data = 0;
  data.clear();
  for (int i = 0; i < size; i++) {
    int ret = input_imu(&imu_, buf[i]);
    if (ret <= 0) continue;

    DataFormat::Ptr data_ptr;
    data_ptr = std::make_shared<DataFormat>(FormatorType::IMUPack);
    data_ptr->imu->time = gnss_common::gtimeToDouble(imu_.time);
    for (int k = 0; k < 3; k++) {
      data_ptr->imu->acceleration[k] = imu_.acc[k];
      data_ptr->imu->angular_velocity[k] = imu_.gyro[k];
    }
    data.push_back(data_ptr);

    if (++n_data >= MaxDataSize::IMUPack) {
      LOG(WARNING) << "Max data length surpassed!";
      break;
    }
  }

  return n_data;
}

// Encode data to stream
int IMUPackFormator::encode(const DataFormat::Ptr& data, uint8_t *buf)
{
  imu_t *imu;
  init_imu(imu);
  imu->time = gnss_common::doubleToGtime(data->imu->time);
  for (int i = 0; i < 3; i++) {
    imu->acc[i] = data->imu->acceleration[i];
    imu->gyro[i] = data->imu->angular_velocity[i];
  }

  if (!gen_imu(imu)) return 0;

  memcpy(buf, imu->buff, imu->nbyte);
  int nbyte = imu->nbyte;
  free_imu(imu);
  return imu->nbyte;
}

// Option pack --------------------------------------------------
OptionFormator::OptionFormator(Config& config)
{

}

OptionFormator::OptionFormator(YAML::Node& node)
{

}

OptionFormator::~OptionFormator()
{

}

// Decode stream to data
int OptionFormator::decode(const uint8_t *buf, int size, 
    std::vector<DataFormat::Ptr>& data)
{
  return 0;
}

// Encode data to stream
int OptionFormator::encode(const DataFormat::Ptr& data, uint8_t *buf)
{
  return 0;
}

// NMEA ----------------------------------------------------------
NmeaFormator::NmeaFormator(Config& config)
{
  type_ = FormatorType::NMEA;

  config_ = config;
}

NmeaFormator::NmeaFormator(YAML::Node& node)
{
  type_ = FormatorType::NMEA;

  Config config;
  LOAD_COMMON(use_gga);
  LOAD_COMMON(use_rmc);
  LOAD_COMMON(use_esa);
  LOAD_COMMON(talker_id);
  config_ = config;
}

NmeaFormator::~NmeaFormator()
{

}

// Decode stream to data
int NmeaFormator::decode(const uint8_t *buf, int size, 
    std::vector<DataFormat::Ptr>& data)
{
  LOG(ERROR) << "NMEA decoding not supported!";

  return 0;
}

// Encode data to stream
int NmeaFormator::encode(const DataFormat::Ptr& data, uint8_t *buf)
{
  if (data->solution == nullptr) return 0;

  uint8_t *p = buf;
  if (config_.use_gga) {
    p += encodeGGA(*data->solution, p);
  }
  if (config_.use_rmc) {
    p += encodeRMC(*data->solution, p);
  }
  if (config_.use_esa) {
    p += encodeESA(*data->solution, p);
  }

  return p - buf;
}

#define MAXFIELD   64           /* max number of fields in a record */
#define MAXNMEA    256          /* max length of nmea sentence */
#define KNOT2M     0.514444444  /* m/knot */
static const int nmea_solq[]={  /* NMEA GPS quality indicator */
    /* 0=Fix not available or invalidi */
    /* 1=GPS SPS Mode, fix valid */
    /* 2=Differential GPS, SPS Mode, fix valid */
    /* 3=GPS PPS Mode, fix valid */
    /* 4=Real Time Kinematic. System used in RTK mode with fixed integers */
    /* 5=Float RTK. Satellite system used in RTK mode, floating integers */
    /* 6=Estimated (dead reckoning) Mode */
    /* 7=Manual Input Mode */
    /* 8=Simulation Mode */
    SOLQ_NONE ,SOLQ_SINGLE, SOLQ_DGPS, SOLQ_PPP , SOLQ_FIX,
    SOLQ_FLOAT,SOLQ_DR    , SOLQ_NONE, SOLQ_NONE, SOLQ_NONE
};

// Encode GNGGA message
int NmeaFormator::encodeGGA(const Solution& solution, uint8_t* buf)
{
  sol_t sol;
  convertSolution(solution, sol);

  gtime_t time;
  double h,ep[6],pos[3],dms1[3],dms2[3],dop=1.0;
  int solq,refid=0;
  char *p=(char *)buf,*q,sum;
  
  if (sol.stat<=SOLQ_NONE) {
    p+=sprintf(p,"$%sGGA,,,,,,,,,,,,,,",config_.talker_id.data());
    for (q=(char *)buf+1,sum=0;*q;q++) sum^=*q;
    p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    return p-(char *)buf;
  }
  for (solq=0;solq<8;solq++) if (nmea_solq[solq]==sol.stat) break;
  if (solq>=8) solq=0;
  time=gpst2utc(sol.time);
  time2epoch(time,ep);
  ecef2pos(sol.rr,pos);
  h=geoidh(pos);
  deg2dms(fabs(pos[0])*R2D,dms1,7);
  deg2dms(fabs(pos[1])*R2D,dms2,7);
  p+=sprintf(p,"$%sGGA,%02.0f%02.0f%06.3f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,"
              "%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,%04d",
              config_.talker_id.data(),ep[3],ep[4],ep[5],dms1[0],dms1[1]+dms1[2]/60.0,
              pos[0]>=0?"N":"S",dms2[0],dms2[1]+dms2[2]/60.0,pos[1]>=0?"E":"W",
              solq,sol.ns,dop,pos[2]-h,h,sol.age,refid);
  for (q=(char *)buf+1,sum=0;*q;q++) sum^=*q; /* check-sum */
  p+=sprintf(p,"*%02X\r\n",sum);
  return p-(char *)buf;
}

// Encode GNRMC message
int NmeaFormator::encodeRMC(const Solution& solution, uint8_t* buf)
{
  sol_t sol;
  convertSolution(solution, sol);

  static double dirp=0.0;
  gtime_t time;
  double ep[6],pos[3],enuv[3],dms1[3],dms2[3],vel,dir,amag=0.0;
  char *p=(char *)buf,*q,sum;
  const char *emag="E",*mode="A",*status="V";
  
  trace(3,"outnmea_rmc:\n");
  
  if (sol.stat<=SOLQ_NONE) {
    p+=sprintf(p,"$%sRMC,,,,,,,,,,,,,",config_.talker_id.data());
    for (q=(char *)buf+1,sum=0;*q;q++) sum^=*q;
    p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    return p-(char *)buf;
  }
  time=gpst2utc(sol.time);
  time2epoch(time,ep);
  ecef2pos(sol.rr,pos);
  ecef2enu(pos,sol.rr+3,enuv);
  vel=norm(enuv,3);
  if (vel>=1.0) {
    dir=atan2(enuv[0],enuv[1])*R2D;
    if (dir<0.0) dir+=360.0;
    dirp=dir;
  }
  else {
    dir=dirp;
  }
  if      (sol.stat==SOLQ_DGPS ||sol.stat==SOLQ_SBAS) mode="D";
  else if (sol.stat==SOLQ_FLOAT||sol.stat==SOLQ_FIX ) mode="R";
  else if (sol.stat==SOLQ_PPP) mode="P";
  deg2dms(fabs(pos[0])*R2D,dms1,7);
  deg2dms(fabs(pos[1])*R2D,dms2,7);
  p+=sprintf(p,"$%sRMC,%02.0f%02.0f%06.3f,A,%02.0f%010.7f,%s,%03.0f%010.7f,"
              "%s,%4.2f,%4.2f,%02.0f%02.0f%02d,%.1f,%s,%s,%s",
              config_.talker_id.data(),ep[3],ep[4],ep[5],dms1[0],dms1[1]+dms1[2]/60.0,
              pos[0]>=0?"N":"S",dms2[0],dms2[1]+dms2[2]/60.0,pos[1]>=0?"E":"W",
              vel/KNOT2M,dir,ep[2],ep[1],(int)ep[0]%100,amag,emag,mode,status);
  for (q=(char *)buf+1,sum=0;*q;q++) sum^=*q; /* check-sum */
  p+=sprintf(p,"*%02X\r\n",sum);
  return p-(char *)buf;
}

// Encode GNESA (self-defined Extended Speed and Attitude) message
// Format: $GNESA,tod,Ve,Vn,Vu,Ar,Ap,Ay*checksum
int NmeaFormator::encodeESA(const Solution& solution, uint8_t* buf)
{
  sol_t sol;
  convertSolution(solution, sol);
  Eigen::Vector3d ypr = 
    solution.integrate_pose.getEigenQuaternion().matrix().eulerAngles(2, 1, 0);
  ypr *= R2D;

  gtime_t time;
  double h,ep[6],pos[3],dms1[3],dms2[3],dop=1.0;
  int solq,refid=0;
  char *p=(char *)buf,*q,sum;
  
  if (sol.stat<=SOLQ_NONE) {
    p+=sprintf(p,"$%sESA,,,,,,,",config_.talker_id.data());
    for (q=(char *)buf+1,sum=0;*q;q++) sum^=*q;
    p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    return p-(char *)buf;
  }
  for (solq=0;solq<8;solq++) if (nmea_solq[solq]==sol.stat) break;
  if (solq>=8) solq=0;
  time=gpst2utc(sol.time);
  time2epoch(time,ep);
  p+=sprintf(p,"$%sESA,%02.0f%02.0f%06.3f,%+.3f,%+.3f,%+.3f,"
             "%+.3f,%+.3f,%+.3f",
             config_.talker_id.data(),ep[3],ep[4],ep[5],sol.rr[3],sol.rr[4],sol.rr[5],
             ypr[2],ypr[1],ypr[0]);
  for (q=(char *)buf+1,sum=0;*q;q++) sum^=*q; /* check-sum */
  p+=sprintf(p,"*%02X\r\n",sum);
  return p-(char *)buf;
}

// Convert Solution to sol_t
void NmeaFormator::convertSolution(const Solution& solution, sol_t& sol)
{
  sol.type = 0;
  sol.time = gnss_common::doubleToGtime(solution.timestamp);
  sol.time = utc2gpst(sol.time);
  sol.age = solution.differential_age;
  sol.ns = solution.num_satellites;
  Eigen::Map<Eigen::Vector3d> rr(sol.rr);
  rr = solution.coordinate->convert(
    solution.integrate_pose.getPosition(), GeoType::ENU, GeoType::ECEF);
  for (int i = 0; i < 3; i++) sol.rr[i + 3] = solution.speed_and_bias(i);
  if (solution.status == GnssSolutionStatus::Fixed) sol.stat = SOLQ_FIX;
  else if (solution.status == GnssSolutionStatus::Float) sol.stat = SOLQ_FLOAT;
  else if (solution.status == GnssSolutionStatus::DGNSS) sol.stat = SOLQ_DGPS;
  else if (solution.status == GnssSolutionStatus::Single) sol.stat = SOLQ_SINGLE;
  else sol.stat = SOLQ_NONE;
}

// -------------------------------------------------------------
// Get formator handle from yaml
#define MAP_FORMATOR(Type, Formator) \
  if (type == Type) { return std::make_shared<Formator>(node); }
#define LOG_UNSUPPORT LOG(FATAL) << "Formator type not supported!";
inline static FormatorType loadType(YAML::Node& node)
{
  if (!node["type"].IsDefined()) {
    LOG(FATAL) << "Unable to load formator type!";
  }
  std::string type_str = node["type"].as<std::string>();
  FormatorType type;
  option_tools::convert(type_str, type);
  return type;
}
FormatorBase::Ptr makeFormator(YAML::Node& node)
{
  FormatorType type = loadType(node);
  MAP_FORMATOR(FormatorType::RTCM2, RTCM2Formator);
  MAP_FORMATOR(FormatorType::RTCM3, RTCM3Formator);
  MAP_FORMATOR(FormatorType::GnssRaw, GnssRawFormator);
  MAP_FORMATOR(FormatorType::ImagePack, ImagePackFormator);
  MAP_FORMATOR(FormatorType::ImageV4L2, ImageV4L2Formator);
  MAP_FORMATOR(FormatorType::IMUPack, IMUPackFormator);
  MAP_FORMATOR(FormatorType::OptionPack, OptionFormator);
  MAP_FORMATOR(FormatorType::NMEA, NmeaFormator);
  LOG_UNSUPPORT;
}

}