/**
* @Function: Handle stream thread
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/stream/streaming.h"

#include <glog/logging.h>

#include "gici/utility/spin_control.h"

namespace gici {

// Static variables for stream binding
std::vector<Streaming *> Streaming::static_this_;

Streaming::Streaming(YAML::Node& node, int istreamer)
{
  // Get streamer option
  if (!node["streamers"][istreamer].IsDefined() ||
    !node["streamers"][istreamer]["streamer"].IsDefined()) {
    LOG(ERROR) << "Streamer " << istreamer << " not defined!";
    return;
  }
  YAML::Node streamer_node = node["streamers"][istreamer]["streamer"];
  if (!option_tools::safeGet(streamer_node, "tag", &tag_)) {
    LOG(ERROR) << "Unable to load streamer tag!";
    return;
  }

  // Initialize streamer
  streamer_ = makeStreamer(streamer_node);
  if (streamer_ == nullptr) return;

  std::vector<std::string> formator_tags;
  std::vector<YAML::Node> formator_nodes; 
  bool any_option_got = false;
  // Get formator option
  if (option_tools::safeGet(streamer_node, "formator_tags", &formator_tags)) {
    any_option_got = true;
    if (!node["formators"].IsDefined()) {
      LOG(ERROR) << "Unable to load formators!";
      return;
    }
    for (size_t i = 0; i < formator_tags.size(); i++) {
      bool found = false;
      for (auto it : node["formators"]) {
        if (!it["formator"].IsDefined()) {
          LOG(ERROR) << "Unable to load formator!";
          continue;
        }
        std::string formator_tag;
        if (!option_tools::safeGet(it["formator"], "tag", &formator_tag)) {
          LOG(ERROR) << "Unable to load formator tag!";
          continue;
        }
        if (formator_tag == formator_tags[i]) {
          formator_nodes.push_back(it["formator"]);
          found = true; break;
        }
      }
      if (!found) {
        LOG(WARNING) << "Formator-tag " << formator_tags[i] << " not found!";
      }
    }

    // Initialize formators
    for (auto it : formator_nodes) {
      std::string type_str;
      if (!option_tools::safeGet(it, "io", &type_str)) {
        LOG(ERROR) << "Unable to load formator I/O type!";
        continue;
      }
      StreamIOType type;
      option_tools::convert(type_str, type);

      std::shared_ptr<FormatorBase> formator = makeFormator(it);
      FormatorCtrl formator_ctrl;
      formator_ctrl.formator = formator;
      formator_ctrl.tag = it["tag"].as<std::string>();
      formator_ctrl.type = type;
      // If logging stream, find and handle corresponding input stream as well
      if (type == StreamIOType::Log) {
        if (!option_tools::safeGet(it, "input-tag", &formator_ctrl.input_tag)) {
          LOG(ERROR) << "Unable to load formator input-tag!";
          continue;
        }
      }
      formators_.push_back(formator_ctrl);
      data_clusters_.push_back(std::vector<std::shared_ptr<DataCluster>>());

      if (type == StreamIOType::Input) has_input_ = true;
      if (type == StreamIOType::Log) has_logging_ = true;
      if (type == StreamIOType::Output) has_output_ = true;
    }
  }

  // Get input-tag option for direct logging
  if (option_tools::safeGet(streamer_node, "input-tag", &input_tag_)) {
    any_option_got = true;
    has_logging_ = true;
  }

  if (!any_option_got) {
    LOG(ERROR) << "Unable to load either formator_tags nor input-tag!";
    return;
  }

  // Initialize buffer
  if (!option_tools::safeGet(streamer_node, "buffer_length", &max_buf_size_)) {
    LOG(INFO) << "Unable to load buffer length! Using default instead.";
    max_buf_size_ = 32768;
  }
  if ((has_input_ && 
    !(buf_input_ = (uint8_t *)malloc(sizeof(uint8_t) * max_buf_size_))) ||
    (has_logging_ && 
    !(buf_logging_ = (uint8_t *)malloc(sizeof(uint8_t) * max_buf_size_))) ||
    (has_output_ && 
    !(buf_output_ = (uint8_t *)malloc(sizeof(uint8_t) * max_buf_size_))) ) {
    free(buf_input_); free(buf_logging_); free(buf_output_);
    LOG(FATAL) << __FUNCTION__ << ": Buffer malloc error!";
  }

  // Get loop rate
  if (!option_tools::safeGet(streamer_node, "loop_duration", &loop_duration_)) {
    LOG(INFO) << "Unable to load loop duration! Using default instead.";
    loop_duration_ = 0.001;
  }

  // Open streamer
  int n_write = 0, n_read = 0;
  for (auto it : formators_) {
    if (it.type == StreamIOType::Input) n_read++;
    if (it.type == StreamIOType::Output || 
      it.type == StreamIOType::Log) n_write++;
  }
  StreamerRWType rw;
  if (has_logging_) rw = StreamerRWType::Write;
  if (n_read) rw = StreamerRWType::Read;
  if (n_write) rw = StreamerRWType::Write;
  if (n_read && n_write) rw = StreamerRWType::ReadAndWrite;
  if (has_logging_ && n_read) rw = StreamerRWType::ReadAndWrite;
  if (!streamer_->open(rw)) {
    LOG(ERROR) << "Open streamer " << tag_ << " failed!";
  }

  // Save to global for binding
  static_this_.push_back(this);
}

Streaming::~Streaming()
{
  // Close streamer
  streamer_->close();

  // Free buffer
  free(buf_input_); free(buf_logging_); free(buf_output_);
}

// Set data callbacks
void Streaming::setDataCallback(DataCallback& callback)
{
  data_callbacks_.push_back(callback);
}

// Start thread
void Streaming::start()
{
  // Create thread
  quit_thread_ = false;
  thread_.reset(new std::thread(&Streaming::run, this));
}

// Stop thread
void Streaming::stop()
{
  // Kill thread
  if(thread_ != nullptr) {
    quit_thread_ = true;
    thread_->join();
    thread_.reset();
  }
}

// Pipeline sends input data directly to logging stream
void Streaming::pipelineDirectCallback(const uint8_t *buf, int size)
{
  int copy_size = size;
  if (copy_size > max_buf_size_) {
    LOG(WARNING) << "Source buffer size exceeds local buffer size,"
           << " Copy maybe incomplete.";
    copy_size = max_buf_size_;
  }

  mutex_logging_.lock();
  memcpy(buf_logging_, buf, copy_size);
  buf_size_logging_ = copy_size;
  need_logging_ = true;
  mutex_logging_.unlock();
}

// Pipeline sends decoded data to logging stream
void Streaming::pipelineConvertCallback(
  const std::string& tag, const std::shared_ptr<DataCluster>& data)
{
  // Find formator
  std::shared_ptr<FormatorBase> formator = nullptr;
  for (size_t i = 0; i < formators_.size(); i++) {
    if (formators_[i].type != StreamIOType::Log) continue;
    if (formators_[i].tag != tag) continue;
    formator = formators_[i].formator;
  }
  if (!formator) {
    LOG(ERROR) << "Formator with tag " << tag << " not found!";
    return;
  }

  // Encode
  mutex_logging_.lock();
  buf_size_logging_ = formator->encode(data, buf_logging_);
  need_logging_ = true;
  mutex_logging_.unlock();
}

// Send solution data to output stream
void Streaming::solutionOutputCallback(
  std::string tag, SolutionRole role, Solution& solution)
{
  // Find formator
  std::shared_ptr<FormatorBase> formator = nullptr;
  for (size_t i = 0; i < formators_.size(); i++) {
    if (formators_[i].type != StreamIOType::Output) continue;
    formator = formators_[i].formator;
  }
  if (!formator) {
    LOG(ERROR) << "No output formator was specified for this streamer!";
    return;
  }

  // Encode
  mutex_output_.lock();
  std::shared_ptr<DataCluster> data = std::make_shared<DataCluster>(FormatorType::NMEA);
  *data->solution = solution;
  buf_size_output_ = formator->encode(data, buf_output_);
  need_output_ = true;
  mutex_output_.unlock();
}

// Bind input and logging streams
void Streaming::bindLogWithInput()
{
  // Bind through formator pipeline
  for (size_t i = 0; i < static_this_.size(); i++) 
    for (size_t j = 0; j < static_this_[i]->formators_.size(); j++) {
    if (!static_this_[i]->has_logging_) continue;
    auto& streaming_log = static_this_[i];
    if (!(streaming_log->formators_[j].type == StreamIOType::Log)) continue;
    std::string& input_tag = streaming_log->formators_[j].input_tag;
    std::string& log_tag = streaming_log->formators_[j].tag;
    // find input tag in all instantiate objects
    for (size_t m = 0; m < static_this_.size(); m++) 
      for (size_t n = 0; n < static_this_[m]->formators_.size(); n++) {
      if (static_this_[m]->formators_[n].type != StreamIOType::Input) continue;
      if (static_this_[m]->formators_[n].tag != input_tag) continue;
      auto& streaming_in = static_this_[m];
      // name the log_tag to encode the data
      PipelineConvert pipeline = std::bind(&Streaming::pipelineConvertCallback, 
        streaming_log, std::placeholders::_1, std::placeholders::_2);
      streaming_in->pipelines_convert_[input_tag].
        insert(std::make_pair(log_tag, pipeline));
    }
  }

  // Bind through streamer directly
  for (size_t i = 0; i < static_this_.size(); i++) {
    if (!static_this_[i]->has_logging_) continue;
    auto& streaming_log = static_this_[i];
    std::string& input_tag = streaming_log->input_tag_;
    for (size_t j = 0; j < static_this_.size(); j++) {
      if (!static_this_[j]->has_input_) continue;
      if (static_this_[j]->tag_ != input_tag) continue;
      auto& streaming_in = static_this_[j];
      PipelineDirect pipeline = std::bind(&Streaming::pipelineDirectCallback,
        streaming_log, std::placeholders::_1, std::placeholders::_2);
      streaming_in->pipelines_direct_[input_tag] = pipeline;
    }
  }
}

// Enable replay
void Streaming::enableReplay(StreamerReplayOptions option)
{
  // Speed up loop-rate
  for (auto it : static_this_) {
    it->loop_duration_ /= option.speed;
  }

  // Pass options to streamer
  StreamerBase::enableReplay(option);

  // Reopen streams to apply options
  for (auto it : static_this_) {
    StreamerRWType rw_type = it->streamer_->getRwType();
    it->streamer_->close();
    it->streamer_->open(rw_type);
  }

  // Synchronize streams to align timestamps
  StreamerBase::syncStreams();
}

// Stream input processing
void Streaming::processInput()
{
  // Read data from stream
  buf_size_input_ = streamer_->read(buf_input_, max_buf_size_);
  if (buf_size_input_ == 0) return;

  // Decode stream
  for (size_t i = 0; i < formators_.size(); i++) {
    if (formators_[i].type != StreamIOType::Input) continue;
    std::shared_ptr<FormatorBase>& formator = formators_[i].formator;
    std::vector<std::shared_ptr<DataCluster>>& dataset = data_clusters_[i];
    int nobs = formator->decode(buf_input_, buf_size_input_, dataset);
    
    // Call convertion callbacks
    for (int iobs = 0; iobs < nobs; iobs++) {
      // Call data callback
      if (data_callbacks_.size() > 0) 
        for (auto it : data_callbacks_) {
        auto& data_callback = it;
        data_callback(formators_[i].tag, dataset[iobs]);
      }

      // Call logger pipeline
      auto it_i = pipelines_convert_.find(formators_[i].tag);
      if (it_i == pipelines_convert_.end()) continue;
      if (it_i->second.size() == 0) continue;
      auto& pipelines = it_i->second;
      for (auto it_j : pipelines) {
        auto& pipeline = it_j.second;
        auto& log_tag = it_j.first;
        pipeline(log_tag, dataset[iobs]);
      }
    }
  }

  // Call direct pipeline
  for (auto it : pipelines_direct_) {
    auto& pipeline = it.second;
    pipeline(buf_input_, buf_size_input_);
  }
}

// Stream logging processing
void Streaming::processLogging()
{
  if (!need_logging_) return;

  streamer_->write(buf_logging_, buf_size_logging_);

  need_logging_ = false;
}

// Stream output processing
void Streaming::processOutput()
{
  if (!need_output_) return;

  streamer_->write(buf_output_, buf_size_output_);

  need_output_ = false;
}

// Loop processing
void Streaming::run()
{
  // Spin until quit command or global shutdown called 
  SpinControl spin(loop_duration_);
  while (!quit_thread_ && SpinControl::ok()) {
    if (has_input_) {
      mutex_input_.lock();
      processInput();
      mutex_input_.unlock();
    }

    if (has_logging_) {
      mutex_logging_.lock();
      processLogging();
      mutex_logging_.unlock();
    }

    if (has_output_) {
      mutex_output_.lock();
      processOutput();
      mutex_output_.unlock();
    }

    spin.sleep();
  }
}

}
