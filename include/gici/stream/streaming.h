/**
* @Function: Handle stream thread
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <functional>
#include <glog/logging.h>

#include "gici/stream/formator.h"
#include "gici/stream/streamer.h"
#include "gici/estimate/estimator_types.h"

namespace gici {

// I/O type
enum class StreamIOType {
  Input,
  Output,
  Log
};

class Streaming {
public:
  // Types
  using PipelineDirect = std::function<void(const uint8_t *buf, int size)>;
  using PipelineConvert = std::function<void(const std::string&, const std::shared_ptr<DataCluster>&)>;
  using DataClusters = std::vector<std::vector<std::shared_ptr<DataCluster>>>;
  using DataCallback = std::function<void(const std::string&, const std::shared_ptr<DataCluster>&)>;
  struct FormatorCtrl {
    std::string tag, input_tag;
    StreamIOType type;
    std::shared_ptr<FormatorBase> formator;
  };


  Streaming(YAML::Node& node, int istreamer);
  ~Streaming();

  // Get formators
  std::vector<FormatorCtrl>& getFormators() { return formators_; }

  // Set data callbacks from outside
  void setDataCallback(DataCallback& callback);

  // Start thread
  void start();

  // Stop thread
  void stop();

  // Pipeline sends input data directly to logging stream
  void pipelineDirectCallback(const uint8_t *buf, int size);

  // Pipeline sends decoded data to logging stream
  void pipelineConvertCallback(
    const std::string& tag, const std::shared_ptr<DataCluster>& data);

  // Send solution data to output stream
  void solutionOutputCallback(
    std::string tag, SolutionRole role, Solution& solution);

  // Check if has formator tag
  inline bool hasFormatorTag(std::string tag) {
    for (size_t i = 0; i < formators_.size(); i++) {
      if (formators_[i].tag == tag) return true;
    }
    return false;
  }

  // Check if valid
  inline bool valid() { return valid_; }

  // Bind input and logging streams
  static void bindLogWithInput();

  // Enable replay
  // This function should be called after all the streamers are opened
  static void enableReplay(StreamerReplayOptions option);

private:
  // Stream input processing
  void processInput();

  // Stream logging processing
  void processLogging();

  // Stream output processing
  void processOutput();

	// Loop processing
	void run();

protected:
	// Thread handles
	std::unique_ptr<std::thread> thread_;
	std::mutex mutex_input_, mutex_logging_, mutex_output_;
	bool quit_thread_ = false;
  double loop_duration_;

  // Stream control
  std::string tag_, input_tag_;  // streamer tags
  bool valid_, opened_;
  std::shared_ptr<StreamerBase> streamer_;
  std::vector<FormatorCtrl> formators_;
  DataClusters data_clusters_;  // store data from each formators
  std::vector<DataCallback> data_callbacks_;  // call external function to send data out
  using PipelinesDirect = std::map<std::string, PipelineDirect>;
  PipelinesDirect pipelines_direct_; // sending data directly to logging streams
  // outer string: send from whom, inner string: who encode the data
  using PipelinesConvert = std::map<std::string, std::map<std::string, PipelineConvert>>;
  PipelinesConvert pipelines_convert_; // sending decoded data to logging streams
  uint8_t *buf_input_, *buf_logging_, *buf_output_;
  int buf_size_input_, buf_size_logging_, buf_size_output_;
  int max_buf_size_;
  bool has_input_ = false;
  bool has_logging_ = false;
  bool has_output_ = false;
  bool need_logging_ = false;
  bool need_output_ = false;

  // Static variables for stream binding
  static std::vector<Streaming *> static_this_;
};

}
