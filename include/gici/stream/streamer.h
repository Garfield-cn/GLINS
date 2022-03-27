/**
* @Function: Streamer functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#ifndef STREAMER_H
#define STREAMER_H

#include <iostream>
#include <memory>
#include <glog/logging.h>

#include "gici/utility/transform.h"
#include "gici/utility/option.h"
#include "gici/utility/rtklib_safe.h"

namespace gici {

enum StreamerType {
  Serial = STR_SERIAL,
  TcpClient = STR_TCPCLI,
  TcpServer = STR_TCPSVR,
  File = STR_FILE,
  NtripClient = STR_NTRIPCLI,
  NtripServer = STR_NTRIPSVR,
  V4L2 = 10
};

enum StreamerRWType {
  Read = STR_MODE_R,
  Write = STR_MODE_W,
  ReadAndWrite = STR_MODE_RW
};

struct StreamerReplayOptions {
  double start_offset = 0;
  double speed = 1;
};

// Streamer control
class StreamerBase {
public:
  using Ptr = std::shared_ptr<StreamerBase>;

  StreamerBase() { 
    memset(&stream_, 0, sizeof(stream_t));
    static_this_.push_back(this);
  }
  ~StreamerBase() { }

  // Open stream
  virtual int open(StreamerRWType type) = 0;

  // Close stream
  virtual void close(void) {
    strclose(&stream_);
  }

  // Read data from stream
  virtual int read(uint8_t *buf, int max_size) {
    if (disable_) return 0;
    return strread(&stream_, buf, max_size);
  }

  // Write data to stream
  virtual int write(uint8_t *buf, int size) {
    if (disable_) return 0;
    return strwrite(&stream_, buf, size);
  }

  // Get type
  StreamerType getType(void) { return type_; }

  // Pause stream
  void disable(void) { disable_ = true; }

  // Resume stream
  void enable(void) { disable_ = false; }

  // Enable replay mode
  // This function should be called after all the streamers are initialized
  static void enableReplay(StreamerReplayOptions option);

private:
  // Synchronize streams for replay
  static void syncStreams(void);

protected:
  stream_t stream_;
  StreamerType type_;
  bool disable_ = false;
  StreamerReplayOptions replay_options_;
  static std::vector<StreamerBase *> static_this_;
};

// Serial stream control
class SerialStreamer : public StreamerBase {
public:
  struct Config {
    std::string port;
    int baudrate;
    int bit_size = 8;
    std::string parity = "n";  // n|o|e
    int stop_bit = 1;
    std::string flow_control = "off"; // off|rts
  };

  SerialStreamer(Config& config) :
    config_(config) { 
    type_ = StreamerType::Serial;
  }
  SerialStreamer(YAML::Node& node);
  ~SerialStreamer() { }

  // Open stream
  int open(StreamerRWType type) override;

protected:
  Config config_;
};

// File stream control
class FileStreamer : public StreamerBase {
public:
  struct Config {
    std::string path;
    bool enable_time_tag = false;
    int swap_interval = 0;  // output swap interval (hr) (0: no swap)
  };

  FileStreamer(Config& config) :
    config_(config) { 
    type_ = StreamerType::File;
  }
  FileStreamer(YAML::Node& node);
  ~FileStreamer() { }

  // Open stream
  int open(StreamerRWType type) override;

protected:
  Config config_;
};

// TCP server stream control
class TcpServerStreamer : public StreamerBase {
public:
  struct Config {
    std::string port;
  };

  TcpServerStreamer(Config& config) :
    config_(config) {
    type_ = StreamerType::TcpServer;
  }
  TcpServerStreamer(YAML::Node& node);
  ~TcpServerStreamer() { }

  // Open stream
  int open(StreamerRWType type) override;

protected:
  Config config_;
};

// TCP client stream control
class TcpClientStreamer : public StreamerBase {
public:
  struct Config {
    std::string ip;
    std::string port;
  };

  TcpClientStreamer(Config& config) :
    config_(config) {
    type_ = StreamerType::TcpClient;
  }
  TcpClientStreamer(YAML::Node& node);
  ~TcpClientStreamer() { }

  // Open stream
  int open(StreamerRWType type) override;

protected:
  Config config_;
};

// Ntrip server stream control
class NtripServerStreamer : public StreamerBase {
public:
  struct Config {
    std::string ip;
    std::string port;
    std::string passward;
    std::string mountpoint;
  };

  NtripServerStreamer(Config& config) :
    config_(config) { 
    type_ = StreamerType::NtripServer;
  }
  NtripServerStreamer(YAML::Node& node);
  ~NtripServerStreamer() { }
  
  // Open stream
  int open(StreamerRWType type) override;

protected:
  Config config_;
};

// Ntrip client stream control
class NtripClientStreamer : public StreamerBase {
public:
  struct Config {
    std::string ip;
    std::string port;
    std::string username;
    std::string passward;
    std::string mountpoint;
  };

  NtripClientStreamer(Config& config) :
    config_(config) { 
    type_ = StreamerType::NtripClient;
  }
  NtripClientStreamer(YAML::Node& node);
  ~NtripClientStreamer() { }

  // Open stream
  int open(StreamerRWType type) override;

protected:
  Config config_;
};

// V4L2 stream control
class V4l2Streamer : public StreamerBase {
public:
  struct Config {
    std::string dev;
    int height;
    int width;
    int buffer_count = 1;
  };

  V4l2Streamer(Config& config) :
    config_(config) { 
    type_ = StreamerType::V4L2;
  }
  V4l2Streamer(YAML::Node& node);
  ~V4l2Streamer() { }

  // Open stream
  int open(StreamerRWType type) override;

  // Close stream
  void close(void) override;

  // Read data from stream
  int read(uint8_t *buf, int max_size) override;

  // Write data to stream
  int write(uint8_t *buf, int size) override;

protected:
  Config config_;
  dev_t dev_;
  uint8_t **v4l2_buf;
};

// Get stream handle from configure
#define MAKE_STREAMER(Streamer) \
  inline StreamerBase::Ptr makeStreamer(Streamer::Config& config) { \
    return std::make_shared<Streamer>(config); \
  }
MAKE_STREAMER(SerialStreamer);
MAKE_STREAMER(FileStreamer);
MAKE_STREAMER(TcpServerStreamer);
MAKE_STREAMER(TcpClientStreamer);
MAKE_STREAMER(NtripServerStreamer);
MAKE_STREAMER(NtripClientStreamer);
MAKE_STREAMER(V4l2Streamer);

// Get stream handle from yaml
StreamerBase::Ptr makeStreamer(YAML::Node& node);

}

#endif