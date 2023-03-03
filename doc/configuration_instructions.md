# GICI Configuration Instructions

Authors: Cheng Chi

Email: chichengcn@sjtu.edu.cn

GICI supports multithread streaming, de/encoding, and estimating. One can specify an unlimited number of nodes with our YAML configuration file. Here we will introduce how to configure a GICI workflow.

## 1. Overview

Currently, we support the following features:

#### (1) Streamer module

Supports stream input, output, and logging via serial, TCP/IP server, TCP/IP client, Ntrip server, Ntrip client, V4L2, some ROS topics, or file (including common file and replay file).

#### (2) Formator module

Support data decoding and encoding for RTCM2, RTCM3, Ublox raw, Septentrio raw, Tersus raw, NMEA, DCB file, ATX file, V4L2 image pack, GICI image pack, and GICI IMU pack.

#### (3) Estimator module

See the following table:

| Option name | Description |
|:-----|:------------|
| spp | GNSS single point positioning. |
| sdgnss | GNSS single-differenced pseudorange positioning. |
| dgnss | GNSS dobule-differenced pseudorange positioning. |
| rtk | GNSS real-time kinematic. |
| ppp | GNSS undifferenced and uncombined precise point positioning. |
| gnss_imu_lc | GNSS/INS loosely integration. |
| rtk_imu_tc | GNSS/INS tightly integration, where the GNSS uses RTK formulation. |
| ppp_imu_tc | GNSS/INS tightly integration, where the GNSS uses PPP formulation. |
| gnss_imu_camera_srr | GNSS/INS/Camera semi-tightly integration, with GNSS solution, IMU raw measurement, and camera image. |
| rtk_imu_camera_rrr | GNSS/INS/Camera tightly integration, with GNSS raw measurement, IMU raw measurement, and camera image, where the GNSS uses RTK formulation. |
| ppp_imu_camera_rrr | GNSS/INS/Camera tightly integration, with GNSS raw measurement, IMU raw measurement, and camera image, where the GNSS uses PPP formulation. |

Note that in GICI, the GNSS raw measurement means pseudorange, doppler, and phase-range, rather than the intermediate frequency data.

#### (4) Data exchange between modules

The streamers and estimators run at separated threads, and the formators are bound with the streamers. The modules transfer data via function binding in C++ STL. The following table shows how could the data exchanges in GICI:

| Module | I/O mode | Data exchange options | Description
|:-------|:---------|:----------------|:------------|
| Streamer | I/O | Send stream to multiple formators | To decode data packages in different formats. |
|          |     | Send stream to multiple logging streamers | To log streams. |
|          |     | Send data to multiple estimators (ROS only) | We do not need to decode ROS message because it is not binary stream. |
|          |     | Get stream from multiple formators | To send streams from different data packages. |
|          |     | Get stream from an another streamer | Directly log stream from the input streamer. |
|          |     | Get data from an estimator (ROS only) | We do not need to encode ROS message. |
| Formator | In  | Decode stream from a streamer |  |
|          |     | Send data to multiple estimators |  |
|          |     | Send data to other formators in log mode | To convert data format and log to streams. |
|          |     | Send data to an streamer in log mode (ROS only) | Log stream to a ROS streamer. |
| Formator | Log | Encode data from another formator |  |
|          |     | Send stream to a streamer |  |
| Formator | Out | Encode data from an estimator |  |
|          |     | Send stream to a streamer |  |
| Estimator |    | Get data from multiple formators |  |
|           |    | Get data from other estiamtors | For loosely integration. |
|           |    | Get data from multiple streamers (ROS only) |  |
|           |    | Send solution data to multiple formators | For solution output. |
|           |    | Send solution data to other estimators | To send intermediate solution to loosely integration estimators. |
|           |    | Send solution data to multiple streamers (ROS only) | For solution output. |

Note that the I/O type "Log" means that we redirect the input stream to another stream. This is commonly used for storing raw data to files, or distributing data to other softwares.

## 2. Structure of the configuration file

We use YAML file (.yaml) as our configuration file. All the sentences should meet the YAML format.

There are several level of nodes defined in the configuration file. For the first-level, there are three kinds of nodes: stream, estimate, and logging. See below:

```
stream:
  # Defines all the streamer nodes and formator nodes.
estimate: 
  # Defines all the estimator nodes.
logging: 
  # Defines the run-time logging preferences for the software. We use Google glog for logging control.
```

### 2.1 The stream node

There are three second-level nodes under the stream node: streamers, formators, and replay. See below:

```
stream:
  streamers:
    # Defines all the streamer nodes.
  formators:
    # Defines all the formator nodes.
  replay:
    # Defines the replay options. The replay mode can simulate the real-time stream by stored replay files.
```

In the streamers node, you can define multiple streamer nodes according to your requirements:

```
stream:
  streamers:
  - streamer:
      # Streamer tag, should start with "str_".
      tag: str_...      

      # Streamer type.
      type: serial        

      # Tags of nodes that the data should be sent to.
      output_tags: [...]

      # Tags of nodes where the data comes from.
      input_tags: [...]

      # Other options. There are different options for different types. See section 3.1.
      ...

  - streamer:
    # Another streamer node. The tag names should not be duplicated.
  ...
```

The "type" could be "serial", "tcp-client", "tcp-server", "file", "ntrip-client", "ntrip-server", "v4l2", and "ros". 

The "output_tags" could be formators, other streamers, and estimators (only when setting "type" as "ros"). The "input_tags" could be formators, an another streamer, and estimators (only when setting "type" as "ros"). 

In the formators node, you can define multiple formator nodes:

```
stream:
  formators:
  - formator:
      # Formator tag, should start with "fmt_".
      tag: fmt_...   

      # I/O type, could be input, output, and log.
      io: input

      # Formator type.
      type: gnss-rtcm-3

      # Tags of nodes that the data should be sent to.
      output_tags: [...]

      # Tags of nodes where the stream comes from.
      input_tags: [...]

      # Other options. There are different options for different types. See section 3.2.
      ...

  - formator:
    # Another formator node. The tag names should not be duplicated.
  ...
```

The "type" could be "gnss-rtcm-2", "gnss-rtcm-3", "gnss-raw", "image-v4l2", "image-pack", "imu-pack", "nmea", "dcb-file", and "atx-file".

When setting the "io" option as "input", the "output_tags" could be estimators and other formators, the "input_tags" could be a streamer. When setting the "io" option as "output", the "output_tags" could be a streamer, the "input_tags" should be a estimator. When setting the "io" options as "log", the "output_tags" could be a streamer, the "input_tags" should be an another formator.

The replay node defines the replay functions. One can firstly store streams into replay files with the log mode, and then replay the files by replacing the input streams with replay files and setting the replay options. The replay options are defined as follows:

```
stream:
  replay:
    # Whether to enable replay mode. 
    enable: false

    # Replay speed. 
    speed: 1.0

    # Replay start offset in seconds.
    start_offset: 5.0
```

If set "enable" as true, all the streams except for file streams are disabled, and the files will be replayed according to its tags and the following options. 

### 2.2 The estimate node

You can define the estimators in the second-level node:

```
estimate:
- estimator:
    # Estimator tag, should start with "est_".
    tag: est_...

    # Estimator type.
    type: spp

    # Tags of nodes that the solution should be sent to.
    output_tags: [...]

    # Tags of nodes where the data comes from.
    input_tags: [...]

    # Roles of input tags. xxx is input tag name.
    xxx_roles: [...]
    ...

    # Other options. There are different options for different types. See section 3.3.
    ...

- estimator:
    # Another estimator node. The tag names should not be duplicated.
```

The "type" could be "spp", "sdgnss", "dgnss", "rtk", "ppp", "gnss_imu_lc", "rtk_imu_tc", "ppp_imu_tc", "gnss_imu_camera_srr", "rtk_imu_camera_rrr", and "ppp_imu_camera_rrr".

The "output_tags" could be formators, other estimators, and ROS streamers. The "input_tags" could be formators, other estimators, and ROS streamers.

The "input_tag_roles" could be "rover", "reference", "ephemeris", "ssr_ephemeris", "code_bias", "phase_bias", "heading", and "phase_center" for GNSS data, "major" and "minor" for IMU data, "mono", "stereo_major", "stereo_minor", and "array" for image data.

**Note**: Defining the "output_tags" at the source side equivalents with defining the "input_tags" at the destination side. So is okay to just define the tag connections at one side.

### 2.3 The logging node

The definitions in the logging node is very simple. You should just specify the following 4 options:

```
logging:
  # Enable of disable logging, should be true or false.
  enable: true

  # Minimum logging level.
  min_log_level: 0

  # Whether we should print the logging streams to stderr.
  log_to_stderr: true
  
  # Directory you want to generate the log files.
  file_directory: ...
```

The "min_log_level" should be 0 ~ 3. Where 0 ~ 3 stands for INFO, WARNING, ERROR, FATAL, respectively.

if set "log_to_stderr" to false, we will create log files in "file_directory". If set as true, the logs will be printed to stderr, and the files will not be created.

## 3. Options for specific types

We will illustrate the detailed options for each features in the following tables. Note that if the necessity is "N" (necessary), you must specify this option in your configuration file. If the necessity is "O" (Optional), you do not have to specify them.

### 3.1 Streamer

#### 3.1.1 Common options

| Option | Description | Unit | Default | Necessity |
|:-------|:------------|:-----|:--------|:----------|
| tag    | See section 2.1. |  | "" | N |
| type   | See section 2.1. |  | None | N |
| output_tags | See section 2.1. |  | [] | O | 
| input_tags | See section 2.1. |  | [] | O |
| buffer_length | The length of buffer that temporaly stores the binary stream. It should be determined by the stream load on this channel. | Bit | 32768 | O |
| loop_duration | Duration of loop. It should be determined by your package update rate. | s | 0.005 | O |

#### 3.1.2 Serial 

| Option | Description | Unit | Default | Necessity |
|:-------|:------------|:-----|:--------|:----------|
| port    | Serial port. |  | "" | N |
| baudrate | Baudrate. |  | 0 | N |
| bit_size | Bit size. |  | 8 | O |
| parity | Parity check. Should be n, o, or e. |  | n | O |
| stop_bit | Stop bit. |  | 1 | O |
| flow_control | Hardware flow control for 9-pin serial. Should be off or rts. |  | off | O

#### 3.1.3 TCP/IP client

| Option | Description | Unit | Default | Necessity |
|:-------|:------------|:-----|:--------|:----------|
| ip | Host IP. |  | "" | N |
| port | Communication port. |  | "" | N |

#### 3.1.3 TCP/IP server

| Option | Description | Unit | Default | Necessity |
|:-------|:------------|:-----|:--------|:----------|
| port | Communication port. |  | "" | N |

#### 3.1.4 File

| Option | Description | Unit | Default | Necessity |
|:-------|:------------|:-----|:--------|:----------|
| path | File path. |  | "" | N |
| swap_interval | Maximum time duration for storing data in one file. We will create a new file if it is exceeded. Set 0 to disable swap. | h | 0 | O |
| enable_time_tag | Whether to enable time tag for replay mode. If enabled, a ".tag" file will be created together with the data file to store data reaching timestamps. This will make the file a replay file. In replay mode, if the file is not a replay file, all the containings will be instantaneously loaded. If it is a replay file, it will be generally loaded according to the ".tag" file and the replay options. |  | true | O |

#### 3.1.5 Ntrip client

| Option | Description | Unit | Default | Necessity |
|:-------|:------------|:-----|:--------|:----------|
| ip | Host IP. |  | "" | N |
| port | Communication port. |  | "" | N |
| username | User name. |  | "" | N |
| passward | Passward. |  | "" | N |
| mountpoint | Mountpoint to subscribe. |  | "" | N |

#### 3.1.6 Ntrip server

| Option | Description | Unit | Default | Necessity |
|:-------|:------------|:-----|:--------|:----------|
| ip | Host IP. |  | "" | N |
| port | Communication port. |  | "" | N |
| passward | Passward to upload data. |  | "" | N |
| mountpoint | Mountpoint to publish. |  | "" | N |

#### 3.1.7 V4L2

This function is developed only for the MT9V034 CMOS on our GICI board. For other CMOS sensors, you should modify the corresponding code to make it fit with your kernel driver.

| Option | Description | Unit | Default | Necessity |
|:-------|:------------|:-----|:--------|:----------|
| dev | Device name. |  | "" | N |
| height | Image height. |  | 0 | N |
| width | Image width. |  | 0 | N |
| buffer_count | V4L2 buffer count. |  | 1 | O |

#### 3.1.8 ROS

| Option | Description | Unit | Default | Necessity |
|:-------|:------------|:-----|:--------|:----------|
| io | I/O type, could be input, output, and log |  | None | N |
| format | ROS message format, could be image, imu, gnss_raw, pose_stamped, pose_with_covariance_stamped, marker, path. |  | "" | N |
| topic_name | Name of ROS topic |  | "" | N |
| queue_size | Queue parameter when instantiating ROS topic publisher or subscriber. |  | 10 | O |
| subframe_id | Subframe ID for publishing ROS transform. |  | "" | O |

Except for the "gnss_raw", all the formats are common ROS message types. For our self-defined message types, you can find the definations on "ros_wrapper/src/gici/msg/xxx.msg". For others, see http://wiki.ros.org/ for details. 

### 3.2 Formator

#### 3.2.1 Common options

| Option | Description | Unit | Default | Necessity |
|:-------|:------------|:-----|:--------|:----------|
| tag    | See section 2.1. |  | "" | N |
| type   | See section 2.1. |  | None | N |
| io   | See section 2.1. |  | None | N |
| output_tags | See section 2.1. |  | [] | O | 
| input_tags | See section 2.1. |  | [] | O |

#### 3.2.2 GNSS RTCM 2/3

| Option | Description | Unit | Default | Necessity |
|:-------|:------------|:-----|:--------|:----------|
| start_time | We need a coarse (accurate to day) data start time. |  | System | O |

#### 3.2.3 GNSS raw

| Option | Description | Unit | Default | Necessity |
|:-------|:------------|:-----|:--------|:----------|
| sub_type | Raw data type. Could be ublox, septentrio, or tersus. |  | "" | N |
| start_time | We need a coarse (accurate to day) data start time. |  | System | O |

#### 3.2.3 Image V4L2 and image pack

These are developed only for GICI board.

| Option | Description | Unit | Default | Necessity |
|:-------|:------------|:-----|:--------|:----------|
| width | Image width. |  | 0 | N |
| height | Image height. |  | 0 | N |

#### 3.2.4 NMEA

| Option | Description | Unit | Default | Necessity |
|:-------|:------------|:-----|:--------|:----------|
| use_gga | Whether to use GxGGA message. |  | true | O |
| use_rmc | Whether to use GxRMC message. |  | ture | O |
| use_esa | Whether to use GxESA message. |  | false | O |
| talker_id | NMEA talker ID. |  | "GN" | O |

The GxESA is our self-defined sentence, which stands for Extended Speed and Attitude (ESA). The format is as follows:

```
$GxESA,tod,Ve,Vn,Vu,Ar,Ap,Ay*checksum
```

Where tod is the Time of Day. Ve, Vn, Vu is the velocity in East, North, and Up respectively. Ar, Ap, Ay is the attitude in roll, pitch, and yaw respectively.

#### 3.2.5 Other formator types

Other formator types do not have specific options.

### 3.3 Estimator

#### 3.3.1 Common options

| Option | Description | Unit | Default | Necessity |
|:-------|:------------|:-----|:--------|:----------|
| tag    | See section 2.2. |  | "" | N |
| type   | See section 2.2. |  | None | N |
| output_tags | See section 2.2. |  | [] | O | 
| input_tags | See section 2.2. |  | [] | O |
| xxx_roles | See section 2.2. |  | [] | N$^*$ |
| output_align_tag_ | Align the output rate to the data from this input formator. |  | "" | N |
| output_downsample_rate | Downsample rate of output solutions. Set as 1 if you do not want to downsample this data stream. |  | [1, ...] | O | 
| compute_covariance | Whether to compute covariance. |  | true | O |

*: Necessary if "input_tags" is specified.





