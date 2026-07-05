# GLINS

**GLINS** is a GNSS-LiDAR-INS integrated navigation system for accurate and reliable state
estimation in challenging urban environments. It jointly optimizes GNSS measurements, IMU
preintegration, LiDAR geometric constraints, and navigation states in a keyframe-based sliding
window factor graph.

This implementation is developed from
[GICI-LIB](https://github.com/chichengcn/gici-open) and extends its optimization, streaming, and
ROS infrastructure with a LiDAR frontend, voxel-plane landmarks, and GNSS/IMU/LiDAR estimators.

![GLINS pipeline](https://raw.githubusercontent.com/Garfield-cn/GLINS/main/assets/pipeline.png)

## Authors

Jiahui Liu, Cheng Chi, Xin Zhang, Binlin Zhang, Dongen Li, Xingqun Zhan, and Marcelo H. Ang Jr.

## Related Papers

- J. Liu, C. Chi, X. Zhang, B. Zhang, D. Li, X. Zhan, and M. H. Ang Jr., “GLINS:
  GNSS-LiDAR-INS Integrated Navigation System,” *IEEE Robotics and Automation Letters*, vol. 11,
  no. 5, pp. 6472–6479, May 2026. DOI:
  [10.1109/LRA.2026.3681161](https://doi.org/10.1109/LRA.2026.3681161).
- C. Chi, X. Zhang, J. Liu, Y. Sun, Z. Zhang, and X. Zhan, “GICI-LIB: A GNSS/INS/Camera
  Integrated Navigation Library,” *IEEE Robotics and Automation Letters*, vol. 8, no. 12,
  pp. 7970–7977, December 2023. DOI: [10.1109/LRA.2023.3324825](https://doi.org/10.1109/LRA.2023.3324825).

## Estimation Modes

| Mode | Estimator type | Inputs | Description |
| --- | --- | --- | --- |
| SRR | `gnss_imu_lidar_srr` | GNSS solution, raw IMU, raw LiDAR | Semi-tightly coupled GNSS/IMU/LiDAR estimation. `SRR` means Solution-Raw-Raw. |
| RRR | `rtk_imu_lidar_rrr` | Raw rover/reference GNSS, raw IMU, raw LiDAR | Tightly coupled RTK/IMU/LiDAR estimation with pseudorange, carrier-phase, Doppler, IMU, and LiDAR factors. `RRR` means Raw-Raw-Raw. |

The original GICI GNSS, GNSS/IMU, and GNSS/IMU/Camera estimators remain available through the
same configuration-driven pipeline.

## Dependencies

GLINS is built with C++17 and CMake 3.10 or later. The LiDAR runtime currently uses ROS 1.
Ubuntu 20.04 with ROS Noetic is the simplest supported setup; other Ubuntu versions require a
compatible ROS 1 installation.

| Dependency | Requirement |
| --- | --- |
| [Eigen](https://eigen.tuxfamily.org/) | 3.3 or later |
| [OpenCV](https://opencv.org/) | 4.x |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | 0.6 or later |
| [glog](https://github.com/google/glog) and [gflags](https://github.com/gflags/gflags) | Required |
| [Ceres Solver](https://ceres-solver.org/) | 2.1 or later recommended |
| [PCL](https://pointclouds.org/) | 1.10 or later; 1.12 or later recommended |
| [ROS](https://www.ros.org/) | ROS 1 with catkin |
| [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver) | Required for Livox `CustomMsg` input |

Install the LiDAR driver separately by following the
[livox_ros_driver instructions](https://github.com/Livox-SDK/livox_ros_driver).

For common build problems, refer to the
[GICI Open issue tracker](https://github.com/chichengcn/gici-open/issues).

## Build

### ROS build

Clone GLINS into a location with enough space for the catkin build, then run:

```bash
git clone https://github.com/Garfield-cn/GLINS.git
cd GLINS/ros_wrapper
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

If `livox_ros_driver` is built in another catkin workspace, source that workspace before running
`catkin_make`.

## Run

First source the ROS workspace:

```bash
source ros_wrapper/devel/setup.bash
```

### Run SRR

```bash
rosrun gici_ros gici_ros_main \
  "$(pwd)/ros_wrapper/src/gici/option/ros_gil_tersus_replay_rostopic.yaml"
```

### Run RRR

```bash
rosrun gici_ros gici_ros_main \
  "$(pwd)/ros_wrapper/src/gici/option/ros_rostopic_rtk_lidar_rrr.yaml"
```

The YAML files define all streamers, formatters, estimator inputs, output topics, solver settings,
sensor noise, and extrinsic parameters. Adapt topic names and calibration values before using a new
platform or dataset.

### Quick Start

The scripts start `roscore`, RViz, GLINS, and rosbag playback together:

```bash
./ros_wrapper/start_srr.sh /path/to/rosbags
./ros_wrapper/start_rrr.sh /path/to/rosbags
```

UrbanNav configurations are also provided:

```bash
./ros_wrapper/start_urbannav_srr.sh /path/to/rosbags
./ros_wrapper/start_urbannav_rrr.sh /path/to/rosbags
```

These desktop helpers require `gnome-terminal`, `xdotool`, and RViz. For servers or custom bag
layouts, run `roscore`, `gici_ros_main`, and `rosbag play` separately.

## Default ROS Interfaces

The example RRR configuration uses the following topics. The SRR pipeline uses the same sensor
transport and adds a GNSS solution estimator before the LiDAR fusion estimator.

### Inputs

| Topic | Message/data format | Purpose |
| --- | --- | --- |
| `/gnss_rover` | GICI GNSS raw messages | Rover observations |
| `/gnss_reference` | GICI GNSS raw messages | Reference observations, antenna position, and ephemerides |
| `/gici/imu_raw` | `sensor_msgs/Imu` | Raw IMU measurements |
| `/livox/lidar` | `livox_ros_driver/CustomMsg` | Livox point cloud with per-point time offsets |

Generic `sensor_msgs/PointCloud2` input is also supported through the `pointcloud2` stream format.

### Outputs

| Topic | ROS message | Purpose |
| --- | --- | --- |
| `solution` | `geometry_msgs/PoseStamped` | Navigation pose |
| `solution_odometry` | `nav_msgs/Odometry` | Pose, velocity, and body-frame transform |
| `solution_path` | `nav_msgs/Path` | Estimated trajectory |
| `cloud_current` | `sensor_msgs/PointCloud2` | Current LiDAR scan |
| `cloud_map` | `sensor_msgs/PointCloud2` | Local LiDAR map |
| `landmark` | `visualization_msgs/MarkerArray` | Plane landmarks |

Topic names are configuration values rather than fixed API names.

## Dataset

The GLINS LiDAR data was collected together with GICI. Related public data and conversion tools are
available from the [GICI Open Dataset](https://github.com/chichengcn/gici-open-dataset).

The original sensor data and the GNSS/camera-related data are provided through GICI, together with
the corresponding conversion tools. For LiDAR-only users and users who prefer the ROS workflow, we
also provide a converted [GLINS rosbag dataset](https://pan.baidu.com/s/1VyU5ykz5G_QFszEuzgm6ow?pwd=tamm)
(extraction code: `tamm`).

When preparing another dataset, preserve sensor timestamps and provide per-point LiDAR time offsets
in seconds. Accurate GNSS-IMU and LiDAR-IMU extrinsics are required for meaningful results.

## Video Demo

![GLINS demo](https://raw.githubusercontent.com/Garfield-cn/GLINS/main/assets/GLINS-demo.gif)

## Acknowledgements

GNSS tools, stream handling, message encoding/decoding, and the factor-graph framework are inherited
from [GICI-LIB](https://github.com/chichengcn/gici-open). The IMU factor follows ideas from
[OKVIS](https://github.com/ethz-asl/okvis). The LiDAR frontend and voxel-plane processing were
developed with reference to [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2). GNSS decoding and
positioning build on [RTKLIB](https://github.com/tomojitakasu/RTKLIB).

Please retain the original notices and licenses of all third-party components.

## License

The GLINS source code in this repository is released under the
[GNU General Public License v3.0](LICENSE). Third-party directories remain subject to their
respective licenses.
