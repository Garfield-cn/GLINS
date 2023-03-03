# GICI Build Instructions

Authors: Cheng Chi

Email: chichengcn@sjtu.edu.cn

GICI supports two build modes: common build or ROS build. 

The common build enables:

a) Most of the streamer I/Os, including serial, TCP/IP server, TCP/IP client, Ntrip server, Ntrip client, V4L2, and file.

b) All the formator decoder and encoders, including RTCM2, RTCM3, Ublox raw, Septentrio raw, Tersus raw, NMEA, DCB file, ATX file, V4L2 image pack, GICI image pack, and GICI IMU pack.

c) All the estimators, including SPP, SDGNSS, DGNSS, RTK, PPP, GNSS/IMU loosely couple, RTK/IMU tightly couple, PPP/IMU tightly couple, GNSS/IMU/Camera semi-tightly couple, RTK/IMU/Camera tightly couple, and PPP/IMU/Camera tightly couple.

d) Real-time or replay stream flow between the above modules.

The ROS build additially enables:

a) ROS stream I/Os that handles ROS topic advertising and subscribing, including sensor_msgs::Image, sensor_msgs::Imu, geometry_msgs::PoseStamped, geometry_msgs::PoseWithCovarianceStamped, nav_msgs::Odometry, visualization_msgs::Marker, and nav_msgs::Path.

b) Real-time or replay stream flow between ROS and common GICI modules.

## 1. Dependencies

#### 1.1 Ubuntu

We are developing our code on Ubuntu-20.04.3-Desktop-amd64. We recommend you to use the same or similar environment if you do not familiar with cross-compiling.

#### 1.2 Eigen 3.3 or later. REQUIRED.

Eigen is a C++ template library for linear algebra. You can find the releases on [Eigen][eigen].

[eigen]: https://eigen.tuxfamily.org/index.php?title=Main_Page

#### 1.3 OpenCV 4.2.0 or later. REQUIRED.

OpenCV is a computer vision library. You can find the releases on [OpenCV][opencv].

[opencv]: https://opencv.org/releases/

#### 1.4 Yaml-cpp 0.6.0 or later. REQUIRED.

Yaml-cpp is a decoder and encoder for YAML formats. We use YAML file to configure our workflow. You can find the releases on [yaml-cpp][yaml].

[yaml]: https://github.com/jbeder/yaml-cpp

#### 1.5 Glog 0.6.0 or later. REQUIRED.
Glog is a logging control library. You can find the releases on [Glog][glog_]. You should install Glog together with [Gflags][gflags]. We suggest you install Glog from source code, rather than apt-get. Because installing from apt-get may make GICI fail to find the Glog library during compiling.

[glog_]: https://github.com/google/glog
[gflags]: https://github.com/gflags/gflags

#### 1.6 Ceres-Solver 2.1.0 or later. REQUIRED.

Ceres-Solver is a nonlinear optimization library. You can find the releases on [Ceres-Solver][ceres].

[ceres]: http://ceres-solver.org/

#### 1.7 ROS Noetic. OPTIONAL.

ROS is a library for robot applications. We provide a ROS wrapper to enable GICI handling some ROS messages. If you want to build GICI with ROS, you should install ROS Noetic. You can find the instrunctions on [ROS][ros].

[ros]: http://wiki.ros.org/Documentation

## 2. Build GICI

#### 2.1 Common build

```
cd <gici-root-directory>
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8 
```

Now you can run GICI via 

```
./gici_main <gici-config-file>
```

#### 2.2 ROS build

```
cd <gici-root-directory>/ros_wrapper
catkin_make -DCMAKE_BUILD_TYPE=Release
source ./devel/setup.bash
```

Now you can run GICI ROS wrapper via 

```
rosrun gici_ros gici_ros_main <gici-config-file>
```
