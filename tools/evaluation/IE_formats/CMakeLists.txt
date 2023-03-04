cmake_minimum_required(VERSION 2.8)
project(ie_formats)

# Set build flags. 
set(CMAKE_CXX_FLAGS "-std=c++11" )
set(CMAKE_C_FLAGS "-std=c99")

link_libraries(m dl rt)

# Add third-party libraries
# Eigen
find_package(Eigen3 REQUIRED NO_MODULE)
# Glog
find_package(glog REQUIRED)
# RTKLIB
add_definitions(-DENAGLO -DENACMP -DENAGAL -DNFREQ=3 -DNEXOBS=3 -DDLL)
add_subdirectory(../../../third_party/rtklib rtklib.out)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  include
  ../../../include
)

# excecutables
add_executable(ie_to_tum src/ie_to_tum.cpp)
target_link_libraries(ie_to_tum
                      rtklib
                      glog::glog)

add_executable(ie_to_nmea src/ie_to_nmea.cpp src/nmea_encoder.c)
target_link_libraries(ie_to_nmea
                      rtklib
                      glog::glog)

add_executable(ie_gnssins_to_gnss src/ie_gnssins_to_gnss.cpp)
target_link_libraries(ie_gnssins_to_gnss
                      rtklib
                      glog::glog)

add_executable(imupack_to_imr src/imupack_to_imr.cpp 
                              ../../../src/stream/format_imu.c)
target_link_libraries(imupack_to_imr
                      rtklib
                      glog::glog)