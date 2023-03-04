cmake_minimum_required(VERSION 2.8)
project(gici_formats)

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
)

# excecutables
add_executable(nmea_to_tum src/nmea_to_tum.cpp src/nmea_formator.c)
target_link_libraries(nmea_to_tum
                      rtklib
                      glog::glog)

add_executable(nmea_gnssins_to_gnss src/nmea_gnssins_to_gnss.cpp src/nmea_formator.c)
target_link_libraries(nmea_gnssins_to_gnss
                      rtklib
                      glog::glog)

add_executable(nmea_to_ie src/nmea_to_ie.cpp src/nmea_formator.c)
target_link_libraries(nmea_to_ie
                      rtklib
                      glog::glog)