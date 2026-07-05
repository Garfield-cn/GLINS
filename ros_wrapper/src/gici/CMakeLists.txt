cmake_minimum_required(VERSION 3.10)
project(gici_ros)

# Keep the estimator optimized when catkin_make is run without an explicit
# build type, while still allowing callers to select another configuration.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic -Wno-unused-but-set-variable")

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules
  tf
  tf_conversions
  cv_bridge
  image_transport
  eigen_conversions
  pcl_conversions
  pcl_ros
  livox_ros_driver
  message_generation
)

add_message_files(
  FILES
  GlonassEphemeris.msg
  GnssAntennaPosition.msg
  GnssEphemerides.msg
  GnssEphemeris.msg
  GnssIonosphereParameter.msg
  GnssObservation.msg
  GnssObservations.msg
  GnssSsrCodeBias.msg
  GnssSsrCodeBiases.msg
  GnssSsrEphemerides.msg
  GnssSsrEphemeris.msg
  GnssSsrPhaseBias.msg
  GnssSsrPhaseBiases.msg
) 

generate_messages(
  DEPENDENCIES std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(../../.. gici.out)

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS roscpp std_msgs nav_msgs
        sensor_msgs geometry_msgs 
        tf tf_conversions cv_bridge
        image_transport eigen_conversions
        message_generation
  DEPENDS EIGEN3 OpenCV PCL
)

# Add definitions here to ensure correct memory allocation in RTKLIB
add_definitions(-DENAGLO -DENACMP -DENAGAL -DENAQZS -DENAIRN -DNFREQ=3 -DNEXOBS=3 -DDLL)

# Suppress boost bind global placeholders deprecation warning
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

# Sources
aux_source_directory(src/ros_interface DIR_ROS_TOOLS)

# GICI ROS interface
add_executable(gici_ros_main
               src/gici_ros_main.cpp ${DIR_ROS_TOOLS})
target_link_libraries(gici_ros_main 
      gici ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
