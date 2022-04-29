cmake_minimum_required(VERSION 2.8)
project(gici_ros)

set(CMAKE_CXX_FLAGS "-std=c++11" )
set(CMAKE_CXX_FLAGS "-fpermissive")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -O3 -ansi -pedantic")
set(CMAKE_C_FLAGS "-Wno-unused-but-set-variable")

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED)
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
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(../../.. gici.out)

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS roscpp std_msgs nav_msgs
        sensor_msgs geometry_msgs 
        tf tf_conversions cv_bridge
        image_transport eigen_conversions
  DEPENDS EIGEN3 OpenCV
)

# Sources
aux_source_directory(src/ros_interface DIR_ROS_TOOLS)

# test
add_executable(test_ros_gnss_imu_lc 
               src/test/test_ros_gnss_imu_lc.cpp ${DIR_ROS_TOOLS})
target_link_libraries(test_ros_gnss_imu_lc 
      gici ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(test_ros_rtk_imu_tc 
      src/test/test_ros_rtk_imu_tc.cpp ${DIR_ROS_TOOLS})
target_link_libraries(test_ros_rtk_imu_tc 
      gici ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(test_ros_imu 
               src/test/test_ros_imu.cpp ${DIR_ROS_TOOLS})
target_link_libraries(test_ros_imu 
      gici ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(test_ros_plot_traj 
      src/test/test_ros_plot_traj.cpp ${DIR_ROS_TOOLS})
target_link_libraries(test_ros_plot_traj 
      gici ${OpenCV_LIBS} ${catkin_LIBRARIES})