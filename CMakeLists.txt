cmake_minimum_required(VERSION 2.8)
project(gici)

# Set build flags. 
set(CMAKE_CXX_FLAGS "-std=c++11" )
set(CMAKE_CXX_FLAGS "-fpermissive")
# Set IS_ARM on odroid board as environment variable
# set(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-const-variable -Wno-unknown-pragmas")
# IF(IS_ARM)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
#   message("#########################")
# ELSE()
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
# ENDIF()
# IF(CMAKE_COMPILER_IS_GNUCC)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
# ELSE()
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# ENDIF()
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Add third-party libraries
# Eigen
find_package(Eigen3 REQUIRED NO_MODULE)
# Opencv
find_package(OpenCV 4 REQUIRED)
# Yaml-cpp 
find_package(yaml-cpp REQUIRED)
# Glog
find_package(glog REQUIRED)
# Fast feature detector
add_subdirectory(third_party/fast)
# SVO vikit tool-kits
add_subdirectory(third_party/rpg_vikit/vikit_common)
# SVO libraries
add_subdirectory(third_party/rpg_svo)
# RTKLIb
add_definitions(-DENAGLO -DENAQZS -DENACMP -DENAGAL -DNFREQ=3 -DNEXOBS=3 -DDLL)
add_subdirectory(third_party/rtklib)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

aux_source_directory(src/utility DIR_utility)
aux_source_directory(src/stream DIR_stream)
aux_source_directory(src/gnss DIR_gnss)
aux_source_directory(src/imu DIR_imu)
aux_source_directory(src/vision DIR_vision)
aux_source_directory(src/optimizer DIR_optimizer)
aux_source_directory(src/fusion DIR_fusion)
list(APPEND DIR_ALL ${DIR_utility}
                    ${DIR_stream}
                    ${DIR_gnss}
                    ${DIR_imu}
                    ${DIR_vision}
                    ${DIR_optimizer}
                    ${DIR_fusion})

# library
add_library(${PROJECT_NAME} SHARED ${DIR_ALL})
target_link_libraries(${PROJECT_NAME} 
                      rtklib
                      vikit_common
                      svo
                      glog
                      ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

# test
add_executable(test_common test/test_common.cpp)
target_link_libraries(test_common 
                      ${PROJECT_NAME})

add_executable(test_stream_handle test/test_stream_handle.cpp)
target_link_libraries(test_stream_handle 
                      ${PROJECT_NAME})

add_executable(test_spp test/test_spp.cpp)
target_link_libraries(test_spp 
                      ${PROJECT_NAME})

add_executable(test_sdgnss test/test_sdgnss.cpp)
target_link_libraries(test_sdgnss 
                      ${PROJECT_NAME})

add_executable(test_dgnss test/test_dgnss.cpp)
target_link_libraries(test_dgnss 
                      ${PROJECT_NAME})

add_executable(test_rtk test/test_rtk.cpp)
target_link_libraries(test_rtk
                      ${PROJECT_NAME})

add_executable(test_gnss_imu_lc test/test_gnss_imu_lc.cpp)
target_link_libraries(test_gnss_imu_lc
                      ${PROJECT_NAME})

add_executable(test_pose test/test_pose.cpp)
target_link_libraries(test_pose
                      ${PROJECT_NAME})