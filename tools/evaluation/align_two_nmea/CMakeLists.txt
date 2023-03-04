cmake_minimum_required(VERSION 2.8)
project(align_two_nmea)

# Set build flags. 
set(CMAKE_CXX_FLAGS "-std=c++11" )
set(CMAKE_C_FLAGS "-std=c99")

link_libraries(m dl rt)

# Add third-party libraries
# Eigen
find_package(Eigen3 REQUIRED NO_MODULE)
# RTKLIB
add_definitions(-DENAGLO -DENACMP -DENAGAL -DNFREQ=3 -DNEXOBS=3 -DDLL)
add_subdirectory(../../../third_party/rtklib rtklib.out)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  include
)

aux_source_directory(src DIR_SRC)
list(APPEND DIR_ALL ${DIR_SRC})

# library
add_executable(${PROJECT_NAME} ${DIR_ALL})
target_link_libraries(${PROJECT_NAME} 
                      rtklib)
