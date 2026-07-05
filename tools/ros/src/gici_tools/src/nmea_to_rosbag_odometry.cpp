/**
 * @Function: Convert file from GICI NMEA to rosbag with topic type Odometry
 *
 * @Author  : Jiahui Liu
 * @Email   : jh.liu@sjtu.edu.cn
 **/
#include <rosbag/bag.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include "gici/ros_utility/nmea_formator.h"
#include "gici/gnss/gnss_common.h"

using namespace gici;

const std::string topic_name = "/gici/solution_odometry";

int main(int argc, char** argv)
{
  const double ref[3] = {31.02605369 * D2R, 121.44236563 * D2R, 14.5};

  char nmea_buf[1024];
  if (argc < 2) {
    return -1;
  } else if (argc == 2) {
    strcpy(nmea_buf, argv[1]);
  }

  std::vector<NmeaEpoch> epochs;
  loadNmeaFile(nmea_buf, epochs);

  char buf[1034];
  sprintf(buf, "%s.bag", nmea_buf);
  rosbag::Bag bag;
  bag.open(buf, rosbag::bagmode::Write);

  // Write rosbag
  for (int i = 0; i < epochs.size(); i++) {
    NmeaEpoch& epoch = epochs[i];

    if (epoch.sol.stat == SOLQ_NONE) continue;

    nav_msgs::Odometry msg;
    msg.header.seq = i + 1;
    msg.header.stamp = ros::Time(gnss_common::gtimeToDouble(gpst2utc(epoch.sol.time)));
    msg.header.frame_id = "World";

    double lla[3];
    ecef2pos(epoch.sol.rr, lla);

    double r[3], r0[3], dr[3], enu[3];
    pos2ecef(lla, r);
    pos2ecef(ref, r0);
    for (int i = 0; i < 3; i++) dr[i] = r[i] - r0[i];
    ecef2enu(ref, dr, enu);

    msg.pose.pose.position.x = enu[0];
    msg.pose.pose.position.y = enu[1];
    msg.pose.pose.position.z = enu[2];

    msg.pose.covariance[0] = square(epoch.esd.std_pos[1]);
    msg.pose.covariance[7] = square(epoch.esd.std_pos[0]);
    msg.pose.covariance[14] = square(epoch.esd.std_pos[2]);

    bag.write(topic_name, msg.header.stamp, msg);
  }

  bag.close();

  return 0;
}
