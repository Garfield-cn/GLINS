#!/bin/bash
# Author: Jiahui Liu <jh.liu@sjtu.edu.cn>

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="${1:-}"
if [[ -z "${BAG_DIR}" ]]; then
  echo "Usage: $0 <rosbag-directory>" >&2
  exit 2
fi

# Start ROS master
echo "Starting roscore..."
roscore &
ROSCORE_PID=$!

# Wait for ROS master
sleep 1

# Start RViz with the LIO configuration
echo "Starting RViz..."
rviz -d "${SCRIPT_DIR}/src/gici/rviz/gici_gil.rviz" &
RVIZ_PID=$!

# Start GLINS
echo "Starting gici_ros_main..."
rosrun gici_ros gici_ros_main "${SCRIPT_DIR}/src/gici/option/ros_rostopic_lidar_imu.yaml" &
ROS_MAIN_PID=$!

# Play the IMU and LiDAR rosbags in a new terminal
echo "Starting rosbag in a new terminal..."
gnome-terminal --working-directory="${BAG_DIR}" --title="rosbag_play_terminal" -- \
  bash -c "rosbag play imu.bag lidar.bag -r 1.5; exec bash"

# Wait for the rosbag terminal and obtain its window ID
sleep 2
ROS_TERMINAL_WIN_ID=$(xdotool search --name "rosbag_play_terminal")

# Close child processes and terminal windows on exit
trap "echo 'Shutting down...'; kill $ROS_MAIN_PID $RVIZ_PID $ROSCORE_PID 2>/dev/null; xdotool windowclose $ROS_TERMINAL_WIN_ID" SIGINT SIGTERM

# Wait for child processes
wait $ROS_MAIN_PID $RVIZ_PID

# Stop ROS master if it is still running
kill $ROSCORE_PID 2>/dev/null
