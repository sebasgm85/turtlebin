#! bin/bash

. /opt/ros/groovy/setup.bash
. ~/ros/devel/setup.bash # This line should be replaced by the one that actually correspond to the catkin workspace of the turtlebin package
export ROS_PACKAGE_PATH=/home/turtlebot/ros_video_streamer_ws/src:$ROS_PACKAGE_PATH # The video streamer workspace is also harcoded so, this needs revision

TURTLEBOT_IP="$(ifconfig eth0 | grep "inet\ " | cut -d: -f 2 | awk '{print $1}')"
export ROS_MASTER_URI=http://"$TURTLEBOT_IP":11311
export ROS_HOSTNAME="$TURTLEBOT_IP"


