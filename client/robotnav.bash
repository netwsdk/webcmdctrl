#!/usr/bin/env bash
# generated from catkin/cmake/templates/setup.bash.in
#
# rc-robotnav.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.

source /opt/ros/melodic/setup.bash
source /home/bingda/catkin_ws/devel/setup.bash
#source /home/bingda/catkin_ws_py3/devel/setup.bash

export ROS_IP=`hostname -I | awk '{print $2}'`
export ROS_HOSTNAME=`hostname -I | awk '{print $2}'`
export ROS_MASTER_URI=http://`hostname -I | awk '{print $2}'`:11311
#export ROS_MASTER_URI=http://192.168.9.1:11311

#BASE_TYPE can Set As NanoRobot NanoCar NanoRobot_Pro NanoCar_Pro NanoCar_SE 4WD 4WD_OMNI
export BASE_TYPE=NanoCar_Pro
#LIDAR_TYPE can set As rplidar/sclidar/rpliadr_super
export LIDAR_TYPE=sclidar
#CAMERA_TYPE can set As astrapro/csi72
export CAMERA_TYPE=astrapro
export SONAR_NUM=2

echo "start ros ..." > /home/bingda/robotnav.log
echo $BASE_TYPE >> /home/bingda/robotnav.log
echo $PATH >> /home/bingda/robotnav.log

# roslaunch robot_navigation robot_nav_web.launch >> /home/bingda/robotnav.log &

/home/bingda/workspace/webcmd_server.py &
/home/bingda/workspace/webcmd_client.py >> /home/bingda/robotnav.log &
