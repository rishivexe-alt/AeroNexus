#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/aerostack2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/project_gazebo/config/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$HOME/project_gazebo/config/fastdds_udp_only.xml"
cd ~/project_gazebo
for i in 1 2 3 4; do python3 leader_tasks.py follower $i > /tmp/follower$i.log 2>&1 & done
trap 'kill $(jobs -p) 2>/dev/null' EXIT
sleep 3
python3 leader_tasks.py leader
