#!/bin/bash
# Usage: ./run_dynamic_swarm.sh [battery|id|centroid]
source /opt/ros/humble/setup.bash
source ~/aerostack2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/project_gazebo/config/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$HOME/project_gazebo/config/fastdds_udp_only.xml"
cd ~/project_gazebo
CRIT="${1:-battery}"
for i in 0 1 2 3 4; do python3 dynamic_swarm.py $i --criterion "$CRIT" > /tmp/swarm$i.log 2>&1 & done
trap 'kill $(jobs -p) 2>/dev/null' EXIT
sleep 3
python3 swarm_cli.py
