#!/usr/bin/env bash
# Build a floor's RTAB-Map database from a recorded walk (record_walk.sh), as
# mapping.launch.py does live. Run from the repository root.
#
#   bash ros2_ws/src/guide_bringup/scripts/map_from_walk.sh walks/<bag dir> AQ-03 [rate]
#
# Replays the bag on sim time at <rate> (default 1.0; 0.5 on a slow laptop so no
# frames are dropped) into rectification + stereo odometry + RTAB-Map in mapping
# mode, writing rtabmap_db/<floor>.db. Then align it at three or more doors with
# `ros2 run guide_mission align_floor` (README phase 1). Rerun with another walk
# of the same floor to extend the map (edit out fresh:=true below).
set -euo pipefail
BAG=${1:?usage: map_from_walk.sh <bag dir> <floor> [rate]}
FLOOR=${2:?floor id, e.g. AQ-03}
RATE=${3:-1.0}
ros2 launch guide_bringup description.launch.py use_sim_time:=true > /dev/null 2>&1 &
ros2 launch guide_bringup cameras.launch.py camera_driver:=false use_sim_time:=true > /tmp/map_from_walk.cameras.log 2>&1 &
ros2 launch guide_bringup rtabmap.launch.py mode:=mapping handheld:=true fresh:=true use_sim_time:=true \
  database:=rtabmap_db/$FLOOR.db > /tmp/map_from_walk.rtabmap.log 2>&1 &
sleep 6
ros2 bag play "$BAG" --clock --rate "$RATE" --topics /stereo/left/image_raw /stereo/left/camera_info \
  /stereo/right/image_raw /stereo/right/camera_info /phone/imu
sleep 5
kill %1 %2 %3 2>/dev/null || true
echo "wrote rtabmap_db/$FLOOR.db"; grep -E "rtabmap \([0-9]+\)" /tmp/map_from_walk.rtabmap.log | tail -1
