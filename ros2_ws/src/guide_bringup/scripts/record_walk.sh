#!/usr/bin/env bash
# Walk the campus with the stereo rig and record everything the car will later
# map and localize from (README, "Walking the campus"). Run from the repository root.
#
#   bash ros2_ws/src/guide_bringup/scripts/record_walk.sh <name> [360p|720p]
#
# Starts the two front cameras (usb_cam, fixed exposure), the phone bridge
# (Sensor Logger pushing to this laptop) and the GPS driver if one is plugged
# in, then records a bag under walks/<date>-<name>/ until Ctrl-C:
# raw left/right images and camera info, /phone/imu, /phone/fix, /gps/fix.
# Calibrate first (left.yaml / right.yaml in ~/.ros/camera_info, README phase 1)
# or record the checkerboard at the start of the walk and calibrate from the bag.
set -euo pipefail
NAME=${1:?usage: record_walk.sh <name> [360p|720p]}
CAPTURE=${2:-360p}
OUT=walks/$(date +%Y%m%d-%H%M)-$NAME
mkdir -p walks
N=$(ls /dev/v4l/by-path/ 2>/dev/null | grep -c video-index0 || true)
[ "$N" -ge 2 ] || { echo "found $N cameras under /dev/v4l/by-path/, need the pair"; exit 1; }
GPS=$(ls /dev/serial/by-id/ 2>/dev/null | grep -iv "cp2102\|ch340\|silicon\|wch" | head -1 || true)
ros2 launch guide_bringup cameras.launch.py capture:=$CAPTURE > "$OUT.cameras.log" 2>&1 &
ros2 run guide_bridge phone_imu --ros-args --params-file "$(ros2 pkg prefix guide_bringup)/share/guide_bringup/config/guide.yaml" > "$OUT.phone.log" 2>&1 &
if [ -n "$GPS" ]; then
  ros2 run nmea_navsat_driver nmea_serial_driver --ros-args -p port:=/dev/serial/by-id/$GPS -p baud:=9600 -r fix:=/gps/fix > "$OUT.gps.log" 2>&1 &
fi
sleep 4
bash "$(dirname "$0")/set_exposure.sh" 150 || true
echo "recording to $OUT (Ctrl-C to stop). Walk at the car's pace, about 0.5 m/s, no fast turns;"
echo "hold the rig at the car's camera height and tilt, level side to side."
ros2 bag record -o "$OUT" \
  /stereo/left/image_raw /stereo/left/camera_info /stereo/right/image_raw /stereo/right/camera_info \
  /phone/imu /phone/mag /phone/fix /gps/fix || true
kill %1 %2 2>/dev/null || true; [ -n "$GPS" ] && kill %3 2>/dev/null || true
du -sh "$OUT"
