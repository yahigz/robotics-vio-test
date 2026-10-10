#!/bin/bash
# Usage: run.sh <dataset> <sequence> <config_name>
# Reads  /data/<dataset>/<seq>/<seq>.bag, config /opt/vio/config/<config_name>/
# Writes /results/openvins/<dataset>/<seq>/<config>/{traj_tum.txt,log.txt,timing.txt,time_wall.txt}
# Env: BAG (override bag path), BAG_START (s, default 0), BAG_DURR (s, default -1 = all), VERBOSITY (default INFO)
set -eo pipefail
DS=${1:?dataset}; SEQ=${2:?sequence}; CFG=${3:?config name}
BAG=${BAG:-/data/$DS/$SEQ/$SEQ.bag}
CFGDIR=/opt/vio/config/$CFG
OUT=/results/openvins/$DS/$SEQ/$CFG
[ -f "$BAG" ] || { echo "missing bag $BAG" >&2; exit 2; }
[ -f "$CFGDIR/estimator_config.yaml" ] || { echo "missing config $CFGDIR" >&2; exit 2; }
mkdir -p "$OUT"
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

MAXCAM=$(awk '/^max_cameras:/{print $2}' "$CFGDIR/estimator_config.yaml")
STEREO=$(awk '/^use_stereo:/{print $2}' "$CFGDIR/estimator_config.yaml")
[ "$MAXCAM" = 1 ] && STEREO=false

exec > >(tee "$OUT/log.txt") 2>&1
echo "== OpenVINS v2.7 | $DS/$SEQ | config=$CFG | max_cameras=$MAXCAM use_stereo=$STEREO | $(date -u)"
rm -f "$OUT/traj_tum.txt" "$OUT/timing.txt"

roscore >/dev/null 2>&1 &
ROSCORE=$!
until rostopic list >/dev/null 2>&1; do sleep 0.5; done
python3 /opt/vio/scripts/pose_to_tum.py /ov_msckf/poseimu "$OUT/traj_tum.txt" &
REC=$!
sleep 2
START=$(date +%s.%N)
set +e
roslaunch /opt/vio/launch/serial.launch \
  config_path:="$CFGDIR/estimator_config.yaml" bag:="$BAG" \
  max_cameras:="$MAXCAM" use_stereo:="$STEREO" \
  bag_start:="${BAG_START:-0}" bag_durr:="${BAG_DURR:--1}" verbosity:="${VERBOSITY:-INFO}" \
  path_time:="$OUT/timing.txt"
RC=$?
set -e
END=$(date +%s.%N)
sleep 1
kill -INT $REC; wait $REC 2>/dev/null || true
kill $ROSCORE 2>/dev/null || true
echo "wall_time_s $(python3 -c "print(round($END - $START, 2))")" | tee "$OUT/time_wall.txt"
echo "poses: $(wc -l < "$OUT/traj_tum.txt")  roslaunch rc=$RC"
exit $RC
