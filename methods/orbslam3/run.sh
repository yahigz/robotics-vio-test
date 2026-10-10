#!/usr/bin/env bash
# Run ORB-SLAM3 (EuRoC/ASL example executables) on data/<dataset>/<seq>/asl and write
# results/orbslam3_<mode>/<dataset>/<seq>/traj_tum.txt  (TUM, t in seconds, IMU/body-frame pose)
#
# usage: run_orbslam3 <dataset> <seq> <mono_inertial|stereo_inertial> <settings.yaml> [--runs N] [--loop-closing] [--max-frames N]
#   settings.yaml: path, or a file name in /workspace/config (then /opt/vio-config)
#   --loop-closing : enable loop closing (default OFF -> `loopClosing: 0` forced into the settings copy)
#   --max-frames N : only use the first N camera frames (smoke tests; trims the timestamps file)
set -euo pipefail

usage() { sed -n '2,8p' "$0"; exit 1; }
[ $# -ge 4 ] || usage
DATASET=$1; SEQ=$2; MODE=$3; SETTINGS=$4; shift 4
RUNS=1; LOOP=0; MAXF=0
while [ $# -gt 0 ]; do
  case $1 in
    --runs) RUNS=$2; shift 2;;
    --loop-closing) LOOP=1; shift;;
    --max-frames) MAXF=$2; shift 2;;
    *) echo "unknown arg $1"; usage;;
  esac
done

ROOT=${ORB_SLAM3_ROOT:-/opt/ORB_SLAM3}
case $MODE in
  mono_inertial)   EXE=$ROOT/Examples/Monocular-Inertial/mono_inertial_euroc;;
  stereo_inertial) EXE=$ROOT/Examples/Stereo-Inertial/stereo_inertial_euroc;;
  *) echo "mode must be mono_inertial|stereo_inertial"; exit 1;;
esac

DATA=/workspace/data/$DATASET/$SEQ/asl
OUT=/workspace/results/orbslam3_$MODE/$DATASET/$SEQ
[ -f "$DATA/mav0/cam0/data.csv" ] || { echo "missing $DATA/mav0/cam0/data.csv"; exit 1; }
[ -f "$DATA/mav0/imu0/data.csv" ] || { echo "missing imu0/data.csv"; exit 1; }
if   [ -f "$SETTINGS" ]; then :
elif [ -f "/workspace/config/$SETTINGS" ]; then SETTINGS=/workspace/config/$SETTINGS
elif [ -f "/opt/vio-config/$SETTINGS" ]; then SETTINGS=/opt/vio-config/$SETTINGS
else echo "settings not found: $SETTINGS"; exit 1; fi

TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
mkdir -p "$OUT"; : > "$OUT/log.txt"

# timestamps file: one ns timestamp per line, from cam0/data.csv
grep -v '^#' "$DATA/mav0/cam0/data.csv" | cut -d, -f1 | tr -d '\r' | sed '/^$/d' > "$TMP/times.txt"
[ "$MAXF" -gt 0 ] && { head -n "$MAXF" "$TMP/times.txt" > "$TMP/t.txt"; mv "$TMP/t.txt" "$TMP/times.txt"; }
# settings copy with the loop-closing switch forced
grep -v '^loopClosing' "$SETTINGS" > "$TMP/settings.yaml"
echo "loopClosing: $LOOP" >> "$TMP/settings.yaml"
cp "$TMP/settings.yaml" "$OUT/settings_used.yaml"

{
  echo "# orbslam3 $(date -u +%FT%TZ) dataset=$DATASET seq=$SEQ mode=$MODE settings=$SETTINGS runs=$RUNS loopClosing=$LOOP max_frames=$MAXF"
  echo "# frames: $(wc -l < "$TMP/times.txt")"
} | tee -a "$OUT/log.txt"

# ORB-SLAM3 output (SaveTrajectoryEuRoC): "t_ns tx ty tz qx qy qz qw", pose of IMU/body frame (Twb),
# world frame = body frame of the first keyframe. Convert t to seconds.
to_tum() { awk '!/^#/ && NF==8 {printf "%.9f %s %s %s %s %s %s %s\n", $1/1e9,$2,$3,$4,$5,$6,$7,$8}' "$1" > "$2"; }

for k in $(seq 1 "$RUNS"); do
  W=$TMP/run$k; mkdir -p "$W"; cd "$W"
  echo "=== run $k/$RUNS ===" | tee -a "$OUT/log.txt"
  T0=$(date +%s.%N)
  rc=0
  "$EXE" "$ROOT/Vocabulary/ORBvoc.txt" "$TMP/settings.yaml" "$DATA" "$TMP/times.txt" run 2>&1 | tee -a "$OUT/log.txt" || rc=$?
  T1=$(date +%s.%N)
  echo "# run $k exit=$rc wall time: $(awk "BEGIN{print $T1-$T0}") s" | tee -a "$OUT/log.txt"
  if [ -s "$W/f_run.txt" ]; then
    to_tum "$W/f_run.txt"  "$OUT/traj_tum_run$k.txt"
    [ -s "$W/kf_run.txt" ] && to_tum "$W/kf_run.txt" "$OUT/kf_traj_tum_run$k.txt"
    echo "# run $k: $(wc -l < "$OUT/traj_tum_run$k.txt") poses written" | tee -a "$OUT/log.txt"
  else
    echo "# run $k: NO trajectory produced" | tee -a "$OUT/log.txt"
  fi
done
cd /
[ -s "$OUT/traj_tum_run1.txt" ] && cp "$OUT/traj_tum_run1.txt" "$OUT/traj_tum.txt"
[ -s "$OUT/kf_traj_tum_run1.txt" ] && cp "$OUT/kf_traj_tum_run1.txt" "$OUT/kf_traj_tum.txt"
echo "done -> $OUT/traj_tum.txt"
