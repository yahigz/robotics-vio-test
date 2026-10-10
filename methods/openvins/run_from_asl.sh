#!/bin/bash
# HOST wrapper: build a temporary bag from data/<dataset>/<seq>/asl, run OpenVINS, always delete the bag.
# Usage (from repo root or anywhere): methods/openvins/run_from_asl.sh <dataset> <seq> <config> [extra asl_to_bag args e.g. --duration 60]
# Needs the pixi env (rosbags, cv2). Bags never persist (ROVER: ~1.2 GB/min raw).
set -euo pipefail
DS=$1; SEQ=$2; CFG=$3; shift 3
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
TMP=$(mktemp -d "${TMPDIR:-/tmp}/ovbag.XXXXXX")
trap 'rm -rf "$TMP"' EXIT
CONV=tools/asl_to_bag.py; [ "$DS" = rover ] && CONV=tools/rover/asl_to_bag.py
cd "$ROOT"
# rosbags' LZ4 chunks are rejected by ROS1 rosbag (ROSLZ4_DATA_ERROR), so write an uncompressed bag.
pixi run python -c "
import sys, runpy
from rosbags.rosbag1 import Writer
Writer.set_compression = lambda self, fmt: None
sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name='__main__')" "$CONV" "data/$DS/$SEQ/asl" "$TMP/$SEQ.bag" "$@"
docker compose run --rm -v "$TMP:/bags:ro" -e BAG="/bags/$SEQ.bag" openvins "$DS" "$SEQ" "$CFG"
