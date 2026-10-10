#!/usr/bin/env bash
# Pull DBA-Fusion results of the Colab run (ClearML artifacts) and re-run evaluation + collect,
# so DBA-Fusion appears in the same tables/plots as OpenVINS and ORB-SLAM3.
#
# usage: scripts/fetch_colab_results.sh [--clearml] [--seqs garden,park]
# ClearML: project vio-bench, tasks "dbafusion/<dataset>/<seq>", artifacts traj_tum/log/run_stats/metrics/plots
#          -> results/dbafusion/<dataset>/<seq>/   (needs ~/clearml.conf; scripts/fetch_clearml_results.py)
set -euo pipefail
cd "$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p results/logs results/dbafusion
exec > >(tee -a "results/logs/fetch_colab_$(date +%Y%m%d_%H%M%S).log") 2>&1

CLEARML=""; ONLY=""
while [ $# -gt 0 ]; do
  case $1 in
    --clearml) CLEARML="--clearml"; shift;;
    --seqs) ONLY=$2; shift 2;;
    *) echo "unknown arg $1"; exit 2;;
  esac
done
LIST=$(mktemp)
pixi run python scripts/fetch_clearml_results.py --out results/dbafusion --seqs "$ONLY" --list-file "$LIST"
SEQS=$(sed 's#.*/##' "$LIST" | paste -sd, -); rm -f "$LIST"
echo "== DBA-Fusion results for: $SEQS"
scripts/run_local.sh --eval-only --seqs "$SEQS" $CLEARML
