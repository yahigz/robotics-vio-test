#!/usr/bin/env bash
# Fetch DBA-Fusion results produced by the Colab notebook from Drive and re-run evaluation + collect,
# so DBA-Fusion appears in the same tables/plots as OpenVINS and ORB-SLAM3.
#
# usage: scripts/fetch_colab_results.sh [--clearml]
# Drive: gdrive:vio-bench/results/dbafusion/<dataset>/<seq>/  ->  results/dbafusion/<dataset>/<seq>/
set -euo pipefail
cd "$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p results/logs results/dbafusion
exec > >(tee -a "results/logs/fetch_colab_$(date +%Y%m%d_%H%M%S).log") 2>&1

CLEARML=""; [ "${1:-}" = "--clearml" ] && CLEARML="--clearml"
if ! rclone listremotes 2>/dev/null | grep -qx "gdrive:"; then
  echo "!! rclone remote 'gdrive:' not configured (docs/ACCESS_SETUP.md section 2: rclone config)"; exit 1
fi
rclone copy gdrive:vio-bench/results/dbafusion results/dbafusion --stats-one-line --exclude "raw_result.txt"

SEQS=""
for t in results/dbafusion/*/*/traj_tum.txt; do
  [ -f "$t" ] || continue
  SEQS="$SEQS,$(basename "$(dirname "$t")")"
done
SEQS=${SEQS#,}
if [ -z "$SEQS" ]; then echo "!! no results/dbafusion/<dataset>/<seq>/traj_tum.txt found (did the Colab run finish?)"; exit 1; fi
echo "== DBA-Fusion results for: $SEQS"
scripts/run_local.sh --eval-only --seqs "$SEQS" $CLEARML
