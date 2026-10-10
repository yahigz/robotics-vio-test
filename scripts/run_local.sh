#!/usr/bin/env bash
# Run the local methods (OpenVINS, ORB-SLAM3; Docker) on ROVER + EuRoC sanity, evaluate, collect.
#
# usage: scripts/run_local.sh [--seqs a,b,c] [--methods openvins,orbslam3] [--runs 3] [--force] [--eval-only] [--no-eval] [--clearml]
#   --seqs     comma list of sequence names (prefix match under data/rover or data/euroc; "garden", "campus", "park",
#              "MH_01" work). Default: the 3 ROVER sequences + euroc MH_01_easy.
#   --methods  default openvins,orbslam3
#   --runs     ORB-SLAM3 runs per sequence (default 3; all kept, evaluation uses the median-ATE run)
#   --force    redo runs whose traj_tum.txt already exists (default: skip them)
#   --eval-only / --no-eval   only evaluate+collect / only run
#   --clearml  also report evaluation metrics to ClearML (needs ~/clearml.conf)
# Logs: results/logs/run_local_<time>.log and per-run logs in results/logs/.
set -euo pipefail
cd "$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p results/logs
LOG=results/logs/run_local_$(date +%Y%m%d_%H%M%S).log
exec > >(tee -a "$LOG") 2>&1

SEQS=""; METHODS="openvins,orbslam3"; RUNS=3; FORCE=0; DO_RUN=1; DO_EVAL=1; CLEARML=""
while [ $# -gt 0 ]; do
  case $1 in
    --seqs) SEQS=$2; shift 2;;
    --methods) METHODS=$2; shift 2;;
    --runs) RUNS=$2; shift 2;;
    --force) FORCE=1; shift;;
    --eval-only) DO_RUN=0; shift;;
    --no-eval) DO_EVAL=0; shift;;
    --clearml) CLEARML="--clearml"; shift;;
    -h|--help) sed -n '2,13p' "$0"; exit 0;;
    *) echo "unknown arg $1"; exit 2;;
  esac
done

resolve() {  # name -> "dataset seq"
  local n=$1 ds p
  for ds in rover euroc; do
    p=$(ls -d data/$ds/$n* 2>/dev/null | head -1 || true)
    [ -n "$p" ] && [ -f "$p/gt_tum.txt" ] && { echo "$ds $(basename "$p")"; return; }
  done
  return 1
}
if [ -z "$SEQS" ]; then
  SEQS=""
  for d in data/rover/*/; do [ -f "$d/gt_tum.txt" ] && SEQS="$SEQS,$(basename "$d")"; done
  [ -f data/euroc/MH_01_easy/gt_tum.txt ] && SEQS="$SEQS,MH_01_easy"
  SEQS=${SEQS#,}
fi
has() { case ",$METHODS," in *",$1,"*) return 0;; *) return 1;; esac; }

echo "== run_local $(date) | seqs=$SEQS | methods=$METHODS | runs=$RUNS | force=$FORCE | log=$LOG"
PAIRS=()
IFS=',' read -ra NAMES <<< "$SEQS"
for n in "${NAMES[@]}"; do
  if r=$(resolve "$n"); then PAIRS+=("$r"); else echo "!! sequence '$n' not found under data/ (needs asl/ and gt_tum.txt), skipping"; fi
done

if [ "$DO_RUN" = 1 ]; then
  for pair in "${PAIRS[@]}"; do
    read -r DS SEQ <<< "$pair"
    [ -f "data/$DS/$SEQ/asl/mav0/cam0/data.csv" ] || { echo "!! data/$DS/$SEQ/asl missing, skipping runs"; continue; }
    if has openvins; then
      OVCFG=rover_t265_mono; [ "$DS" = euroc ] && OVCFG=euroc_mav_mono
      OUT=results/openvins/$DS/$SEQ/$OVCFG/traj_tum.txt
      if [ -s "$OUT" ] && [ "$FORCE" = 0 ]; then echo "[skip] openvins $DS/$SEQ (exists)"; else
        echo "[run ] openvins $DS/$SEQ ($OVCFG)"
        methods/openvins/run_from_asl.sh "$DS" "$SEQ" "$OVCFG" > "results/logs/openvins_${DS}_${SEQ}.log" 2>&1 \
          && tail -2 "results/logs/openvins_${DS}_${SEQ}.log" || { echo "!! openvins $DS/$SEQ FAILED, see results/logs/openvins_${DS}_${SEQ}.log"; }
      fi
    fi
    if has orbslam3; then
      CFG=rover_t265_mono_inertial.yaml; [ "$DS" = euroc ] && CFG=EuRoC_mono_inertial.yaml
      OUT=results/orbslam3_mono_inertial/$DS/$SEQ/traj_tum.txt
      if [ -s "$OUT" ] && [ "$FORCE" = 0 ]; then echo "[skip] orbslam3 $DS/$SEQ (exists)"; else
        echo "[run ] orbslam3 mono_inertial $DS/$SEQ x$RUNS (real-time speed: ~sequence duration per run)"
        docker compose run --rm -T orbslam3 "$DS" "$SEQ" mono_inertial "$CFG" --runs "$RUNS" > "results/logs/orbslam3_${DS}_${SEQ}.log" 2>&1 \
          && tail -2 "results/logs/orbslam3_${DS}_${SEQ}.log" || { echo "!! orbslam3 $DS/$SEQ FAILED, see results/logs/orbslam3_${DS}_${SEQ}.log"; }
      fi
    fi
  done
fi

if [ "$DO_EVAL" = 1 ]; then
  for pair in "${PAIRS[@]}"; do
    read -r DS SEQ <<< "$pair"
    echo "== evaluate $DS/$SEQ"
    pixi run python scripts/eval_seq.py "$DS" "$SEQ" $CLEARML || echo "!! evaluation of $DS/$SEQ failed"
  done
  pixi run collect
fi
echo "== done $(date). Tables: results/summary/all.md, per sequence: results/summary/<dataset>/<seq>/"
