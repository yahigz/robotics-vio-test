#!/usr/bin/env bash
# Upload datasets (ASL + GT) and repo code to Google Drive for the Colab notebook (rclone remote "gdrive").
#
# usage: scripts/upload_drive.sh [--seqs a,b,c] [--code-only] [--data-only] [--dry-run]
#   default seqs: all data/rover/*/ with asl/ + gt_tum.txt, and data/euroc/MH_01_easy
# Layout on Drive (what dbafusion_colab.ipynb expects):
#   gdrive:vio-bench/data/<dataset>/<seq>/{asl,gt_tum.txt}
#   gdrive:vio-bench/repo/{methods/dbafusion,tools,eval,CONTRACT.md}
# rclone copy is idempotent (skips identical files), so re-running only uploads what changed.
set -euo pipefail
cd "$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p results/logs
exec > >(tee -a "results/logs/upload_drive_$(date +%Y%m%d_%H%M%S).log") 2>&1

REMOTE=gdrive; BASE=vio-bench; SEQS=""; DO_DATA=1; DO_CODE=1; DRY=""
while [ $# -gt 0 ]; do
  case $1 in
    --seqs) SEQS=$2; shift 2;;
    --code-only) DO_DATA=0; shift;;
    --data-only) DO_CODE=0; shift;;
    --dry-run) DRY="--dry-run"; shift;;
    -h|--help) sed -n '2,10p' "$0"; exit 0;;
    *) echo "unknown arg $1"; exit 2;;
  esac
done

command -v rclone >/dev/null || { echo "rclone not installed: brew install rclone"; exit 1; }
if ! rclone listremotes 2>/dev/null | grep -qx "$REMOTE:"; then
  cat <<MSG
!! rclone remote '$REMOTE:' is not configured. One-time setup (2 min), see docs/ACCESS_SETUP.md section 2:
     rclone config      # n (new remote) -> name: gdrive -> storage: drive -> client_id/secret empty -> scope 1 -> auto config: y
     rclone lsd gdrive: # check
   then re-run: scripts/upload_drive.sh
MSG
  exit 1
fi

OPTS=(--transfers 8 --checkers 16 --drive-chunk-size 64M --stats 30s --stats-one-line $DRY)

if [ "$DO_DATA" = 1 ]; then
  LIST=()
  if [ -z "$SEQS" ]; then
    for d in data/rover/*/; do [ -d "$d/asl/mav0" ] && [ -f "$d/gt_tum.txt" ] && LIST+=("rover/$(basename "$d")"); done
    [ -d data/euroc/MH_01_easy/asl/mav0 ] && LIST+=("euroc/MH_01_easy")
  else
    IFS=',' read -ra NAMES <<< "$SEQS"
    for n in "${NAMES[@]}"; do
      p=$(ls -d data/rover/$n* data/euroc/$n* 2>/dev/null | head -1 || true)
      [ -n "$p" ] && LIST+=("$(basename "$(dirname "$p")")/$(basename "$p")") || echo "!! '$n' not found under data/, skipping"
    done
  fi
  for s in "${LIST[@]}"; do
    [ -f "data/$s/gt_tum.txt" ] || { echo "!! data/$s/gt_tum.txt missing, skipping"; continue; }
    echo "== data/$s -> $REMOTE:$BASE/data/$s"
    # notebook only needs cam0 + imu0 (ASL), not cam1
    rclone copy "data/$s/asl" "$REMOTE:$BASE/data/$s/asl" --exclude "mav0/cam1/**" "${OPTS[@]}"
    rclone copyto "data/$s/gt_tum.txt" "$REMOTE:$BASE/data/$s/gt_tum.txt" $DRY
  done
  [ -f data/rover/calib_t265.yaml ] && rclone copyto data/rover/calib_t265.yaml "$REMOTE:$BASE/data/rover/calib_t265.yaml" $DRY
fi

if [ "$DO_CODE" = 1 ]; then
  echo "== code -> $REMOTE:$BASE/repo"
  EXC=(--exclude "__pycache__/**" --exclude "*.pyc" --exclude ".ipynb_checkpoints/**" --exclude "generated/**/*.png")
  for d in methods/dbafusion tools eval; do
    rclone copy "$d" "$REMOTE:$BASE/repo/$d" "${EXC[@]}" --stats-one-line $DRY
  done
  rclone copyto CONTRACT.md "$REMOTE:$BASE/repo/CONTRACT.md" $DRY
fi
echo "== done. Next: open methods/dbafusion/dbafusion_colab.ipynb in Colab (Drive repo copy: MyDrive/vio-bench/repo)."
