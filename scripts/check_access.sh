#!/usr/bin/env bash
# Checklist of everything needed before "go". Prints [ OK ] / [WARN] / [FAIL]; exit code 1 if any FAIL.
cd "$(cd "$(dirname "$0")/.." && pwd)"
FAILS=0
to()   { perl -e 'alarm shift; exec @ARGV' "$@"; }   # portable timeout (macOS has none)
ok()   { printf '[ OK ] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*"; }
fail() { printf '[FAIL] %s\n' "$*"; FAILS=$((FAILS+1)); }

echo "== local (OpenVINS, ORB-SLAM3, evaluation)"
if command -v docker >/dev/null && docker info >/dev/null 2>&1; then ok "docker daemon running"; else fail "docker not running (start Docker Desktop)"; fi
for img in vio-bench/openvins:v2.7 vio-bench/orbslam3:4452a3c; do
  if docker image inspect "$img" >/dev/null 2>&1; then ok "image $img"; else fail "image $img missing -> docker compose build ${img#vio-bench/}"; fi
done
if command -v pixi >/dev/null && pixi run python -c "import evo, rosbags, cv2, clearml" >/dev/null 2>&1; then ok "pixi env (evo, rosbags, cv2, clearml)"; else fail "pixi env incomplete -> pixi install"; fi
for d in data/rover/garden_small_2024-05-29_1 data/rover/campus_small_2024-02-19 data/rover/park_2023-07-31 data/euroc/MH_01_easy; do
  if [ -f "$d/asl/mav0/cam0/data.csv" ] && [ -f "$d/gt_tum.txt" ]; then ok "data $d"; else fail "data $d missing (asl/ or gt_tum.txt)"; fi
done
[ -f data/rover/calib_t265.yaml ] && ok "data/rover/calib_t265.yaml" || fail "data/rover/calib_t265.yaml missing"

echo "== accesses (user setup, docs/ACCESS_SETUP.md)"
if git ls-remote origin HEAD >/dev/null 2>&1; then ok "GitHub repo reachable ($(git remote get-url origin))"; else fail "git ls-remote origin failed (ssh key / VPN / remote 'origin' missing)"; fi
if [ -f "$HOME/clearml.conf" ] && grep -q access_key "$HOME/clearml.conf"; then ok "~/clearml.conf with credentials"; else fail "~/clearml.conf missing/empty -> clearml-init (docs/ACCESS_SETUP.md section 1)"; fi
if [ -f "$HOME/clearml.conf" ] && command -v pixi >/dev/null; then
  if to 60 pixi run python -c "from clearml.backend_api.session.client import APIClient; APIClient().projects.get_all(page_size=1)" >/dev/null 2>&1; then ok "ClearML API reachable with these credentials"; else warn "ClearML API call failed (offline / VPN / wrong keys?)"; fi
fi
if command -v claude >/dev/null; then
  MCP=$(to 60 claude mcp list 2>&1 || true)
  for s in colab-mcp clearml; do
    if printf '%s' "$MCP" | grep -q "^$s"; then
      if printf '%s' "$MCP" | grep "^$s" | grep -qi "connected\|✓"; then ok "claude mcp: $s connected"; else warn "claude mcp: $s listed but not connected: $(printf '%s' "$MCP" | grep "^$s" | head -1)"; fi
    else fail "claude mcp: $s not in 'claude mcp list' (restart Claude Code, docs/ACCESS_SETUP.md section 3)"; fi
  done
else warn "claude CLI not found, MCP servers not checked"; fi
warn "Colab (cannot be checked from here): GPU runtime; Secrets GITHUB_TOKEN + 5 CLEARML_* (Notebook access ON), see docs/ACCESS_SETUP.md"

echo
if [ "$FAILS" = 0 ]; then echo "All checks passed."; else echo "$FAILS check(s) failed."; fi
exit $((FAILS>0))
