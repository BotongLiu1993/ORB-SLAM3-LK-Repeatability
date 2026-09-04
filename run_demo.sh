#!/usr/bin/env bash
# Quick environment check for ORB_SLAM3_LK_Repeatability.
# This repository does not bundle the ORB-SLAM3 source or the benchmark
# datasets (see README.md). This script verifies the prerequisites and prints
# the exact commands to deploy, build, and run one TUM sequence.
set -uo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"

echo "== ORB_SLAM3_LK_Repeatability quick check =="
echo "  repo : $REPO"
echo "  orb  : $ORB"

MISS=0

[ -d "$ORB" ] && echo "  [ok] ORB-SLAM3 found at $ORB" || { echo "  [warn] ORB-SLAM3 not found (set ORB_SLAM3_ROOT)"; MISS=$((MISS+1)); }
[ -f "$REPO/patch/S5_lk_v2.patch" ] && echo "  [ok] S5_lk_v2.patch" || { echo "  [error] patch/S5_lk_v2.patch missing"; MISS=$((MISS+1)); }
[ -f "$REPO/patch/Tracking_v2.cc" ] && echo "  [ok] Tracking_v2.cc" || { echo "  [error] patch/Tracking_v2.cc missing"; MISS=$((MISS+1)); }
[ -f "$REPO/build/deploy_v2.sh" ] && echo "  [ok] deploy_v2.sh" || { echo "  [error] build/deploy_v2.sh missing"; MISS=$((MISS+1)); }

command -v cmake >/dev/null 2>&1 && echo "  [ok] cmake" || { echo "  [warn] cmake not found"; MISS=$((MISS+1)); }
command -v g++ >/dev/null 2>&1 && echo "  [ok] g++" || { echo "  [warn] g++ not found"; MISS=$((MISS+1)); }
command -v python3 >/dev/null 2>&1 && echo "  [ok] python3" || { echo "  [warn] python3 not found"; MISS=$((MISS+1)); }
command -v evo_ape >/dev/null 2>&1 && echo "  [ok] evo_ape" || echo "  [info] evo_ape not found (pip install evo --upgrade --no-binary evo)"

echo ""
echo "Next steps (full instructions in README.md):"
echo "  1) cd \"$ORB\" && bash \"$REPO/build/deploy_v2.sh\" \"$REPO/patch\""
echo "  2) download the datasets (README section 3)"
echo "  3) bash \"$REPO/run/run_v2_tum.sh\" fast      # fr1 desk group"
echo "  4) python \"$REPO/eval/pvalues_v10.py\""

if [ "$MISS" -gt 0 ]; then
  echo ""
  echo "  $MISS prerequisite(s) missing - see README section 4."
  exit 1
fi
exit 0
