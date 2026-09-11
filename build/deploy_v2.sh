#!/usr/bin/env bash
# =============================================================================
# deploy_v2.sh - deploy LK-v2 (competing-hypothesis motion prior) into ORB-SLAM3
# =============================================================================
# Usage:
#   bash deploy_v2.sh [v2 files directory]
#     - by default Tracking_v2.cc / Tracking_v2.h are taken from this script's directory
#     - e.g. bash deploy_v2.sh /mnt/d/SLAM/result/scripts/lk_v2
# What it does:
#   1) back up the current working-tree code as *.bak_tum_v1 (if it carries the v1 marker)
#   2) create baseline backups from git show HEAD (code_backups_tum/*.baseline)
#   3) copy the v2 code into src/ and include/
#   4) build rgbd_tum and stereo_kitti
#   5) self-check the v2 markers
# =============================================================================
set -uo pipefail

ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
cd "$ORB" || { echo "[ERROR] $ORB not found"; exit 1; }

V2DIR="${1:-$(cd "$(dirname "$0")" && pwd)}"
FILES=(src/Tracking.cc src/System.cc include/Tracking.h include/System.h Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc)
mkdir -p code_backups_tum

# ---- 1) back up the current code (if it is the v1 version) ----
if grep -q "LK_CONSISTENCY_THRESHOLD" src/Tracking.cc; then
  for f in "${FILES[@]}"; do cp -f "$f" "${f}.bak_tum_v1"; done
  echo "[bak] current v1 LK code backed up as *.bak_tum_v1"
elif grep -q "LK_MM_MATCH_TH" src/Tracking.cc; then
  for f in "${FILES[@]}"; do cp -f "$f" "${f}.bak_tum_v2"; done
  echo "[bak] current v2 code backed up as *.bak_tum_v2"
else
  echo "[bak] current code is the baseline (nothing to back up)"
fi

# ---- 2) baseline backup (git HEAD) ----
for f in "${FILES[@]}"; do
  b="code_backups_tum/$(basename "$f").baseline"
  git show HEAD:"$f" > "$b" 2>/dev/null || { echo "[ERROR] git show HEAD:$f failed"; exit 1; }
done
echo "[ok] baseline backups ready (code_backups_tum/*.baseline)"

# ---- 3) copy the v2 code ----
for pair in "src/Tracking.cc:Tracking_v2.cc" "include/Tracking.h:Tracking_v2.h"; do
  dst="${pair%%:*}"; srcname="${pair##*:}"
  if [ ! -f "$V2DIR/$srcname" ]; then
    echo "[ERROR] missing $V2DIR/$srcname"; exit 1
  fi
  cp -f "$V2DIR/$srcname" "$dst"
done
echo "[ok] v2 code copied"
# System.cc is restored to the baseline first; step (3a2) below applies the Shutdown() patch that prints LK statistics. Example programs stay pristine (headless).
for f in src/System.cc include/System.h Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc; do
  cp -f "code_backups_tum/$(basename "$f").baseline" "$f" || { echo "[ERROR] missing the baseline for $f"; exit 1; }
done
echo "[ok] System/example programs restored to the pristine state"
# Disable the Pangolin Viewer (headless runs; does not affect trajectories or timing statistics)
sed -i 's/System::RGBD, *true)/System::RGBD, false)/' Examples/RGB-D/rgbd_tum.cc
sed -i 's/System::STEREO, *true)/System::STEREO, false)/' Examples/Stereo/stereo_kitti.cc
echo "[ok] Viewer disabled in the example programs (headless)"
grep -q "LK_MM_MATCH_TH" src/Tracking.cc || { echo "[ERROR] v2 marker missing, aborting"; exit 1; }

# ---- 3b) snapshot the v2 code (the swap_to_v2 helper of run_v2_*.sh relies on *.bak_tum_v2) ----
# ---- 3a2) v2 System.cc: Shutdown() prints LK stats ----
# This ORB-SLAM3 fork has NO System destructor, so Tracking::~Tracking()
# never runs; System::Shutdown() is the last reliable hook (S4-era mechanism).
python3 - <<'PYEOF' || { echo "[ERROR] System.cc Shutdown patch failed"; exit 1; }
import io
p = "src/System.cc"
s = io.open(p, "r", encoding="utf-8").read()
if "mpTracker->PrintLKStats();" not in s:
    old = "#ifdef REGISTER_TIMES\n    mpTracker->PrintTimeStats();\n#endif\n"
    assert s.count(old) == 1, "Shutdown REGISTER_TIMES anchor"
    add = old + (\
        "\n"
        "    // LK-v2: print adoption/validation/timing statistics at shutdown.\n"
        "    // (This ORB-SLAM3 fork has no System destructor, so the Tracking\n"
        "    // destructor is never reached; Shutdown() is the last reliable hook.)\n"
        "    mpTracker->PrintLKStats();\n"
    )
    s = s.replace(old, add, 1)
    io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("[ok] System.cc Shutdown() now prints LK stats")
PYEOF

for f in "${FILES[@]}"; do cp -f "$f" "${f}.bak_tum_v2"; done
echo "[bak] v2 snapshot saved as *.bak_tum_v2 (used by the experiment scripts to switch states)"

# ---- 3c) g2o library check (make clean removes it; rebuild automatically when missing) ----
if [ ! -f Thirdparty/g2o/lib/libg2o.so ]; then
  echo "[g2o] Thirdparty/g2o/lib/libg2o.so not found, rebuilding ..."
  ( cd Thirdparty/g2o/build && make -j4 > /tmp/build_g2o.log 2>&1 ); rc=$?
  if [ $rc -ne 0 ]; then echo "[ERROR] g2o rebuild failed:"; tail -20 /tmp/build_g2o.log; exit 1; fi
  echo "[g2o] libg2o.so is ready"
fi
# ---- 4) build ----
( cd build && make -j4 rgbd_tum stereo_kitti > /tmp/build_v2.log 2>&1 ); rc=$?
if [ $rc -ne 0 ]; then
  echo "[ERROR] build failed (rc=$rc):"
  grep -n -i -B2 -A2 "error" /tmp/build_v2.log | head -60
  exit 1
fi
echo "[make] rgbd_tum + stereo_kitti built"

# ---- 5) self-check ----
echo "[check] v2 markers:"
grep -c "LK_MM_MATCH_TH" src/Tracking.cc
grep -c "LK_FB_TH" src/Tracking.cc
echo "ALL DONE. Current code is LK-v2."
