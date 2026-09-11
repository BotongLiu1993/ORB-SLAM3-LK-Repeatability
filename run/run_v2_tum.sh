#!/usr/bin/env bash
# =============================================================================
# run_v2_tum.sh - TUM RGB-D: LK-v2 competing-hypothesis motion prior vs baseline (repeated-run protocol)
# =============================================================================
# Prerequisite: deploy_v2.sh has been run (code_backups_tum/*.baseline and the v2 code are in place).
# Usage:
#   nohup bash run_v2_tum.sh > logs/v2tum_outer.log 2>&1 &     # all sequences
#   bash run_v2_tum.sh fast                                    # fast-motion group only
#   bash run_v2_tum.sh control                                 # control group only
# Output:
#   results/v2_tum/{baseline,v2}/{seq}_r{n}_{traj,kf_traj,times,evo,lkstats,rotstats,timing}
#   Results are copied to $WIN_DST at the end (default: $HOME/slam_result_mirror/tum_experiments/v2_tum).
# =============================================================================
set -uo pipefail

ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
cd "$ORB" || { echo "[ERROR] $ORB not found"; exit 1; }

PHASE="${1:-all}"
case "$PHASE" in all|fast|control|extend|sel) ;; *) echo "unknown phase: $PHASE"; exit 1 ;; esac

BIN="Examples/RGB-D/rgbd_tum"
VOCAB="Vocabulary/ORBvoc.txt"
OUT="results/v2_tum"
mkdir -p "$OUT/baseline" "$OUT/v2" logs code_backups_tum

# LK-v2 trigger threshold (v2 weak motion model: LK is computed only when nmatches < LK_MM_MATCH_TH)
# Ablation: 200 => ~21% trigger rate / ~50% adoption / <2% time overhead; 25 (old default) => LK never triggers
LK_MM_MATCH_TH="${LK_MM_MATCH_TH:-200}"

# Fast-motion group (where the motion model is likely to fail; positive-result candidates) and control group (smooth; "no degradation" evidence)
declare -A SEQ_DIR YAML RUNS
SEQ_DIR[fr1_desk]="datasets/rgbd_dataset_freiburg1_desk";          YAML[fr1_desk]="Examples/RGB-D/TUM1.yaml"; RUNS[fr1_desk]=12
SEQ_DIR[fr1_desk2]="datasets/rgbd_dataset_freiburg1_desk2";        YAML[fr1_desk2]="Examples/RGB-D/TUM1.yaml"; RUNS[fr1_desk2]=3
SEQ_DIR[fr1_room]="datasets/rgbd_dataset_freiburg1_room";          YAML[fr1_room]="Examples/RGB-D/TUM1.yaml"; RUNS[fr1_room]=3
SEQ_DIR[fr1_360]="datasets/rgbd_dataset_freiburg1_360";            YAML[fr1_360]="Examples/RGB-D/TUM1.yaml"; RUNS[fr1_360]=3
SEQ_DIR[fr1_floor]="datasets/rgbd_dataset_freiburg1_floor";        YAML[fr1_floor]="Examples/RGB-D/TUM1.yaml"; RUNS[fr1_floor]=3
SEQ_DIR[fr2_xyz]="datasets/rgbd_dataset_freiburg2_xyz";            YAML[fr2_xyz]="Examples/RGB-D/TUM2.yaml"; RUNS[fr2_xyz]=3
SEQ_DIR[fr2_desk]="datasets/rgbd_dataset_freiburg2_desk";          YAML[fr2_desk]="Examples/RGB-D/TUM2.yaml"; RUNS[fr2_desk]=3
SEQ_DIR[fr3_sitting]="datasets/rgbd_dataset_freiburg3_sitting_xyz";YAML[fr3_sitting]="Examples/RGB-D/TUM3.yaml"; RUNS[fr3_sitting]=3
SEQ_DIR[fr3_office]="datasets/rgbd_dataset_freiburg3_long_office_household"; YAML[fr3_office]="Examples/RGB-D/TUM3.yaml"; RUNS[fr3_office]=3

FAST=(fr1_desk fr1_desk2 fr1_room fr1_360 fr1_floor)
CONTROL=(fr2_xyz fr2_desk fr3_sitting fr3_office)
SEQS=()
if [ "$PHASE" = all ] || [ "$PHASE" = fast ]; then SEQS+=("${FAST[@]}"); fi
if [ "$PHASE" = all ] || [ "$PHASE" = control ]; then SEQS+=("${CONTROL[@]}"); fi
if [ "$PHASE" = sel ]; then
  # run only listed sequences at n=3: bash run_v2_tum.sh sel fr1_room fr1_360 fr1_floor
  SEL_SEQS=("${@:2}")
  [ ${#SEL_SEQS[@]} -gt 0 ] || { echo "[ERROR] sel mode needs sequences"; exit 1; }
  SEQS=("${SEL_SEQS[@]}")
  for s in "${SEQS[@]}"; do RUNS[$s]=3; done
fi
if [ "$PHASE" = extend ]; then
  # Extend the given sequences to n=12 (to check whether the small n=3 differences are noise or a real effect)
  # Usage: bash run_v2_tum.sh extend fr1_desk2 fr2_desk
  EXT_SEQS=("${@:2}")
  [ ${#EXT_SEQS[@]} -gt 0 ] || EXT_SEQS=(fr1_desk2 fr2_desk)
  SEQS=("${EXT_SEQS[@]}")
  for s in "${SEQS[@]}"; do RUNS[$s]=12; done
fi

FILES=(src/Tracking.cc src/System.cc include/Tracking.h include/System.h Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc)

patch_viewer_off() {
  # Run without an X display (headless WSL / SSH); does not affect trajectories or timing statistics
  sed -i 's/System::RGBD, *true)/System::RGBD, false)/' Examples/RGB-D/rgbd_tum.cc
  sed -i 's/System::STEREO, *true)/System::STEREO, false)/' Examples/Stereo/stereo_kitti.cc
  grep -q "System::RGBD, false)" Examples/RGB-D/rgbd_tum.cc || echo "[warn] failed to disable the Viewer in rgbd_tum.cc"
  grep -q "System::STEREO, false)" Examples/Stereo/stereo_kitti.cc || echo "[warn] failed to disable the Viewer in stereo_kitti.cc"
}
swap_to_baseline() { local f; for f in "${FILES[@]}"; do cp -f "code_backups_tum/$(basename "$f").baseline" "$f"; done; patch_viewer_off; }
swap_to_v2() {
  cp -f src/Tracking.cc.bak_tum_v2 src/Tracking.cc
  cp -f include/Tracking.h.bak_tum_v2 include/Tracking.h
  cp -f src/System.cc.bak_tum_v2 src/System.cc
  grep -q "PrintLKStats();" src/System.cc || echo "  [warn] System.cc missing Shutdown print, redeploy needed"
  # v2 prints LK statistics from System::Shutdown(), so System.cc uses the patched v2 snapshot
  for f in include/System.h Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc; do
    cp -f "code_backups_tum/$(basename "$f").baseline" "$f"
  done
  patch_viewer_off
}
check() {
  local keep=()
  for s in "${SEQS[@]}"; do
    if [ -d "${SEQ_DIR[$s]}" ]; then
      if [ -f "${SEQ_DIR[$s]}/rgb.txt" ] && [ -f "${SEQ_DIR[$s]}/depth.txt" ]; then
        keep+=("$s")
      else
        echo "[warn] incomplete data (missing rgb.txt/depth.txt), skipping ${SEQ_DIR[$s]}"
      fi
    else
      echo "[warn] skipping missing dataset ${SEQ_DIR[$s]}"
    fi
  done
  SEQS=("${keep[@]}")
  [ ${#SEQS[@]} -gt 0 ] || { echo "[ERROR] no runnable sequence"; exit 1; }
  command -v evo_ape >/dev/null 2>&1 || { echo "[ERROR] evo_ape not found"; exit 1; }
  if ! grep -q "LK_MM_MATCH_TH" src/Tracking.cc; then
    if [ -f src/Tracking.cc.bak_tum_v2 ] && grep -q "LK_MM_MATCH_TH" src/Tracking.cc.bak_tum_v2; then
      echo "  [warn] current code is not v2; a v2 snapshot (*.bak_tum_v2) was found and restored automatically"
    else
      echo "[ERROR] neither v2 code nor a v2 snapshot found (run deploy_v2.sh first)"; exit 1
    fi
  fi
}
make_assoc() {
  local seq=$1 f="Examples/associations/${seq}.txt"
  [ -s "$f" ] && return 0
  mkdir -p Examples/associations
  local py="Examples/RGB-D/associate.py"
  [ -f "$py" ] || py="${0%/*}/associate.py"   # this fork has no associate.py; use the copy shipped next to this script
  if ! python3 "$py" "${SEQ_DIR[$seq]}/rgb.txt" "${SEQ_DIR[$seq]}/depth.txt" > "$f"; then
    rm -f "$f"
    echo "[ERROR] failed to build the association file: $seq (check ${SEQ_DIR[$seq]}/rgb.txt and depth.txt)"
    return 1
  fi
  echo "  [assoc] generated $f"
}

build() {
  local tag=$1
  if [ ! -f Thirdparty/g2o/lib/libg2o.so ]; then
    echo "  [g2o] libg2o.so not found, rebuilding ..."
    ( cd Thirdparty/g2o/build && make -j4 > "/tmp/build_g2o_${tag}.log" 2>&1 ); rc=$?
    [ $rc -ne 0 ] && { echo "[ERROR] g2o rebuild failed:"; tail -20 "/tmp/build_g2o_${tag}.log"; exit 1; }
    echo "  [g2o] libg2o.so is ready"
  fi
  echo "  [make] building $tag ..."
  ( cd build && make -j4 rgbd_tum > "/tmp/build_v2tum_${tag}.log" 2>&1 ); rc=$?
  if [ $rc -ne 0 ]; then
    echo "[ERROR] $tag build failed:"
    grep -n -i -B2 -A2 "error" "/tmp/build_v2tum_${tag}.log" | head -40
    exit 1
  fi
  echo "  [make] $tag built"
}

run_once() {
  local tag=$1 seq=$2 r=$3
  local dir="${SEQ_DIR[$seq]}" yaml="${YAML[$seq]}" assoc="Examples/associations/${seq}.txt"
  make_assoc "$seq" || { echo "  [skip] $seq (association file failed, skipping)"; return 0; }
  echo "---- [$tag] $seq r$r start:$(date +%H:%M:%S) ----"
  rm -f CameraTrajectory.txt KeyFrameTrajectory.txt track_times_ms.txt
  "$BIN" "$VOCAB" "$yaml" "$dir" "$assoc" > "logs/v2tum_${seq}_${tag}_r${r}.log" 2>&1
  local rc=$?
  echo "  exit=$rc end:$(date +%H:%M:%S)"
  [ -f CameraTrajectory.txt ] && cp CameraTrajectory.txt "$OUT/$tag/${seq}_r${r}_traj.txt" || echo "  [warn] no camera trajectory"
  [ -f KeyFrameTrajectory.txt ] && cp KeyFrameTrajectory.txt "$OUT/$tag/${seq}_r${r}_kf_traj.txt" || true
  [ -f track_times_ms.txt ] && cp track_times_ms.txt "$OUT/$tag/${seq}_r${r}_times.txt" || echo "  [warn] no per-frame timing"
  sed -n '/Tracking Time Statistics/,/====/p' "logs/v2tum_${seq}_${tag}_r${r}.log" > "$OUT/$tag/${seq}_r${r}_timing.txt" 2>/dev/null || true
  if [ "$tag" = v2 ]; then
    sed -n '/LK Tracking Statistics/,/====/p' "logs/v2tum_${seq}_${tag}_r${r}.log" > "$OUT/$tag/${seq}_r${r}_lkstats.txt" 2>/dev/null || true
    sed -n '/LK Rotation Deviation/,/====/p' "logs/v2tum_${seq}_${tag}_r${r}.log" > "$OUT/$tag/${seq}_r${r}_rotstats.txt" 2>/dev/null || true
  fi
  if [ -f "$OUT/$tag/${seq}_r${r}_traj.txt" ]; then
    rm -f "$OUT/$tag/${seq}_r${r}_evo.zip"   # remove first so evo never stops at an overwrite prompt under nohup
    evo_ape tum "${dir}/groundtruth.txt" "$OUT/$tag/${seq}_r${r}_traj.txt" -a \
        --save_results "$OUT/$tag/${seq}_r${r}_evo.zip" 2>&1 \
        | tee "$OUT/$tag/${seq}_r${r}_evo.txt"
  fi
}

# ---------------- main ----------------
check

echo "=================== Baseline (original ORB-SLAM3) ==================="
swap_to_baseline
TIMING_PY="${0%/*}/apply_timing_only_patch.py"   # prefer the copy shipped next to this script
[ -f "$TIMING_PY" ] || TIMING_PY="apply_timing_only_patch.py"
if [ -f "$TIMING_PY" ]; then
  python3 "$TIMING_PY" && echo "  [ok] per-frame timing added to the baseline" || echo "  [warn] baseline timing patch failed (no per-frame timing)"
fi
build baseline
for seq in "${SEQS[@]}"; do
  for r in $(seq 1 "${RUNS[$seq]}"); do run_once baseline "$seq" "$r"; done
done

echo "=================== LK-v2 (competing-hypothesis motion prior) ==================="
export LK_MM_MATCH_TH
echo "  [gate] LK_MM_MATCH_TH=$LK_MM_MATCH_TH (LK is computed only when nmatches < this value)"
swap_to_v2
build v2
for seq in "${SEQS[@]}"; do
  for r in $(seq 1 "${RUNS[$seq]}"); do run_once v2 "$seq" "$r"; done
done

echo ""
echo "======================================================"
echo "done. Results in $OUT/"
find "$OUT" -name "*_traj.txt" | sort
echo "======================================================"

WIN_DST="${WIN_DST:-$HOME/slam_result_mirror/tum_experiments/v2_tum}"
if mkdir -p "$WIN_DST" 2>/dev/null; then
  mkdir -p "$WIN_DST"
  cp -r "$OUT/." "$WIN_DST/" 2>/dev/null
  cp logs/v2tum_*.log "$WIN_DST/logs/" 2>/dev/null || true
  echo "[OK] copied to $WIN_DST"
else
  echo "[warn] could not create $WIN_DST; results remain in $OUT"
fi

echo "ALL DONE. Current code is LK-v2."
