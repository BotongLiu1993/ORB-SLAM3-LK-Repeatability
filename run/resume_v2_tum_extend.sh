#!/usr/bin/env bash
# =============================================================================
# resume_v2_tum_extend.sh - recovery after interruption plus extension: extend the given sequences to n=12
#   Usage (WSL):
#     bash "$W/resume_v2_tum_extend.sh" fr1_floor fr1_360   # these two are also the defaults
#     in the background: nohup bash "$W/resume_v2_tum_extend.sh" > logs/v2tum_extend_outer.log 2>&1 &
#   Behaviour: both the baseline and the v2 stages run, but runs whose *_traj.txt already exists are skipped (r1-3 are not repeated).
#   Prerequisite: code_backups_tum/*.baseline and src/*.bak_tum_v2 are in place (deploy_v2.sh has been run).
# =============================================================================
set -uo pipefail
ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
cd "$ORB" || { echo "[ERROR] $ORB not found"; exit 1; }

BIN="Examples/RGB-D/rgbd_tum"
VOCAB="Vocabulary/ORBvoc.txt"
OUT="results/v2_tum"
mkdir -p "$OUT/baseline" "$OUT/v2" logs code_backups_tum
LK_MM_MATCH_TH="${LK_MM_MATCH_TH:-200}"
RUNS_N=12

SEQS=("${@:-fr1_floor fr1_360}")
[ ${#SEQS[@]} -gt 0 ] || SEQS=(fr1_floor fr1_360)

declare -A SEQ_DIR YAML
SEQ_DIR[fr1_room]="datasets/rgbd_dataset_freiburg1_room";          YAML[fr1_room]="Examples/RGB-D/TUM1.yaml"
SEQ_DIR[fr1_360]="datasets/rgbd_dataset_freiburg1_360";            YAML[fr1_360]="Examples/RGB-D/TUM1.yaml"
SEQ_DIR[fr1_floor]="datasets/rgbd_dataset_freiburg1_floor";        YAML[fr1_floor]="Examples/RGB-D/TUM1.yaml"
SEQ_DIR[fr1_desk]="datasets/rgbd_dataset_freiburg1_desk";          YAML[fr1_desk]="Examples/RGB-D/TUM1.yaml"
SEQ_DIR[fr1_desk2]="datasets/rgbd_dataset_freiburg1_desk2";        YAML[fr1_desk2]="Examples/RGB-D/TUM1.yaml"
SEQ_DIR[fr2_xyz]="datasets/rgbd_dataset_freiburg2_xyz";            YAML[fr2_xyz]="Examples/RGB-D/TUM2.yaml"
SEQ_DIR[fr2_desk]="datasets/rgbd_dataset_freiburg2_desk";          YAML[fr2_desk]="Examples/RGB-D/TUM2.yaml"
SEQ_DIR[fr3_sitting]="datasets/rgbd_dataset_freiburg3_sitting_xyz";YAML[fr3_sitting]="Examples/RGB-D/TUM3.yaml"
SEQ_DIR[fr3_office]="datasets/rgbd_dataset_freiburg3_long_office_household"; YAML[fr3_office]="Examples/RGB-D/TUM3.yaml"

FILES=(src/Tracking.cc src/System.cc include/Tracking.h include/System.h Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc)

patch_viewer_off() {
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
  for f in include/System.h Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc; do
    cp -f "code_backups_tum/$(basename "$f").baseline" "$f"
  done
  patch_viewer_off
}
make_assoc() {
  local seq=$1 f="Examples/associations/${seq}.txt"
  [ -s "$f" ] && return 0
  mkdir -p Examples/associations
  local py="Examples/RGB-D/associate.py"
  [ -f "$py" ] || py="${0%/*}/associate.py"
  if ! python3 "$py" "${SEQ_DIR[$seq]}/rgb.txt" "${SEQ_DIR[$seq]}/depth.txt" > "$f"; then
    rm -f "$f"
    echo "[ERROR] failed to build the association file: $seq"; return 1
  fi
  echo "  [assoc] generated $f"
}
build() {
  local tag=$1
  if [ ! -f Thirdparty/g2o/lib/libg2o.so ]; then
    echo "  [g2o] libg2o.so not found, rebuilding ..."
    ( cd Thirdparty/g2o/build && make -j4 > "/tmp/build_g2o_${tag}.log" 2>&1 ); rc=$?
    [ $rc -ne 0 ] && { echo "[ERROR] g2o rebuild failed:"; tail -20 "/tmp/build_g2o_${tag}.log"; exit 1; }
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
  make_assoc "$seq" || { echo "  [skip] $seq (association file failed)"; return 0; }
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
    rm -f "$OUT/$tag/${seq}_r${r}_evo.zip"
    evo_ape tum "${dir}/groundtruth.txt" "$OUT/$tag/${seq}_r${r}_traj.txt" -a \
        --save_results "$OUT/$tag/${seq}_r${r}_evo.zip" 2>&1 \
        | tee "$OUT/$tag/${seq}_r${r}_evo.txt"
  fi
}
run_phase() {
  local tag=$1
  for seq in "${SEQS[@]}"; do
    for r in $(seq 1 $RUNS_N); do
      if [ -s "$OUT/$tag/${seq}_r${r}_traj.txt" ]; then
        echo "[skip] $tag $seq r$r (trajectory already present)"
        continue
      fi
      run_once "$tag" "$seq" "$r"
    done
  done
}

# ---------------- main ----------------
  command -v evo_ape >/dev/null 2>&1 || { echo "[ERROR] evo_ape not found"; exit 1; }
for s in "${SEQS[@]}"; do
  [ -d "${SEQ_DIR[$s]}" ] || { echo "[ERROR] missing dataset ${SEQ_DIR[$s]}"; exit 1; }
done

echo "=================== Extension: ${SEQS[*]} -> n=$RUNS_N (existing runs are skipped) ==================="
echo "=================== Baseline ==================="
swap_to_baseline
TIMING_PY="${0%/*}/apply_timing_only_patch.py"
[ -f "$TIMING_PY" ] || TIMING_PY="apply_timing_only_patch.py"
if [ -f "$TIMING_PY" ]; then
  python3 "$TIMING_PY" && echo "  [ok] per-frame timing added to the baseline" || echo "  [warn] baseline timing patch failed"
fi
build baseline
run_phase baseline

echo "=================== LK-v2 ==================="
export LK_MM_MATCH_TH
echo "  [gate] LK_MM_MATCH_TH=$LK_MM_MATCH_TH"
swap_to_v2
build v2
run_phase v2

echo ""
echo "======================================================"
echo "done. Results in $OUT/"
find "$OUT" -name "*_traj.txt" | sort
echo "======================================================"
WIN_DST="${WIN_DST:-$HOME/slam_result_mirror/tum_experiments/v2_tum}"
if mkdir -p "$WIN_DST" 2>/dev/null; then
  mkdir -p "$WIN_DST"
  cp -r "$OUT/." "$WIN_DST/" 2>/dev/null
  echo "[OK] copied to $WIN_DST"
else
  echo "[warn] could not create $WIN_DST; results remain in $OUT"
fi
echo "ALL DONE. Current code is LK-v2."