#!/usr/bin/env bash
# =============================================================================
# resume_v2_tum_sel.sh - power-loss recovery: run only the LK-v2 stage (baseline results are already on disk)
#   Usage (WSL):
#     bash "$W/resume_v2_tum_sel.sh" > logs/v2tum_resume_outer.log 2>&1 &
#     or in the foreground: bash "$W/resume_v2_tum_sel.sh"
#   Idempotent: runs whose results/v2_tum/v2/{seq}_r{r}_traj.txt already exists are skipped.
#   Output: results/v2_tum/v2/ plus an automatic copy to $WIN_DST at the end.
# =============================================================================
set -uo pipefail
ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
cd "$ORB" || { echo "[ERROR] $ORB not found"; exit 1; }

BIN="Examples/RGB-D/rgbd_tum"
VOCAB="Vocabulary/ORBvoc.txt"
OUT="results/v2_tum"
mkdir -p "$OUT/baseline" "$OUT/v2" logs code_backups_tum

LK_MM_MATCH_TH="${LK_MM_MATCH_TH:-200}"

declare -A SEQ_DIR YAML
SEQ_DIR[fr1_room]="datasets/rgbd_dataset_freiburg1_room";          YAML[fr1_room]="Examples/RGB-D/TUM1.yaml"
SEQ_DIR[fr1_360]="datasets/rgbd_dataset_freiburg1_360";            YAML[fr1_360]="Examples/RGB-D/TUM1.yaml"
SEQ_DIR[fr1_floor]="datasets/rgbd_dataset_freiburg1_floor";        YAML[fr1_floor]="Examples/RGB-D/TUM1.yaml"
SEQS=(fr1_room fr1_360 fr1_floor)

FILES=(src/Tracking.cc src/System.cc include/Tracking.h include/System.h Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc)

patch_viewer_off() {
  sed -i 's/System::RGBD, *true)/System::RGBD, false)/' Examples/RGB-D/rgbd_tum.cc
  sed -i 's/System::STEREO, *true)/System::STEREO, false)/' Examples/Stereo/stereo_kitti.cc
  grep -q "System::RGBD, false)" Examples/RGB-D/rgbd_tum.cc || echo "[warn] failed to disable the Viewer in rgbd_tum.cc"
  grep -q "System::STEREO, false)" Examples/Stereo/stereo_kitti.cc || echo "[warn] failed to disable the Viewer in stereo_kitti.cc"
}
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
  sed -n '/LK Tracking Statistics/,/====/p' "logs/v2tum_${seq}_${tag}_r${r}.log" > "$OUT/$tag/${seq}_r${r}_lkstats.txt" 2>/dev/null || true
  sed -n '/LK Rotation Deviation/,/====/p' "logs/v2tum_${seq}_${tag}_r${r}.log" > "$OUT/$tag/${seq}_r${r}_rotstats.txt" 2>/dev/null || true
  if [ -f "$OUT/$tag/${seq}_r${r}_traj.txt" ]; then
    rm -f "$OUT/$tag/${seq}_r${r}_evo.zip"
    evo_ape tum "${dir}/groundtruth.txt" "$OUT/$tag/${seq}_r${r}_traj.txt" -a \
        --save_results "$OUT/$tag/${seq}_r${r}_evo.zip" 2>&1 \
        | tee "$OUT/$tag/${seq}_r${r}_evo.txt"
  fi
}

# ---------------- main ----------------
  command -v evo_ape >/dev/null 2>&1 || { echo "[ERROR] evo_ape not found"; exit 1; }
if ! grep -q "LK_MM_MATCH_TH" src/Tracking.cc; then
  if [ -f src/Tracking.cc.bak_tum_v2 ] && grep -q "LK_MM_MATCH_TH" src/Tracking.cc.bak_tum_v2; then
    echo "  [warn] current code is not v2; a v2 snapshot was found and restored automatically"
  else
    echo "[ERROR] neither v2 code nor a v2 snapshot found (run deploy_v2.sh first)"; exit 1
  fi
fi
for s in "${SEQS[@]}"; do
  if [ ! -d "${SEQ_DIR[$s]}" ]; then echo "[warn] missing dataset ${SEQ_DIR[$s]}"; fi
done

echo "=================== LK-v2 (resume mode: baseline already done, skipped) ==================="
export LK_MM_MATCH_TH
echo "  [gate] LK_MM_MATCH_TH=$LK_MM_MATCH_TH"
swap_to_v2
build v2
for seq in "${SEQS[@]}"; do
  for r in 1 2 3; do
    if [ -f "$OUT/v2/${seq}_r${r}_traj.txt" ] && [ -s "$OUT/v2/${seq}_r${r}_traj.txt" ]; then
      echo "[skip] v2 $seq r$r (trajectory already present)"
      continue
    fi
    run_once v2 "$seq" "$r"
  done
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