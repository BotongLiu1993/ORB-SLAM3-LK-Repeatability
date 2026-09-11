#!/usr/bin/env bash
# =============================================================================
# run_v2_euroc.sh - EuRoC MAV (stereo only, no IMU): LK-v2 vs baseline (n=3 repeats)
# =============================================================================
# Why stereo only:
#   The v2 LK competition acts on the constant-velocity motion-model path. The official EuRoC
#   demo runs stereo-inertial by default, and once the IMU is initialized PredictStateIMU
#   bypasses LK entirely. We therefore use Examples/Stereo/EuRoC.yaml
#   (no IMU section) with stereo_euroc. Manuscript rationale: the method replaces the constant-velocity motion model, not IMU integration, so stereo-only is the fair comparison.
# Datasets:
#   Extract the official zips (MH_01_easy ... V2_03_difficult) into datasets/euroc/; the
#   script detects the directory names automatically (MH_01_easy <-> MH01, V1_02_medium <-> V102).
#   Download: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
# Prerequisite: deploy_v2.sh has been run (code_backups_tum/*.baseline and *.bak_tum_v2 in place).
# Usage:
#   nohup bash run_v2_euroc.sh > logs/v2euroc_outer.log 2>&1 &   # all 11 sequences
#   bash run_v2_euroc.sh MH01 V102                                # only the listed sequences
# Output:
#   results/v2_euroc/{baseline,v2}/euroc{seq}_r{n}_{traj,evo,lkstats,rotstats,timing}
#   Results are copied to $WIN_DST at the end (default: $HOME/slam_result_mirror/v2_euroc).
# =============================================================================
set -uo pipefail

ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
cd "$ORB" || { echo "[ERROR] $ORB not found"; exit 1; }

BIN="Examples/Stereo/stereo_euroc"
VOCAB="Vocabulary/ORBvoc.txt"
YAML="Examples/Stereo/EuRoC.yaml"
OUT="results/v2_euroc"
mkdir -p "$OUT/baseline" "$OUT/v2" logs code_backups_tum

if [ $# -gt 0 ]; then
  SEQS=("$@")
else
  SEQS=(MH01 MH02 MH03 MH04 MH05 V101 V102 V103 V201 V202 V203)
fi
RUNS=3

LK_MM_MATCH_TH="${LK_MM_MATCH_TH:-200}"

# deploy_v2.sh does not back up stereo_euroc.cc (identical on both sides apart from the Viewer switch);
# all other files are restored from code_backups_tum/*.baseline.
CORE_FILES=(src/Tracking.cc src/System.cc include/Tracking.h include/System.h \
            Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc)
CLEAN_FILES=(include/System.h Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc)

patch_viewer_off() {
  sed -i 's/System::STEREO, *true)/System::STEREO, false)/' Examples/Stereo/stereo_euroc.cc
  grep -q "System::STEREO, false)" Examples/Stereo/stereo_euroc.cc \
    || echo "[warn] failed to disable the Viewer in stereo_euroc.cc"
}
swap_to_baseline() {
  local f; for f in "${CORE_FILES[@]}"; do cp -f "code_backups_tum/$(basename "$f").baseline" "$f"; done
  patch_viewer_off
}
swap_to_v2() {
  cp -f src/Tracking.cc.bak_tum_v2 src/Tracking.cc
  cp -f include/Tracking.h.bak_tum_v2 include/Tracking.h
  cp -f src/System.cc.bak_tum_v2 src/System.cc
  grep -q "PrintLKStats();" src/System.cc || echo "  [warn] System.cc missing Shutdown print, redeploy needed"
  local f; for f in "${CLEAN_FILES[@]}"; do cp -f "code_backups_tum/$(basename "$f").baseline" "$f"; done
  patch_viewer_off
}

# Resolve the dataset directory: prefer datasets/euroc/<seq>, otherwise match the official naming (MH_01_easy -> MH01)
resolve_seq_dir() {
  local s=$1 d b norm
  if [ -d "datasets/euroc/$s/mav0" ]; then echo "datasets/euroc/$s"; return 0; fi
  for d in datasets/euroc/*/; do
    [ -d "$d/mav0" ] || continue
    b=$(basename "$d")
    norm=$(printf '%s' "$b" | tr -cd '[:alnum:]')
    case "$norm" in
      "$s"*) echo "${d%/}"; return 0 ;;
    esac
  done
  return 1
}

check() {
  command -v evo_ape >/dev/null 2>&1 || { echo "[ERROR] evo_ape not found"; exit 1; }
  if ! grep -q "LK_MM_MATCH_TH" src/Tracking.cc; then
    if [ -f src/Tracking.cc.bak_tum_v2 ] && grep -q "LK_MM_MATCH_TH" src/Tracking.cc.bak_tum_v2; then
      echo "  [warn] current code is not v2; a v2 snapshot (*.bak_tum_v2) was found and restored automatically"
    else
      echo "[ERROR] neither v2 code nor a v2 snapshot found (run deploy_v2.sh first)"; exit 1
    fi
  fi
  [ -f "$YAML" ] || { echo "[ERROR] missing $YAML"; exit 1; }
  local keep=() s dir ts
  for s in "${SEQS[@]}"; do
    dir=$(resolve_seq_dir "$s") || { echo "[warn] skipping missing dataset $s (extract it into datasets/euroc/ and rerun)"; continue; }
    ts="Examples/Stereo/EuRoC_TimeStamps/$s.txt"
    [ -f "$ts" ] || { echo "[warn] missing timestamp file $ts, skipping $s"; continue; }
    keep+=("$s")
  done
  SEQS=("${keep[@]}")
  [ ${#SEQS[@]} -gt 0 ] || { echo "[ERROR] no runnable sequence"; exit 1; }
}

build() {
  local tag=$1
  if [ ! -f Thirdparty/g2o/lib/libg2o.so ]; then
    echo "  [g2o] libg2o.so not found, rebuilding ..."
    ( cd Thirdparty/g2o/build && make -j4 > "/tmp/build_g2o_${tag}.log" 2>&1 ); rc=$?
    [ $rc -ne 0 ] && { echo "[ERROR] g2o rebuild failed:"; tail -20 "/tmp/build_g2o_${tag}.log"; exit 1; }
    echo "  [g2o] libg2o.so is ready"
  fi
  echo "  [make] building $tag (stereo_euroc) ..."
  ( cd build && make -j4 stereo_euroc > "/tmp/build_v2euroc_${tag}.log" 2>&1 ); rc=$?
  if [ $rc -ne 0 ]; then
    echo "[ERROR] $tag build failed:"
    grep -n -i -B2 -A2 "error" "/tmp/build_v2euroc_${tag}.log" | head -40
    exit 1
  fi
  echo "  [make] $tag built"
}

run_once() {
  local tag=$1 s=$2 r=$3
  local seqdir ts gt name
  seqdir=$(resolve_seq_dir "$s")
  ts="Examples/Stereo/EuRoC_TimeStamps/$s.txt"
  gt="$seqdir/mav0/state_groundtruth_estimate0"
  gtcsv="$gt/data.csv"
  name="euroc${s}"
  echo "---- [$tag] $s r$r start:$(date +%H:%M:%S) ----"
  rm -f "f_${name}.txt" "kf_${name}.txt" track_times_ms.txt
  "$BIN" "$VOCAB" "$YAML" "$seqdir" "$ts" "$name" > "logs/v2euroc_${s}_${tag}_r${r}.log" 2>&1
  local rc=$?
  echo "  exit=$rc end:$(date +%H:%M:%S)"
  [ -f "f_${name}.txt" ] && cp "f_${name}.txt" "$OUT/$tag/${name}_r${r}_traj.txt" || echo "  [warn] no camera trajectory"
  [ -f "kf_${name}.txt" ] && cp "kf_${name}.txt" "$OUT/$tag/${name}_r${r}_kf_traj.txt" || true
  [ -f track_times_ms.txt ] && cp track_times_ms.txt "$OUT/$tag/${name}_r${r}_times.txt" || echo "  [warn] no per-frame timing"
  sed -n '/Tracking Time Statistics/,/====/p' "logs/v2euroc_${s}_${tag}_r${r}.log" > "$OUT/$tag/${name}_r${r}_timing.txt" 2>/dev/null || true
  if [ "$tag" = v2 ]; then
    sed -n '/LK Tracking Statistics/,/====/p' "logs/v2euroc_${s}_${tag}_r${r}.log" > "$OUT/$tag/${name}_r${r}_lkstats.txt" 2>/dev/null || true
    sed -n '/LK Rotation Deviation/,/====/p' "logs/v2euroc_${s}_${tag}_r${r}.log" > "$OUT/$tag/${name}_r${r}_rotstats.txt" 2>/dev/null || true
  fi
  if [ -f "$OUT/$tag/${name}_r${r}_traj.txt" ]; then
    rm -f "$OUT/$tag/${name}_r${r}_evo.zip"   # remove first so evo never stops at an overwrite prompt under nohup
    if [ ! -f "$gtcsv" ]; then
      echo "  [warn] ground truth missing: $gtcsv (trajectory saved; rerun evo after adding the data)"
    fi
    evo_ape euroc "$gtcsv" "$OUT/$tag/${name}_r${r}_traj.txt" -a \
        --save_results "$OUT/$tag/${name}_r${r}_evo.zip" 2>&1 \
        | tee "$OUT/$tag/${name}_r${r}_evo.txt"
  fi
}

# ---------------- main ----------------
check
echo "======================================================"
echo "EuRoC stereo-only: baseline vs LK-v2 (LK_MM_MATCH_TH=$LK_MM_MATCH_TH)"
echo "Sequences: ${SEQS[*]}  repeats: n=$RUNS"
echo "======================================================"

echo "=================== Baseline (original ORB-SLAM3) ==================="
swap_to_baseline
TIMING_PY="${0%/*}/apply_timing_only_patch.py"
[ -f "$TIMING_PY" ] || TIMING_PY="apply_timing_only_patch.py"
if [ -f "$TIMING_PY" ]; then
  python3 "$TIMING_PY" && echo "  [ok] per-frame timing added to the baseline" || echo "  [warn] baseline timing patch failed (no per-frame timing)"
fi
build baseline
for s in "${SEQS[@]}"; do
  for r in $(seq 1 "$RUNS"); do run_once baseline "$s" "$r"; done
done

echo "=================== LK-v2 (competing-hypothesis motion prior) ==================="
export LK_MM_MATCH_TH
swap_to_v2
build v2
for s in "${SEQS[@]}"; do
  for r in $(seq 1 "$RUNS"); do run_once v2 "$s" "$r"; done
done

echo ""
echo "======================================================"
echo "done. Results in $OUT/"
find "$OUT" -name "*_traj.txt" | sort
echo "======================================================"

WIN_DST="${WIN_DST:-$HOME/slam_result_mirror/v2_euroc}"
if mkdir -p "$WIN_DST" 2>/dev/null; then
  mkdir -p "$WIN_DST"
  cp -r "$OUT/." "$WIN_DST/" 2>/dev/null
  cp logs/v2euroc_*.log "$WIN_DST/logs/" 2>/dev/null || true
  echo "[OK] copied to $WIN_DST"
else
  echo "[warn] could not create $WIN_DST; results remain in $OUT"
fi

echo "ALL DONE. Current code is LK-v2."