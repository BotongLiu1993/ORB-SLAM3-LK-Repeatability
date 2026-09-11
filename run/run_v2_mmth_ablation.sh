#!/usr/bin/env bash
# =============================================================================
# run_v2_mmth_ablation.sh - ablation over LK_MM_MATCH_TH (the v2 weak-motion-model trigger threshold)
# =============================================================================
# Finding: on TUM RGB-D the SearchByProjection match count is usually 100-200+,
#          so the default 25 (LK only when nmatches<25) almost never triggers; when it does trigger the adoption rate is 100%.
# Usage (from ~/ORB_SLAM3, after deploy_v2):
#   bash <path to this script> [thresholds]   default: 120 160 200 250
# e.g. bash "$W/run_v2_mmth_ablation.sh" 120 160 200 250
# Produces: results/v2_mmth_ablation/th{TH}_r{1,2}_{traj,times,lkstats,evo}.txt
# =============================================================================
set -u
cd "${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}" || { echo "[ERROR] ORB_SLAM3_ROOT/~/ORB_SLAM3 not found"; exit 1; }

grep -q "LK_MM_MATCH_TH" src/Tracking.cc || { echo "[ERROR] current code is not v2, run deploy_v2.sh first"; exit 1; }

BIN="Examples/RGB-D/rgbd_tum"
VOCAB="Vocabulary/ORBvoc.txt"
YAML="Examples/RGB-D/TUM1.yaml"
DIR="datasets/rgbd_dataset_freiburg1_desk"
ASSOC="Examples/associations/fr1_desk.txt"
OUT="results/v2_mmth_ablation"
mkdir -p "$OUT" logs
[ -f "$ASSOC" ] || { echo "[ERROR] missing association file $ASSOC"; exit 1; }

THS=(${@:-120 160 200 250})

for th in "${THS[@]}"; do
  for r in 1 2; do
    echo "---- LK_MM_MATCH_TH=$th r$r $(date +%H:%M:%S) ----"
    export LK_MM_MATCH_TH=$th
    rm -f CameraTrajectory.txt KeyFrameTrajectory.txt track_times_ms.txt
    "$BIN" "$VOCAB" "$YAML" "$DIR" "$ASSOC" > "logs/v2mab_th${th}_r${r}.log" 2>&1
    rc=$?
    echo "  exit=$rc end:$(date +%H:%M:%S)"
    [ -f CameraTrajectory.txt ] && cp CameraTrajectory.txt "$OUT/th${th}_r${r}_traj.txt" || echo "  [warn] no trajectory"
    [ -f track_times_ms.txt ] && cp track_times_ms.txt "$OUT/th${th}_r${r}_times.txt"
    sed -n '/LK Tracking Statistics/,/====/p' "logs/v2mab_th${th}_r${r}.log" > "$OUT/th${th}_r${r}_lkstats.txt" 2>/dev/null || true
    if [ -f "$OUT/th${th}_r${r}_traj.txt" ]; then
      evo_ape tum "$DIR/groundtruth.txt" "$OUT/th${th}_r${r}_traj.txt" -a \
        > "$OUT/th${th}_r${r}_evo.txt" 2>&1 || true
    fi
  done
done

echo ""
echo "================ Summary ================"
printf "%-6s %-4s %-9s %-9s %-9s %-9s %-9s\n" "th" "run" "att%" "adopt%" "weak%" "ATE_rmse" "medTime"
for th in "${THS[@]}"; do
  for r in 1 2; do
    lk="$OUT/th${th}_r${r}_lkstats.txt"
    att=$(grep -E "LK attempts" "$lk" | awk '{print $3}')
    adp=$(grep -E "LK adopted" "$lk" | awk '{print $4}')
    wk=$(grep -E "weak-MM" "$lk" | awk '{print $4}')
    frames=573
    attpct=$(awk -v a="${att:-0}" -v f=$frames 'BEGIN{printf "%.2f", 100*a/f}')
    adppct=$(awk -v a="${adp:-0}" -v n="${att:-0}" 'BEGIN{if(n>0) printf "%.1f", 100*a/n; else printf "-"}')
    rmse=$(grep -E "^[[:space:]]+rmse" "$OUT/th${th}_r${r}_evo.txt" | awk '{print $2}')
    med=$(sort -n "$OUT/th${th}_r${r}_times.txt" 2>/dev/null | awk '{a[NR]=$1} END{if(NR>0) printf "%.2f", (NR%2? a[(NR+1)/2] : (a[NR/2]+a[NR/2+1])/2)}')
    printf "%-6s %-4s %-9s %-9s %-9s %-9s %-9s\n" "$th" "r$r" "$attpct" "$adppct" "${wk:-NA}" "${rmse:-NA}" "${med:-NA}"
  done
done
echo "======================================"
echo "Details: $OUT/ (traj/times/lkstats/evo all saved)"
