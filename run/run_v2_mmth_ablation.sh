#!/usr/bin/env bash
# =============================================================================
# run_v2_mmth_ablation.sh - 消融: LK_MM_MATCH_TH (v2 弱运动模型触发阈值)
# =============================================================================
# 发现: TUM RGB-D 下 SearchByProjection 匹配数通常在 100-200+,
#       默认 25 (nmatches<25 才触发) 几乎永不触发; 一旦触发采纳率 100% 。
# 用法(在 ~/ORB_SLAM3, 需已 deploy_v2):
#   bash <本脚本路径> [阈值列表]   默认: 120 160 200 250
# 例: bash "$W/run_v2_mmth_ablation.sh" 120 160 200 250
# 产出: results/v2_mmth_ablation/th{TH}_r{1,2}_{traj,times,lkstats,evo}.txt
# =============================================================================
set -u
cd "${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}" || { echo "[ERROR] ORB_SLAM3_ROOT/~/ORB_SLAM3 不存在"; exit 1; }

grep -q "LK_MM_MATCH_TH" src/Tracking.cc || { echo "[ERROR] 当前代码非 v2, 请先运行 deploy_v2.sh"; exit 1; }

BIN="Examples/RGB-D/rgbd_tum"
VOCAB="Vocabulary/ORBvoc.txt"
YAML="Examples/RGB-D/TUM1.yaml"
DIR="datasets/rgbd_dataset_freiburg1_desk"
ASSOC="Examples/associations/fr1_desk.txt"
OUT="results/v2_mmth_ablation"
mkdir -p "$OUT" logs
[ -f "$ASSOC" ] || { echo "[ERROR] 缺少关联文件 $ASSOC"; exit 1; }

THS=(${@:-120 160 200 250})

for th in "${THS[@]}"; do
  for r in 1 2; do
    echo "---- LK_MM_MATCH_TH=$th r$r $(date +%H:%M:%S) ----"
    export LK_MM_MATCH_TH=$th
    rm -f CameraTrajectory.txt KeyFrameTrajectory.txt track_times_ms.txt
    "$BIN" "$VOCAB" "$YAML" "$DIR" "$ASSOC" > "logs/v2mab_th${th}_r${r}.log" 2>&1
    rc=$?
    echo "  exit=$rc 结束:$(date +%H:%M:%S)"
    [ -f CameraTrajectory.txt ] && cp CameraTrajectory.txt "$OUT/th${th}_r${r}_traj.txt" || echo "  [warn] 无轨迹"
    [ -f track_times_ms.txt ] && cp track_times_ms.txt "$OUT/th${th}_r${r}_times.txt"
    sed -n '/LK Tracking Statistics/,/====/p' "logs/v2mab_th${th}_r${r}.log" > "$OUT/th${th}_r${r}_lkstats.txt" 2>/dev/null || true
    if [ -f "$OUT/th${th}_r${r}_traj.txt" ]; then
      evo_ape tum "$DIR/groundtruth.txt" "$OUT/th${th}_r${r}_traj.txt" -a \
        > "$OUT/th${th}_r${r}_evo.txt" 2>&1 || true
    fi
  done
done

echo ""
echo "================ 汇总 ================"
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
echo "详情: $OUT/ (traj/times/lkstats/evo 均已保存)"
