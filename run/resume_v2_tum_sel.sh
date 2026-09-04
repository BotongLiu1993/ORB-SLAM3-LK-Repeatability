#!/usr/bin/env bash
# =============================================================================
# resume_v2_tum_sel.sh - 断电恢复: 只跑 LK-v2 阶段 (基线结果已落盘, 跳过)
#   用法 (WSL):
#     bash "$W/resume_v2_tum_sel.sh" > logs/v2tum_resume_outer.log 2>&1 &
#     或前台: bash "$W/resume_v2_tum_sel.sh"
#   幂等: 已有 results/v2_tum/v2/{seq}_r{r}_traj.txt 的轮次自动跳过。
#   结果: results/v2_tum/v2/ + 结束自动复制到 /mnt/d/0 科研学习/SLAM/result/...
# =============================================================================
set -uo pipefail
ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
cd "$ORB" || { echo "[ERROR] $ORB 不存在"; exit 1; }

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
  grep -q "System::RGBD, false)" Examples/RGB-D/rgbd_tum.cc || echo "[warn] rgbd_tum.cc 的 Viewer 未成功关闭"
  grep -q "System::STEREO, false)" Examples/Stereo/stereo_kitti.cc || echo "[warn] stereo_kitti.cc 的 Viewer 未成功关闭"
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
    echo "[ERROR] 关联文件生成失败: $seq (请检查 ${SEQ_DIR[$seq]}/rgb.txt 与 depth.txt)"
    return 1
  fi
  echo "  [assoc] 已生成 $f"
}
build() {
  local tag=$1
  if [ ! -f Thirdparty/g2o/lib/libg2o.so ]; then
    echo "  [g2o] 未找到 libg2o.so, 重建中 ..."
    ( cd Thirdparty/g2o/build && make -j4 > "/tmp/build_g2o_${tag}.log" 2>&1 ); rc=$?
    [ $rc -ne 0 ] && { echo "[ERROR] g2o 重建失败:"; tail -20 "/tmp/build_g2o_${tag}.log"; exit 1; }
    echo "  [g2o] libg2o.so 就绪"
  fi
  echo "  [make] 编译 $tag ..."
  ( cd build && make -j4 rgbd_tum > "/tmp/build_v2tum_${tag}.log" 2>&1 ); rc=$?
  if [ $rc -ne 0 ]; then
    echo "[ERROR] $tag 编译失败:"
    grep -n -i -B2 -A2 "error" "/tmp/build_v2tum_${tag}.log" | head -40
    exit 1
  fi
  echo "  [make] $tag 编译完成"
}
run_once() {
  local tag=$1 seq=$2 r=$3
  local dir="${SEQ_DIR[$seq]}" yaml="${YAML[$seq]}" assoc="Examples/associations/${seq}.txt"
  make_assoc "$seq" || { echo "  [skip] $seq (关联文件生成失败, 跳过)"; return 0; }
  echo "---- [$tag] $seq r$r 开始:$(date +%H:%M:%S) ----"
  rm -f CameraTrajectory.txt KeyFrameTrajectory.txt track_times_ms.txt
  "$BIN" "$VOCAB" "$yaml" "$dir" "$assoc" > "logs/v2tum_${seq}_${tag}_r${r}.log" 2>&1
  local rc=$?
  echo "  exit=$rc 结束:$(date +%H:%M:%S)"
  [ -f CameraTrajectory.txt ] && cp CameraTrajectory.txt "$OUT/$tag/${seq}_r${r}_traj.txt" || echo "  [warn] 无相机轨迹"
  [ -f KeyFrameTrajectory.txt ] && cp KeyFrameTrajectory.txt "$OUT/$tag/${seq}_r${r}_kf_traj.txt" || true
  [ -f track_times_ms.txt ] && cp track_times_ms.txt "$OUT/$tag/${seq}_r${r}_times.txt" || echo "  [warn] 无逐帧耗时"
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

# ---------------- 主流程 ----------------
command -v evo_ape >/dev/null 2>&1 || { echo "[ERROR] 缺 evo_ape"; exit 1; }
if ! grep -q "LK_MM_MATCH_TH" src/Tracking.cc; then
  if [ -f src/Tracking.cc.bak_tum_v2 ] && grep -q "LK_MM_MATCH_TH" src/Tracking.cc.bak_tum_v2; then
    echo "  [warn] 当前代码非 v2, 检测到 v2 快照, 自动恢复"
  else
    echo "[ERROR] 既非 v2 代码也无 v2 快照 (先运行 deploy_v2.sh)"; exit 1
  fi
fi
for s in "${SEQS[@]}"; do
  if [ ! -d "${SEQ_DIR[$s]}" ]; then echo "[warn] 缺数据集 ${SEQ_DIR[$s]}"; fi
done

echo "=================== LK-v2 (恢复模式: 基线已完成, 跳过) ==================="
export LK_MM_MATCH_TH
echo "  [gate] LK_MM_MATCH_TH=$LK_MM_MATCH_TH"
swap_to_v2
build v2
for seq in "${SEQS[@]}"; do
  for r in 1 2 3; do
    if [ -f "$OUT/v2/${seq}_r${r}_traj.txt" ] && [ -s "$OUT/v2/${seq}_r${r}_traj.txt" ]; then
      echo "[skip] v2 $seq r$r (已有轨迹)"
      continue
    fi
    run_once v2 "$seq" "$r"
  done
done

echo ""
echo "======================================================"
echo "完成. 结果在 $OUT/"
find "$OUT" -name "*_traj.txt" | sort
echo "======================================================"

WIN_DST="/mnt/d/0 科研学习/SLAM/result/evo_results/tum_experiments/v2_tum"
if [ -d /mnt/d ]; then
  mkdir -p "$WIN_DST"
  cp -r "$OUT/." "$WIN_DST/" 2>/dev/null
  cp logs/v2tum_*.log "$WIN_DST/logs/" 2>/dev/null || true
  echo "[OK] 已复制到 $WIN_DST"
else
  echo "[warn] /mnt/d 未挂载, 结果保留在 $OUT"
fi

echo "ALL DONE. 当前代码为 LK-v2."