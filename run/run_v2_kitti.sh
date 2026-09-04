#!/usr/bin/env bash
# =============================================================================
# run_v2_kitti.sh - KITTI stereo: LK-v2 vs 基线 (n=3 重复)
# =============================================================================
# 序列: 00 02 05 07 09 10 (含急弯/环路; 02 有强转弯与环路)
# 前置: deploy_v2.sh 已完成; 数据集在 datasets/kitti/dataset/sequences/{00..10}
# 用法:
#   nohup bash run_v2_kitti.sh > logs/v2kitti_outer.log 2>&1 &
# 结果:
#   results/v2_kitti/{baseline,v2}/kitti{seq}_r{n}_{traj,evo,lkstats,...}
#   结束自动复制到 /mnt/d/0 科研学习/SLAM/result/evo_results/v2_kitti/
# =============================================================================
set -uo pipefail

ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
cd "$ORB" || { echo "[ERROR] $ORB 不存在"; exit 1; }

BIN="Examples/Stereo/stereo_kitti"
VOCAB="Vocabulary/ORBvoc.txt"
OUT="results/v2_kitti"
mkdir -p "$OUT/baseline" "$OUT/v2" logs code_backups_tum

SEQS=(00 02 05 07 09 10)
RUNS=3
FILES=(src/Tracking.cc src/System.cc include/Tracking.h include/System.h Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc)

declare -A YAML
YAML[00]="Examples/Stereo/KITTI00-02.yaml"
YAML[02]="Examples/Stereo/KITTI00-02.yaml"
YAML[05]="Examples/Stereo/KITTI04-12.yaml"
YAML[07]="Examples/Stereo/KITTI04-12.yaml"
YAML[09]="Examples/Stereo/KITTI04-12.yaml"
YAML[10]="Examples/Stereo/KITTI04-12.yaml"

patch_viewer_off() {
  # 无 X 显示时运行 (WSL 无 GUI/SSH 场景); 不影响轨迹与耗时统计
  sed -i 's/System::RGBD, *true)/System::RGBD, false)/' Examples/RGB-D/rgbd_tum.cc
  sed -i 's/System::STEREO, *true)/System::STEREO, false)/' Examples/Stereo/stereo_kitti.cc
  grep -q "System::RGBD, false)" Examples/RGB-D/rgbd_tum.cc || echo "[warn] rgbd_tum.cc 的 Viewer 未成功关闭"
  grep -q "System::STEREO, false)" Examples/Stereo/stereo_kitti.cc || echo "[warn] stereo_kitti.cc 的 Viewer 未成功关闭"
}
swap_to_baseline() { local f; for f in "${FILES[@]}"; do cp -f "code_backups_tum/$(basename "$f").baseline" "$f"; done; patch_viewer_off; }
swap_to_v2() {
  cp -f src/Tracking.cc.bak_tum_v2 src/Tracking.cc
  cp -f include/Tracking.h.bak_tum_v2 include/Tracking.h
  cp -f src/System.cc.bak_tum_v2 src/System.cc
  grep -q "PrintLKStats();" src/System.cc || echo "  [warn] System.cc missing Shutdown print, redeploy needed"
  # v2 的 LK 统计由 System::Shutdown() 打印, System.cc 使用带补丁的 v2 快照
  for f in include/System.h Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc; do
    cp -f "code_backups_tum/$(basename "$f").baseline" "$f"
  done
  patch_viewer_off
}
check() {
  local keep=()
  for s in "${SEQS[@]}"; do
    if [ -d "datasets/kitti/dataset/sequences/$s" ]; then keep+=("$s"); else echo "[warn] 跳过缺失数据集 sequences/$s"; fi
  done
  SEQS=("${keep[@]}")
  [ ${#SEQS[@]} -gt 0 ] || { echo "[ERROR] 没有可运行的序列"; exit 1; }
  command -v evo_ape >/dev/null 2>&1 || { echo "[ERROR] 缺 evo_ape"; exit 1; }
  for s in "${SEQS[@]}"; do
    [ -f "${YAML[$s]}" ] || { echo "[ERROR] 缺配置文件 ${YAML[$s]} (用于 kitti$s)"; exit 1; }
  done
  if ! grep -q "LK_MM_MATCH_TH" src/Tracking.cc; then
    if [ -f src/Tracking.cc.bak_tum_v2 ] && grep -q "LK_MM_MATCH_TH" src/Tracking.cc.bak_tum_v2; then
      echo "  [warn] 当前代码非 v2, 检测到 v2 快照 (*.bak_tum_v2), 自动恢复后继续"
    else
      echo "[ERROR] 既非 v2 代码也无 v2 快照 (先运行 deploy_v2.sh)"; exit 1
    fi
  fi
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
  ( cd build && make -j4 stereo_kitti > "/tmp/build_v2kitti_${tag}.log" 2>&1 ); rc=$?
  if [ $rc -ne 0 ]; then
    echo "[ERROR] $tag 编译失败:"
    grep -n -i -B2 -A2 "error" "/tmp/build_v2kitti_${tag}.log" | head -40
    exit 1
  fi
  echo "  [make] $tag 编译完成"
}

run_once() {
  local tag=$1 s=$2 r=$3
  local seqdir="datasets/kitti/dataset/sequences/$s"
  local gt="datasets/kitti/dataset/poses/$s.txt"
  echo "---- [$tag] kitti$s r$r 开始:$(date +%H:%M:%S) ----"
  rm -f CameraTrajectory.txt KeyFrameTrajectory.txt track_times_ms.txt
  "$BIN" "$VOCAB" "${YAML[$s]}" "$seqdir" > "logs/v2kitti_${s}_${tag}_r${r}.log" 2>&1
  local rc=$?
  echo "  exit=$rc 结束:$(date +%H:%M:%S)"
  [ -f CameraTrajectory.txt ] && cp CameraTrajectory.txt "$OUT/$tag/kitti${s}_r${r}_cam_traj.txt" || echo "  [warn] 无相机轨迹"
  [ -f KeyFrameTrajectory.txt ] && cp KeyFrameTrajectory.txt "$OUT/$tag/kitti${s}_r${r}_kf_traj.txt" || true
  [ -f track_times_ms.txt ] && cp track_times_ms.txt "$OUT/$tag/kitti${s}_r${r}_times.txt" || echo "  [warn] 无逐帧耗时"
  sed -n '/Tracking Time Statistics/,/====/p' "logs/v2kitti_${s}_${tag}_r${r}.log" > "$OUT/$tag/kitti${s}_r${r}_timing.txt" 2>/dev/null || true
  if [ "$tag" = v2 ]; then
    sed -n '/LK Tracking Statistics/,/====/p' "logs/v2kitti_${s}_${tag}_r${r}.log" > "$OUT/$tag/kitti${s}_r${r}_lkstats.txt" 2>/dev/null || true
    sed -n '/LK Rotation Deviation/,/====/p' "logs/v2kitti_${s}_${tag}_r${r}.log" > "$OUT/$tag/kitti${s}_r${r}_rotstats.txt" 2>/dev/null || true
  fi
  if [ -f "$OUT/$tag/kitti${s}_r${r}_cam_traj.txt" ]; then
    evo_ape kitti "$gt" "$OUT/$tag/kitti${s}_r${r}_cam_traj.txt" -a \
        --save_results "$OUT/$tag/kitti${s}_r${r}_evo.zip" 2>&1 \
        | tee "$OUT/$tag/kitti${s}_r${r}_evo.txt"
  fi
}

# ---------------- 主流程 ----------------
check

echo "=================== 基线 ==================="
swap_to_baseline
TIMING_PY="${0%/*}/apply_timing_only_patch.py"   # prefer Shutdown-based version next to this script (v10_work)
[ -f "$TIMING_PY" ] || TIMING_PY="apply_timing_only_patch.py"
if [ -f "$TIMING_PY" ]; then
  python3 "$TIMING_PY" && echo "  [ok] baseline got per-frame timing" || echo "  [warn] baseline timing patch failed"
fi
build baseline
for s in "${SEQS[@]}"; do
  for r in $(seq 1 "$RUNS"); do
    if [ -s "$OUT/baseline/kitti${s}_r${r}_cam_traj.txt" ]; then echo "[skip] baseline kitti$s r$r (已有轨迹)"; continue; fi
    run_once baseline "$s" "$r"
  done
done

echo "=================== LK-v2 ==================="
swap_to_v2
build v2
for s in "${SEQS[@]}"; do
  for r in $(seq 1 "$RUNS"); do
    if [ -s "$OUT/v2/kitti${s}_r${r}_cam_traj.txt" ]; then echo "[skip] v2 kitti$s r$r (已有轨迹)"; continue; fi
    run_once v2 "$s" "$r"
  done
done

echo ""
echo "======================================================"
echo "完成. 结果在 $OUT/"
find "$OUT" -name "*_cam_traj.txt" | sort
echo "======================================================"

WIN_DST="/mnt/d/0 科研学习/SLAM/result/evo_results/v2_kitti"
if [ -d /mnt/d ]; then
  mkdir -p "$WIN_DST" "$WIN_DST/logs"
  cp -r "$OUT/." "$WIN_DST/" 2>/dev/null
  cp logs/v2kitti_*.log "$WIN_DST/logs/" 2>/dev/null || true
  echo "[OK] 已复制到 $WIN_DST"
else
  echo "[warn] /mnt/d 未挂载, 结果保留在 $OUT"
fi

echo "ALL DONE. 当前代码为 LK-v2."
