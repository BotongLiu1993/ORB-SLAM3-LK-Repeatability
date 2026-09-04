#!/usr/bin/env bash
# =============================================================================
# run_v2_tum.sh - TUM RGB-D: LK-v2 竞争式运动先验 vs 基线 (重复运行协议)
# =============================================================================
# 前置: 已运行 deploy_v2.sh (code_backups_tum/*.baseline 与 v2 代码就绪)
# 用法:
#   nohup bash run_v2_tum.sh > logs/v2tum_outer.log 2>&1 &     # 全部
#   bash run_v2_tum.sh fast                                    # 仅快运动组
#   bash run_v2_tum.sh control                                 # 仅控制组
# 结果:
#   results/v2_tum/{baseline,v2}/{seq}_r{n}_{traj,kf_traj,times,evo,lkstats,rotstats,timing}
#   结束自动复制到 /mnt/d/0 科研学习/SLAM/result/evo_results/tum_experiments/v2_tum/
# =============================================================================
set -uo pipefail

ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
cd "$ORB" || { echo "[ERROR] $ORB 不存在"; exit 1; }

PHASE="${1:-all}"
case "$PHASE" in all|fast|control|extend|sel) ;; *) echo "unknown phase: $PHASE"; exit 1 ;; esac

BIN="Examples/RGB-D/rgbd_tum"
VOCAB="Vocabulary/ORBvoc.txt"
OUT="results/v2_tum"
mkdir -p "$OUT/baseline" "$OUT/v2" logs code_backups_tum

# LK-v2 触发阈值 (v2 弱运动模型: nmatches<LK_MM_MATCH_TH 才计算 LK)
# 消融结果: 200 => 触发~21%/ 采纳~50%/ 耗时<2%; 25 (旧默认) => LK 从不触发
LK_MM_MATCH_TH="${LK_MM_MATCH_TH:-200}"

# 快运动组 (运动模型易失效, 正结果候选) 与控制组 (平滑, "不劣化" 证明)
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
  # 补跑指定序列到 n=12 (用于确认 n=3 时的小差异是噪声还是真问题)
  # 用法: bash run_v2_tum.sh extend fr1_desk2 fr2_desk
  EXT_SEQS=("${@:2}")
  [ ${#EXT_SEQS[@]} -gt 0 ] || EXT_SEQS=(fr1_desk2 fr2_desk)
  SEQS=("${EXT_SEQS[@]}")
  for s in "${SEQS[@]}"; do RUNS[$s]=12; done
fi

FILES=(src/Tracking.cc src/System.cc include/Tracking.h include/System.h Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc)

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
    if [ -d "${SEQ_DIR[$s]}" ]; then
      if [ -f "${SEQ_DIR[$s]}/rgb.txt" ] && [ -f "${SEQ_DIR[$s]}/depth.txt" ]; then
        keep+=("$s")
      else
        echo "[warn] 数据不完整(缺 rgb.txt/depth.txt), 跳过 ${SEQ_DIR[$s]}"
      fi
    else
      echo "[warn] 跳过缺失数据集 ${SEQ_DIR[$s]}"
    fi
  done
  SEQS=("${keep[@]}")
  [ ${#SEQS[@]} -gt 0 ] || { echo "[ERROR] 没有可运行的序列"; exit 1; }
  command -v evo_ape >/dev/null 2>&1 || { echo "[ERROR] 缺 evo_ape"; exit 1; }
  if ! grep -q "LK_MM_MATCH_TH" src/Tracking.cc; then
    if [ -f src/Tracking.cc.bak_tum_v2 ] && grep -q "LK_MM_MATCH_TH" src/Tracking.cc.bak_tum_v2; then
      echo "  [warn] 当前代码非 v2, 检测到 v2 快照 (*.bak_tum_v2), 自动恢复后继续"
    else
      echo "[ERROR] 既非 v2 代码也无 v2 快照 (先运行 deploy_v2.sh)"; exit 1
    fi
  fi
}
make_assoc() {
  local seq=$1 f="Examples/associations/${seq}.txt"
  [ -s "$f" ] && return 0
  mkdir -p Examples/associations
  local py="Examples/RGB-D/associate.py"
  [ -f "$py" ] || py="${0%/*}/associate.py"   # 这个 fork 缺少 associate.py, 用 v10_work 内置版本
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
  if [ "$tag" = v2 ]; then
    sed -n '/LK Tracking Statistics/,/====/p' "logs/v2tum_${seq}_${tag}_r${r}.log" > "$OUT/$tag/${seq}_r${r}_lkstats.txt" 2>/dev/null || true
    sed -n '/LK Rotation Deviation/,/====/p' "logs/v2tum_${seq}_${tag}_r${r}.log" > "$OUT/$tag/${seq}_r${r}_rotstats.txt" 2>/dev/null || true
  fi
  if [ -f "$OUT/$tag/${seq}_r${r}_traj.txt" ]; then
    rm -f "$OUT/$tag/${seq}_r${r}_evo.zip"   # 避免 evo 覆盖提示在 nohup 下失败
    evo_ape tum "${dir}/groundtruth.txt" "$OUT/$tag/${seq}_r${r}_traj.txt" -a \
        --save_results "$OUT/$tag/${seq}_r${r}_evo.zip" 2>&1 \
        | tee "$OUT/$tag/${seq}_r${r}_evo.txt"
  fi
}

# ---------------- 主流程 ----------------
check

echo "=================== 基线 (原始 ORB-SLAM3) ==================="
swap_to_baseline
TIMING_PY="${0%/*}/apply_timing_only_patch.py"   # 优先用脚本所在目录 (v10_work) 的 Shutdown 版本
[ -f "$TIMING_PY" ] || TIMING_PY="apply_timing_only_patch.py"
if [ -f "$TIMING_PY" ]; then
  python3 "$TIMING_PY" && echo "  [ok] 基线已加逐帧计时" || echo "  [warn] 基线计时补丁失败(无逐帧耗时)"
fi
build baseline
for seq in "${SEQS[@]}"; do
  for r in $(seq 1 "${RUNS[$seq]}"); do run_once baseline "$seq" "$r"; done
done

echo "=================== LK-v2 (竞争式运动先验) ==================="
export LK_MM_MATCH_TH
echo "  [gate] LK_MM_MATCH_TH=$LK_MM_MATCH_TH (nmatches<该值才触发 LK)"
swap_to_v2
build v2
for seq in "${SEQS[@]}"; do
  for r in $(seq 1 "${RUNS[$seq]}"); do run_once v2 "$seq" "$r"; done
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
