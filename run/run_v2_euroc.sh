#!/usr/bin/env bash
# =============================================================================
# run_v2_euroc.sh - EuRoC MAV (纯立体, 无 IMU): LK-v2 vs 基线 (n=3 重复)
# =============================================================================
# 为什么用纯立体:
#   v2 的 LK 竞争在恒速运动模型路径上生效; EuRoC 官方 Demo 默认走 Stereo-Inertial,
#   IMU 初始化后直接 PredictStateIMU 跳过 LK。因此这里用 Examples/Stereo/EuRoC.yaml
#   (无 IMU 段) + stereo_euroc => 恒速模型路径, LK 竞争照常生效。
#   论文叙事: 方法替代的是恒速运动模型, 不是 IMU 积分, 纯立体才是公平对照。
# 数据集:
#   官方 zip (MH_01_easy ... V2_03_difficult) 解压后放进 datasets/euroc/ 即可,
#   脚本自动识别目录名 (MH_01_easy <-> MH01, V1_02_medium <-> V102)。
#   下载: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
# 前置: deploy_v2.sh 已完成 (code_backups_tum/*.baseline 与 *.bak_tum_v2 就绪)
# 用法:
#   nohup bash run_v2_euroc.sh > logs/v2euroc_outer.log 2>&1 &   # 全部 11 序列
#   bash run_v2_euroc.sh MH01 V102                                # 只跑指定序列
# 结果:
#   results/v2_euroc/{baseline,v2}/euroc{seq}_r{n}_{traj,evo,lkstats,rotstats,timing}
#   结束自动复制到 /mnt/d/0 科研学习/SLAM/result/evo_results/v2_euroc/
# =============================================================================
set -uo pipefail

ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
cd "$ORB" || { echo "[ERROR] $ORB 不存在"; exit 1; }

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

# deploy_v2.sh 未备份 stereo_euroc.cc (它基线/v2 两侧相同, 只关 Viewer);
# 其余文件从 code_backups_tum/*.baseline 恢复。
CORE_FILES=(src/Tracking.cc src/System.cc include/Tracking.h include/System.h \
            Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc)
CLEAN_FILES=(include/System.h Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc)

patch_viewer_off() {
  sed -i 's/System::STEREO, *true)/System::STEREO, false)/' Examples/Stereo/stereo_euroc.cc
  grep -q "System::STEREO, false)" Examples/Stereo/stereo_euroc.cc \
    || echo "[warn] stereo_euroc.cc 的 Viewer 未成功关闭"
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

# 解析数据集目录: 优先 datasets/euroc/<seq>, 否则自动识别官方命名 (MH_01_easy -> MH01)
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
  command -v evo_ape >/dev/null 2>&1 || { echo "[ERROR] 缺 evo_ape"; exit 1; }
  if ! grep -q "LK_MM_MATCH_TH" src/Tracking.cc; then
    if [ -f src/Tracking.cc.bak_tum_v2 ] && grep -q "LK_MM_MATCH_TH" src/Tracking.cc.bak_tum_v2; then
      echo "  [warn] 当前代码非 v2, 检测到 v2 快照 (*.bak_tum_v2), 自动恢复后继续"
    else
      echo "[ERROR] 既非 v2 代码也无 v2 快照 (先运行 deploy_v2.sh)"; exit 1
    fi
  fi
  [ -f "$YAML" ] || { echo "[ERROR] 缺 $YAML"; exit 1; }
  local keep=() s dir ts
  for s in "${SEQS[@]}"; do
    dir=$(resolve_seq_dir "$s") || { echo "[warn] 跳过缺失数据集 $s (解压到 datasets/euroc/ 后重跑)"; continue; }
    ts="Examples/Stereo/EuRoC_TimeStamps/$s.txt"
    [ -f "$ts" ] || { echo "[warn] 缺时间戳文件 $ts, 跳过 $s"; continue; }
    keep+=("$s")
  done
  SEQS=("${keep[@]}")
  [ ${#SEQS[@]} -gt 0 ] || { echo "[ERROR] 没有可运行的序列"; exit 1; }
}

build() {
  local tag=$1
  if [ ! -f Thirdparty/g2o/lib/libg2o.so ]; then
    echo "  [g2o] 未找到 libg2o.so, 重建中 ..."
    ( cd Thirdparty/g2o/build && make -j4 > "/tmp/build_g2o_${tag}.log" 2>&1 ); rc=$?
    [ $rc -ne 0 ] && { echo "[ERROR] g2o 重建失败:"; tail -20 "/tmp/build_g2o_${tag}.log"; exit 1; }
    echo "  [g2o] libg2o.so 就绪"
  fi
  echo "  [make] 编译 $tag (stereo_euroc) ..."
  ( cd build && make -j4 stereo_euroc > "/tmp/build_v2euroc_${tag}.log" 2>&1 ); rc=$?
  if [ $rc -ne 0 ]; then
    echo "[ERROR] $tag 编译失败:"
    grep -n -i -B2 -A2 "error" "/tmp/build_v2euroc_${tag}.log" | head -40
    exit 1
  fi
  echo "  [make] $tag 编译完成"
}

run_once() {
  local tag=$1 s=$2 r=$3
  local seqdir ts gt name
  seqdir=$(resolve_seq_dir "$s")
  ts="Examples/Stereo/EuRoC_TimeStamps/$s.txt"
  gt="$seqdir/mav0/state_groundtruth_estimate0"
  gtcsv="$gt/data.csv"
  name="euroc${s}"
  echo "---- [$tag] $s r$r 开始:$(date +%H:%M:%S) ----"
  rm -f "f_${name}.txt" "kf_${name}.txt" track_times_ms.txt
  "$BIN" "$VOCAB" "$YAML" "$seqdir" "$ts" "$name" > "logs/v2euroc_${s}_${tag}_r${r}.log" 2>&1
  local rc=$?
  echo "  exit=$rc 结束:$(date +%H:%M:%S)"
  [ -f "f_${name}.txt" ] && cp "f_${name}.txt" "$OUT/$tag/${name}_r${r}_traj.txt" || echo "  [warn] 无相机轨迹"
  [ -f "kf_${name}.txt" ] && cp "kf_${name}.txt" "$OUT/$tag/${name}_r${r}_kf_traj.txt" || true
  [ -f track_times_ms.txt ] && cp track_times_ms.txt "$OUT/$tag/${name}_r${r}_times.txt" || echo "  [warn] 无逐帧耗时"
  sed -n '/Tracking Time Statistics/,/====/p' "logs/v2euroc_${s}_${tag}_r${r}.log" > "$OUT/$tag/${name}_r${r}_timing.txt" 2>/dev/null || true
  if [ "$tag" = v2 ]; then
    sed -n '/LK Tracking Statistics/,/====/p' "logs/v2euroc_${s}_${tag}_r${r}.log" > "$OUT/$tag/${name}_r${r}_lkstats.txt" 2>/dev/null || true
    sed -n '/LK Rotation Deviation/,/====/p' "logs/v2euroc_${s}_${tag}_r${r}.log" > "$OUT/$tag/${name}_r${r}_rotstats.txt" 2>/dev/null || true
  fi
  if [ -f "$OUT/$tag/${name}_r${r}_traj.txt" ]; then
    rm -f "$OUT/$tag/${name}_r${r}_evo.zip"   # 避免 evo 覆盖提示在 nohup 下失败
    if [ ! -f "$gtcsv" ]; then
      echo "  [warn] GT 缺失: $gtcsv (轨迹已保存, 补数据后可重跑 evo)"
    fi
    evo_ape euroc "$gtcsv" "$OUT/$tag/${name}_r${r}_traj.txt" -a \
        --save_results "$OUT/$tag/${name}_r${r}_evo.zip" 2>&1 \
        | tee "$OUT/$tag/${name}_r${r}_evo.txt"
  fi
}

# ---------------- 主流程 ----------------
check
echo "======================================================"
echo "EuRoC stereo-only: baseline vs LK-v2 (LK_MM_MATCH_TH=$LK_MM_MATCH_TH)"
echo "序列: ${SEQS[*]}  重复: n=$RUNS"
echo "======================================================"

echo "=================== 基线 (原始 ORB-SLAM3) ==================="
swap_to_baseline
TIMING_PY="${0%/*}/apply_timing_only_patch.py"
[ -f "$TIMING_PY" ] || TIMING_PY="apply_timing_only_patch.py"
if [ -f "$TIMING_PY" ]; then
  python3 "$TIMING_PY" && echo "  [ok] 基线已加逐帧计时" || echo "  [warn] 基线计时补丁失败(无逐帧耗时)"
fi
build baseline
for s in "${SEQS[@]}"; do
  for r in $(seq 1 "$RUNS"); do run_once baseline "$s" "$r"; done
done

echo "=================== LK-v2 (竞争式运动先验) ==================="
export LK_MM_MATCH_TH
swap_to_v2
build v2
for s in "${SEQS[@]}"; do
  for r in $(seq 1 "$RUNS"); do run_once v2 "$s" "$r"; done
done

echo ""
echo "======================================================"
echo "完成. 结果在 $OUT/"
find "$OUT" -name "*_traj.txt" | sort
echo "======================================================"

WIN_DST="/mnt/d/0 科研学习/SLAM/result/evo_results/v2_euroc"
if [ -d /mnt/d ]; then
  mkdir -p "$WIN_DST"
  cp -r "$OUT/." "$WIN_DST/" 2>/dev/null
  cp logs/v2euroc_*.log "$WIN_DST/logs/" 2>/dev/null || true
  echo "[OK] 已复制到 $WIN_DST"
else
  echo "[warn] /mnt/d 未挂载, 结果保留在 $OUT"
fi

echo "ALL DONE. 当前代码为 LK-v2."