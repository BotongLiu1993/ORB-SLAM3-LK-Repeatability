#!/usr/bin/env bash
# =============================================================================
# deploy_v2.sh - 部署 LK-v2 (竞争式运动先验) 到 ORB-SLAM3
# =============================================================================
# 用法:
#   bash deploy_v2.sh [v2文件目录]
#     - 默认从脚本所在目录找 Tracking_v2.cc / Tracking_v2.h
#     - 例: bash deploy_v2.sh /mnt/d/0\ 科研学习/SLAM/result/scripts/lk_v2
# 作用:
#   1) 备份当前工作区代码为 *.bak_tum_v1 (若含 v1 标记)
#   2) 用 git show HEAD 生成 baseline 备份 (code_backups_tum/*.baseline)
#   3) 复制 v2 代码到 src/ include/
#   4) 编译 rgbd_tum 与 stereo_kitti
#   5) 自检 v2 标记
# =============================================================================
set -uo pipefail

ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
cd "$ORB" || { echo "[ERROR] $ORB 不存在"; exit 1; }

V2DIR="${1:-$(cd "$(dirname "$0")" && pwd)}"
FILES=(src/Tracking.cc src/System.cc include/Tracking.h include/System.h Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc)
mkdir -p code_backups_tum

# ---- 1) 备份当前代码 (若是 v1 版) ----
if grep -q "LK_CONSISTENCY_THRESHOLD" src/Tracking.cc; then
  for f in "${FILES[@]}"; do cp -f "$f" "${f}.bak_tum_v1"; done
  echo "[bak] 当前 v1 LK 代码已备份为 *.bak_tum_v1"
elif grep -q "LK_MM_MATCH_TH" src/Tracking.cc; then
  for f in "${FILES[@]}"; do cp -f "$f" "${f}.bak_tum_v2"; done
  echo "[bak] 当前 v2 代码已备份为 *.bak_tum_v2"
else
  echo "[bak] 当前为基线代码 (无备份, 无需)"
fi

# ---- 2) baseline 备份 (git HEAD) ----
for f in "${FILES[@]}"; do
  b="code_backups_tum/$(basename "$f").baseline"
  git show HEAD:"$f" > "$b" 2>/dev/null || { echo "[ERROR] git show HEAD:$f 失败"; exit 1; }
done
echo "[ok] baseline 备份就绪 (code_backups_tum/*.baseline)"

# ---- 3) 复制 v2 代码 ----
for pair in "src/Tracking.cc:Tracking_v2.cc" "include/Tracking.h:Tracking_v2.h"; do
  dst="${pair%%:*}"; srcname="${pair##*:}"
  if [ ! -f "$V2DIR/$srcname" ]; then
    echo "[ERROR] 缺 $V2DIR/$srcname"; exit 1
  fi
  cp -f "$V2DIR/$srcname" "$dst"
done
echo "[ok] v2 代码已复制"
# System.cc 先恢复基线, 稍后 (3a2) 会打上 Shutdown() 打印 LK 统计的补丁; 示例程序保持纯净版 (headless)
for f in src/System.cc include/System.h Examples/RGB-D/rgbd_tum.cc Examples/Stereo/stereo_kitti.cc; do
  cp -f "code_backups_tum/$(basename "$f").baseline" "$f" || { echo "[ERROR] 缺 $f 的 baseline"; exit 1; }
done
echo "[ok] System/示例程序已恢复为纯净版"
# 关闭 Pangolin Viewer (headless 运行, 不影响轨迹与耗时统计)
sed -i 's/System::RGBD, *true)/System::RGBD, false)/' Examples/RGB-D/rgbd_tum.cc
sed -i 's/System::STEREO, *true)/System::STEREO, false)/' Examples/Stereo/stereo_kitti.cc
echo "[ok] 示例程序已关闭 Viewer (headless)"
grep -q "LK_MM_MATCH_TH" src/Tracking.cc || { echo "[ERROR] v2 标记缺失, 中止"; exit 1; }

# ---- 3b) 快照 v2 代码 (run_v2_*.sh 的 swap_to_v2 依赖 *.bak_tum_v2) ----
# ---- 3a2) v2 System.cc: Shutdown() prints LK stats ----
# This ORB-SLAM3 fork has NO System destructor, so Tracking::~Tracking()
# never runs; System::Shutdown() is the last reliable hook (S4-era mechanism).
python3 - <<'PYEOF' || { echo "[ERROR] System.cc Shutdown patch failed"; exit 1; }
import io
p = "src/System.cc"
s = io.open(p, "r", encoding="utf-8").read()
if "mpTracker->PrintLKStats();" not in s:
    old = "#ifdef REGISTER_TIMES\n    mpTracker->PrintTimeStats();\n#endif\n"
    assert s.count(old) == 1, "Shutdown REGISTER_TIMES anchor"
    add = old + (\
        "\n"
        "    // LK-v2: print adoption/validation/timing statistics at shutdown.\n"
        "    // (This ORB-SLAM3 fork has no System destructor, so the Tracking\n"
        "    // destructor is never reached; Shutdown() is the last reliable hook.)\n"
        "    mpTracker->PrintLKStats();\n"
    )
    s = s.replace(old, add, 1)
    io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("[ok] System.cc Shutdown() now prints LK stats")
PYEOF

for f in "${FILES[@]}"; do cp -f "$f" "${f}.bak_tum_v2"; done
echo "[bak] v2 快照已保存为 *.bak_tum_v2 (供实验脚本切换)"

# ---- 3c) g2o 库检查 (make clean 会清掉它; 缺失时自动重建) ----
if [ ! -f Thirdparty/g2o/lib/libg2o.so ]; then
  echo "[g2o] 未找到 Thirdparty/g2o/lib/libg2o.so, 重建中 ..."
  ( cd Thirdparty/g2o/build && make -j4 > /tmp/build_g2o.log 2>&1 ); rc=$?
  if [ $rc -ne 0 ]; then echo "[ERROR] g2o 重建失败:"; tail -20 /tmp/build_g2o.log; exit 1; fi
  echo "[g2o] libg2o.so 就绪"
fi
# ---- 4) 编译 ----
( cd build && make -j4 rgbd_tum stereo_kitti > /tmp/build_v2.log 2>&1 ); rc=$?
if [ $rc -ne 0 ]; then
  echo "[ERROR] 编译失败 (rc=$rc):"
  grep -n -i -B2 -A2 "error" /tmp/build_v2.log | head -60
  exit 1
fi
echo "[make] rgbd_tum + stereo_kitti 编译完成"

# ---- 5) 自检 ----
echo "[check] v2 标记:"
grep -c "LK_MM_MATCH_TH" src/Tracking.cc
grep -c "LK_FB_TH" src/Tracking.cc
echo "ALL DONE. 当前代码为 LK-v2."
