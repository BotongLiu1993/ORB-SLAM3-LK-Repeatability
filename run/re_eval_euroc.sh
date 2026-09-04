#!/usr/bin/env bash
# =============================================================================
# re_eval_euroc.sh - 用 evo 复核已保存的 EuRoC 轨迹 (不重跑 SLAM)  v2
#   修正 v1 的 bug: 轨迹时间戳为纳秒(ns), 而 evo 读 .txt 按秒处理, 导致
#   "found no matching timestamps"。v2 将 GT 与轨迹统一转为 TUM(秒) 后再算。
#   GT: data.csv -> GT.tum (秒, 四元数重排为 qx qy qz qw)
#   轨迹: traj.txt (ns) -> shifted.tum (秒, 加上每序列实测固定偏移)
#   用法 (WSL):
#     bash "$W/re_eval_euroc.sh"
#   输出:
#     results/v2_euroc/re_eval/{seq}_{meth}_r{r}_{gt.tum,shift.tum,evo.txt,evo.zip}
# =============================================================================
set -uo pipefail
ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
cd "$ORB" || { echo "[ERROR] $ORB 不存在"; exit 1; }
OUT="results/v2_euroc"
mkdir -p "$OUT/re_eval"

declare -A DIR
DIR[V101]="datasets/euroc/V1_01_easy"
DIR[V102]="datasets/euroc/V1_02_medium"
DIR[V103]="datasets/euroc/V1_03_difficult"

gt_to_tum() {
  # $1=data.csv $2=输出 GT.tum ; 四元数: csv(qw qx qy qz) -> tum(qx qy qz qw), 时间 ns -> s
  python3 - "$1" "$2" <<'PY'
import sys
src, out = sys.argv[1], sys.argv[2]
with open(src) as f, open(out, "w") as g:
    for line in f:
        line = line.strip()
        if not line or line.startswith("#"): continue
        p = line.split(",")
        ts = float(p[0]) / 1e9
        x, y, z = p[1], p[2], p[3]
        qw, qx, qy, qz = p[4], p[5], p[6], p[7]
        g.write("%.9f %s %s %s %s %s %s %s\n" % (ts, x, y, z, qx, qy, qz, qw))
print("    [gt] %s -> %s" % (src, out))
PY
}

shift_to_tum() {
  # $1=轨迹(ns, tum列序) $2=输出 shifted.tum(秒) $3=GT data.csv ; 偏移 = GT首帧 - est首帧
  python3 - "$1" "$2" "$3" <<'PY'
import sys
traj, out, gtcsv = sys.argv[1], sys.argv[2], sys.argv[3]
gt0 = None
with open(gtcsv) as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith("#"): continue
        gt0 = float(line.split(",")[0]); break
with open(traj) as f:
    est0 = float(f.readline().split()[0])
off = gt0 - est0                       # ns
with open(traj) as f, open(out, "w") as g:
    for line in f:
        p = line.split()
        if not p: continue
        ts_s = (float(p[0]) + off) / 1e9
        g.write("%.9f %s %s %s %s %s %s %s\n" % (ts_s, p[1], p[2], p[3], p[4], p[5], p[6], p[7]))
print("    [shift] offset=%.6f s -> %s" % (off / 1e9, out))
PY
}

for seq in V101 V102 V103; do
  gt="${DIR[$seq]}/mav0/state_groundtruth_estimate0/data.csv"
  [ -f "$gt" ] || { echo "[warn] 缺 GT: $gt, 跳过 $seq"; continue; }
  gt_tum="$OUT/re_eval/${seq}_GT.tum"
  gt_to_tum "$gt" "$gt_tum"
  for meth in baseline v2; do
    for r in 1 2 3; do
      traj="$OUT/$meth/euroc${seq}_r${r}_traj.txt"
      [ -f "$traj" ] || { echo "[skip] $traj"; continue; }
      shft="$OUT/re_eval/${seq}_${meth}_r${r}_shift.tum"
      echo "== $seq $meth r$r =="
      shift_to_tum "$traj" "$shft" "$gt"
      evo_ape tum "$gt_tum" "$shft" -a \
          --save_results "$OUT/re_eval/${seq}_${meth}_r${r}_evo.zip" 2>&1 \
          | tee "$OUT/re_eval/${seq}_${meth}_r${r}_evo.txt" \
          | grep -E "^\s+(rmse|mean|median|max)\s"
    done
  done
done
echo "DONE. 复核结果在 $OUT/re_eval/"