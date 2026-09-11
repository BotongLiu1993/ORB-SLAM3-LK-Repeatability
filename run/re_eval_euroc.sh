#!/usr/bin/env bash
# =============================================================================
# re_eval_euroc.sh - re-evaluate saved EuRoC trajectories with evo (no SLAM rerun), v2
#   Fixes a v1 bug: trajectory timestamps are in nanoseconds while evo treats .txt as seconds, giving
#   "found no matching timestamps". v2 converts both ground truth and trajectory to TUM (seconds) first.
#   GT: data.csv -> GT.tum (seconds, quaternion reordered to qx qy qz qw)
#   Trajectory: traj.txt (ns) -> shifted.tum (seconds, plus the fixed per-sequence offset measured empirically)
#   Usage (WSL):
#     bash "$W/re_eval_euroc.sh"
#   Output:
#     results/v2_euroc/re_eval/{seq}_{meth}_r{r}_{gt.tum,shift.tum,evo.txt,evo.zip}
# =============================================================================
set -uo pipefail
ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
cd "$ORB" || { echo "[ERROR] $ORB not found"; exit 1; }
OUT="results/v2_euroc"
mkdir -p "$OUT/re_eval"

declare -A DIR
DIR[V101]="datasets/euroc/V1_01_easy"
DIR[V102]="datasets/euroc/V1_02_medium"
DIR[V103]="datasets/euroc/V1_03_difficult"

gt_to_tum() {
  # $1=data.csv $2=output GT.tum ; quaternion: csv(qw qx qy qz) -> tum(qx qy qz qw), time ns -> s
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
  # $1=trajectory (ns, TUM column order) $2=output shifted.tum (seconds) $3=GT data.csv ; offset = GT first frame - est first frame
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
  [ -f "$gt" ] || { echo "[warn] ground truth missing: $gt, skipping $seq"; continue; }
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
echo "DONE. Re-evaluation results in $OUT/re_eval/"